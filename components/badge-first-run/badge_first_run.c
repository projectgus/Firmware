#include <sdkconfig.h>

#include <string.h>
#include <fcntl.h>

#include <esp_event.h>
#include <esp_event_loop.h>
#include <esp_system.h>
#include <esp_wifi.h>
#include <esp_spi_flash.h>
#include <esp_partition.h>
#include <esp_vfs.h>
#include <esp_vfs_fat.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <nvs.h>
#include <wear_levelling.h>

#include <badge.h>
#include <badge_input.h>
#include <badge_mpr121.h>
#include <badge_eink.h>
#include <badge_eink_fb.h>
#include <badge_pins.h>
#include <badge_button.h>
#include <badge_power.h>
#include <badge_sdcard.h>
#include <font.h>
#include <sha2017_ota.h>

#include <file_reader.h>
#include <flash_reader.h>
#include <deflate_reader.h>
#include <png_reader.h>

#define TAG "badge_first_run"

uint32_t baseline_def[8] = {
	0x0138,
	0x0144,
	0x0170,
	0x0174, // guessed; 0x01c9, // too high on my badge
	0x00f0,
	0x0103,
	0x00ff,
	0x00ed,
};

const char *touch_name[8] = {
	"A",
	"B",
	"START",
	"SELECT",
	"DOWN",
	"RIGHT",
	"UP",
	"LEFT",
};

void
load_png(int x, int y, const char *filename)
{
	struct lib_file_reader *fr = lib_file_new(filename, 1024);
	if (fr == NULL)
	{
		fprintf(stderr, "file not found (or out of memory).\n");
		return;
	}

	struct lib_png_reader *pr = lib_png_new((lib_reader_read_t) &lib_file_read, fr);
	if (pr == NULL)
	{
		fprintf(stderr, "out of memory.\n");
		lib_file_destroy(fr);
		return;
	}

	int res = lib_png_load_image(pr, &badge_eink_fb[x + y*BADGE_EINK_WIDTH], 0, 0, BADGE_EINK_WIDTH-x, BADGE_EINK_HEIGHT-y, BADGE_EINK_WIDTH);
	lib_png_destroy(pr);
	lib_file_destroy(fr);

	if (res < 0)
	{
		fprintf(stderr, "failed to load image: res = %i\n", res);
		return;
	}
}

#define NUM_DISP_LINES 12
#define NO_NEWLINE 0x80
// can use the lower <n> lines for showing measurements
void
disp_line(const char *line, int flags)
{
	static int next_line = 0;
	while (1)
	{
		int height = (flags & FONT_16PX) ? 2 : 1;
		while (next_line >= NUM_DISP_LINES - height)
		{ // scroll up
			next_line--;
			memmove(badge_eink_fb, &badge_eink_fb[BADGE_EINK_WIDTH], (NUM_DISP_LINES-1)*BADGE_EINK_WIDTH);
			memset(&badge_eink_fb[(NUM_DISP_LINES-1)*BADGE_EINK_WIDTH], 0xff, BADGE_EINK_WIDTH);
		}
		int len = draw_font(badge_eink_fb, 0, 8*next_line, BADGE_EINK_WIDTH, line, (FONT_FULL_WIDTH|FONT_INVERT)^flags);
		if (height == 2)
			next_line++;
		if ((flags & NO_NEWLINE) == 0)
		{
			next_line++;
			draw_font(badge_eink_fb, 0, 8*next_line, BADGE_EINK_WIDTH, "_", FONT_FULL_WIDTH|FONT_INVERT);
		}
#ifdef CONFIG_DEBUG_ADD_DELAYS
		badge_eink_display(badge_eink_fb, DISPLAY_FLAG_LUT(2));
		vTaskDelay(1000 / portTICK_PERIOD_MS);
#endif // CONFIG_DEBUG_ADD_DELAYS

		if (len == 0 || line[len] == 0)
		{
#ifndef CONFIG_DEBUG_ADD_DELAYS
			badge_eink_display(badge_eink_fb, DISPLAY_FLAG_LUT(2));
#endif // CONFIG_DEBUG_ADD_DELAYS
			return;
		}

		line = &line[len];
	}
}

#ifdef I2C_MPR121_ADDR
void
update_mpr121_bars( const struct badge_mpr121_touch_info *ti, const uint32_t *baseline_top, const uint32_t *baseline_bottom )
{
	int y;
	for (y=0; y<8; y++) {
		int x  = ti->data[y]        - 180;
		int xu = baseline_top[y]    - 180;
		int xd = baseline_bottom[y] - 180;

		if (x > 295) x = 295;
		if (xu > 295) xu = 295;
		if (xd > 295) xd = 295;

		int pos = ( 102 + y*3 ) * (BADGE_EINK_WIDTH/8);
		memset(&badge_eink_fb[pos-(BADGE_EINK_WIDTH/8)], 0xff, (BADGE_EINK_WIDTH/8)*3);
		while (x >= 0)
		{
			badge_eink_fb[pos + (x >> 3)] &= ~( 1 << (x&7) );
			x--;
		}
		badge_eink_fb[pos - (BADGE_EINK_WIDTH/8) + (xu >> 3)] &= ~( 1 << (xu&7) );
		badge_eink_fb[pos           + (xu >> 3)] &= ~( 1 << (xu&7) );
		badge_eink_fb[pos           + (xd >> 3)] &= ~( 1 << (xd&7) );
		badge_eink_fb[pos + (BADGE_EINK_WIDTH/8) + (xd >> 3)] &= ~( 1 << (xd&7) );
	}
	badge_eink_display(badge_eink_fb, DISPLAY_FLAG_LUT(2));
#ifdef CONFIG_DEBUG_ADD_DELAYS
	vTaskDelay(100 / portTICK_PERIOD_MS);
#endif // CONFIG_DEBUG_ADD_DELAYS
}
#endif // I2C_MPR121_ADDR

static esp_err_t
badge_init_locfd(void)
{
	const esp_partition_t * part_ota1 = esp_partition_find_first(
			ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_1, "ota_1");
	if (part_ota1 == NULL)
	{
		ESP_LOGE(TAG, "ota1 partition not found.");
		return ESP_FAIL;
	}

	// allocate memory for all deflates
	struct lib_deflate_reader *dr = (struct lib_deflate_reader *) malloc(sizeof(struct lib_deflate_reader));
	if (dr == NULL)
	{
		ESP_LOGE(TAG, "failed to init deflate object.");
		return ESP_ERR_NO_MEM;
	}

	struct lib_flash_reader *fr = lib_flash_new(part_ota1, 4096);
	if (fr == NULL)
	{
		ESP_LOGE(TAG, "failed to init flash-reader.");
		return ESP_ERR_NO_MEM;
	}

	bool first_chunk = true;
	while (1)
	{
		uint32_t pk_sig;
		ssize_t res = lib_flash_read(fr, (uint8_t *) &pk_sig, 4);
		if (res == -1 || res != 4)
		{
			ESP_LOGE(TAG, "(%d) failed to read from flash.", __LINE__);
			return ESP_FAIL;
		}

		if (pk_sig == 0x04034b50)
		{ // file object
			struct {
				uint16_t version_need;
				uint16_t gp_bit_flag;
				uint16_t compr_method;
				uint16_t file_time;
				uint16_t file_date;
				uint32_t crc32;
				uint32_t compr_size;
				uint32_t uncompr_size;
				uint16_t fname_len;
				uint16_t ext_len;
			} __attribute__((packed)) local_file_header;

			res = lib_flash_read(fr, (uint8_t *) &local_file_header, sizeof(local_file_header));
			if (res == -1 || res != sizeof(local_file_header))
			{
				ESP_LOGE(TAG, "(%d) failed to read from flash.", __LINE__);
				return ESP_FAIL;
			}

			char fname[256];
			if (local_file_header.fname_len == 0)
			{
				ESP_LOGE(TAG, "filename too short.");
				return ESP_FAIL;
			}
			if (local_file_header.fname_len >= sizeof(fname)-1)
			{
				ESP_LOGE(TAG, "filename too long.");
				return ESP_FAIL;
			}

			res = lib_flash_read(fr, (uint8_t *) &fname[1], local_file_header.fname_len);
			if (res == -1 || res != local_file_header.fname_len)
			{
				ESP_LOGE(TAG, "(%d) failed to read from flash.", __LINE__);
				return ESP_FAIL;
			}
			fname[ 0 ] = '/';
			local_file_header.fname_len += 1;
			fname[ local_file_header.fname_len ] = 0;

			while (local_file_header.ext_len > 0)
			{
				uint8_t tmpbuf[16];
				int sz = local_file_header.ext_len > 16 ? 16 : local_file_header.ext_len;
				res = lib_flash_read(fr, tmpbuf, sz);
				if (res == -1 || res != sz)
				{
					ESP_LOGE(TAG, "(%d) failed to read from flash.", __LINE__);
					return ESP_FAIL;
				}
				local_file_header.ext_len -= sz;
			}

			if (fname[ local_file_header.fname_len - 1 ] == '/')
			{ // dir
				fname[ local_file_header.fname_len - 1 ] = 0;
				ESP_LOGD(TAG, "mkdir('%s')", fname);
				int err = mkdir(fname, 0755);
				if (err < 0)
				{
					ESP_LOGE(TAG, "failed to create dir '%s': %d.", fname, errno);
					return ESP_FAIL;
				}
			}
			else
			{ // file
				lib_reader_read_t reader = (lib_reader_read_t) &lib_flash_read;
				void *reader_obj = fr;

				if (local_file_header.compr_method == 8)
				{ // deflated
					lib_deflate_init(dr, reader, reader_obj);

					reader = (lib_reader_read_t) &lib_deflate_read;
					reader_obj = dr;
				}
				else if (local_file_header.compr_method != 0)
				{ // not stored
					ESP_LOGE(TAG, "unknown compression type for file '%s'.", fname);
					return ESP_FAIL;
				}

				/* copy file */
				ESP_LOGD(TAG, "open('%s', O_WRONLY|O_CREAT|O_TRUNC)", fname);
				int fd = open(fname, O_WRONLY|O_CREAT|O_TRUNC, 0644);
				if (fd < 0)
				{
					ESP_LOGE(TAG, "failed to open file '%s': %d.", fname, errno);
					return ESP_FAIL;
				}

				size_t s = local_file_header.uncompr_size;
				while (s > 0)
				{
					uint8_t buf[128];
					size_t sz = s > sizeof(buf) ? sizeof(buf) : s;
					res = reader(reader_obj, buf, sz);
					if (res == -1 || res != sz)
					{
						ESP_LOGE(TAG, "(%d) failed to read from flash.", __LINE__);
						return ESP_FAIL;
					}
					s -= sz;

					uint8_t *ptr = buf;
					while (sz > 0)
					{
						int err = write(fd, ptr, sz);
						if (err <= 0)
						{
							ESP_LOGE(TAG, "failed to write to fat.");
							return ESP_FAIL;
						}
						ptr = &ptr[err];
						sz -= err;
					}
				}

				close(fd);

				if (local_file_header.compr_method == 8)
				{ // deflated
					// check if we're at the end of the stream
					uint8_t read_end;
					ssize_t res = reader(reader_obj, &read_end, 1);
					if (res != 0) // should be 'end-of-stream'
					{
						ESP_LOGE(TAG, "(%d) failed to read from flash.", __LINE__);
						return ESP_FAIL;
					}
				}
			}
		}
		else if (pk_sig == 0x02014b50 || pk_sig == 0x06054b50)
		{ // directory object or end-of-directory object
			ESP_LOGD(TAG, "end of zip reached.");
			break;
		}
		else if (first_chunk)
		{
			ESP_LOGI(TAG, "no preseed .zip found.");
			return ESP_OK;
		}
		else
		{
			ESP_LOGE(TAG, "unknown zip object type 0x%08x.", pk_sig);
			return ESP_FAIL;
		}
		first_chunk = false;
	}

	free(dr);

	// clear first page to avoid double unpacking
	int res = spi_flash_erase_sector((part_ota1->address + 4096) / SPI_FLASH_SEC_SIZE);
	if (res != ESP_OK)
		return res;

	return ESP_OK;
}

#ifdef I2C_MPR121_ADDR
static int
wait_for_key_a(void)
{
	while (1) {
		struct badge_mpr121_touch_info ti;
		int res = badge_mpr121_get_touch_info(&ti);
		if (res != 0)
		{
			disp_line("Error: failed to read touch info!", FONT_MONOSPACE);
			return -1;
		}
		if ((ti.touch_state & 1) == 0)
			break; // touched
	}

	while (1) {
		struct badge_mpr121_touch_info ti;
		int res = badge_mpr121_get_touch_info(&ti);
		if (res != 0)
		{
			disp_line("Error: failed to read touch info!", FONT_MONOSPACE);
			return -1;
		}
		if ((ti.touch_state & 1) == 1)
			break; // released
	}

	return 0;
}
#endif // I2C_MPR121_ADDR

void
badge_first_run(void)
{
	char line[100];

	printf("first_run...\n");

	// initialize display
	esp_err_t err = badge_eink_init(BADGE_EINK_DEFAULT);
	assert( err == ESP_OK );

	err = badge_eink_fb_init();
	assert( err == ESP_OK );

	printf("white...\n");

	// start with white screen
	memset(badge_eink_fb, 0xff, BADGE_EINK_FB_LEN);
	badge_eink_display(badge_eink_fb, DISPLAY_FLAG_LUT(0));

	// add line in split-screen
	if (NUM_DISP_LINES < 16) {
		memset(&badge_eink_fb[NUM_DISP_LINES*BADGE_EINK_WIDTH], 0x00, BADGE_EINK_WIDTH/8);
	}

	printf("displaying lines...\n");

	disp_line("SHA2017-Badge", FONT_16PX);
	disp_line("",0);
	disp_line("Built on: " __DATE__ ", " __TIME__, 0);
	disp_line("Initializing and testing badge.",0);

	// if we get here, the badge is ok.
	//badge_init();
	load_png(0,0, "/media/hacking.png");
	load_png(2,2, "/media/badge_version.png");
	load_png(0,0, "/media/badge_type.png");
	badge_eink_display_greyscale(badge_eink_fb, DISPLAY_FLAG_8BITPIXEL, BADGE_EINK_MAX_LAYERS);

	while (1)
	{
		// infinite deep sleep loop
		esp_deep_sleep_start();
	}
}

void
badge_check_first_run(void)
{
  printf("check_first_run...");
	// search non-volatile storage partition
	const esp_partition_t * nvs_partition = esp_partition_find_first(
			ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_NVS, NULL);
	if (nvs_partition == NULL)
	{
		ESP_LOGE(TAG, "NVS partition not found.");
		return;
	}

	uint8_t buf[64];
	ESP_LOGD(TAG, "nvs partition address: 0x%x", nvs_partition->address);
	int res = spi_flash_read(nvs_partition->address, buf, sizeof(buf));
	if (res != ESP_OK)
	{
		ESP_LOGE(TAG, "failed to read from NVS partition: %d", res);
		return;
	}

	{
		ESP_LOGV(TAG, "nvs read:");
		int i;
		for (i=0; i<sizeof(buf); i+=16)
		{
			ESP_LOGV(TAG, "  %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
				buf[i+0], buf[i+1], buf[i+2], buf[i+3], buf[i+4], buf[i+5], buf[i+6], buf[i+7],
				buf[i+8], buf[i+9], buf[i+10], buf[i+11], buf[i+12], buf[i+13], buf[i+14], buf[i+15]
			);
		}
	}

	static const uint8_t empty[16] = {0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff};
	if (memcmp(buf, empty, 16) == 0)
	{ // nvs partition seems empty. start first_run();
		badge_first_run();

		// error occurred; erase nvs sector
		int res = spi_flash_erase_sector(nvs_partition->address / SPI_FLASH_SEC_SIZE);
		assert(res == ESP_OK);

		while (1)
		{
			// infinite loop - FIXME: replace by deep sleep
		}
	}
}
