#Makefile for a host build of the receiver.

OBJS=main.o chksign_ed25519.o defec.o serdec.o hexdump.o subtitle.o hldemux.o \
		bd_emu.o blockdecode.o blkidcache_mlvl.o partemu/partemu.o bd_flatflash.o \
		 hkpackets.o powerdown.o defec_rs.o defec_parity.o bma.o ../redundancy/redundancy.o \
		bd_ropart.o 
OBJS+=../ed25519/add_scalar.o  ../ed25519/fe.o  ../ed25519/ge.o  ../ed25519/key_exchange.o  \
		../ed25519/keypair.o  ../ed25519/sc.o  ../ed25519/seed.o  ../ed25519/sha512.o  \
		../ed25519/sign.o  ../ed25519/verify.o
TARGET=recv
CFLAGS=-ggdb -I ../common -I ../micro-ecc -I ../ed25519 -I partemu \
		-Og -DHOST_BUILD  -I../redundancy
LDFLAGS=-lmbedtls -lmbedcrypto

all: $(TARGET)

sha256.o: ../sha256/sha256.c ../sha256/sha256.h
	$(CC) $(CFLAGS) -c -o $@ $<

uECC.o: ../micro-ecc/uECC.c ../micro-ecc/uECC.h
	$(CC) $(CFLAGS) -c -o $@ $<

$(TARGET): $(OBJS)
	$(CC) -o $@ $^ $(LDFLAGS)

clean:
	rm -f $(OBJS) $(TARGET)

test: bd_ropart_test

bd_ropart_test: bd_ropart.o bd_ropart_test.o partemu/partemu.o blkidcache_mlvl.o bma.o
	$(CC) -o $@ $^ $(LDFLAGS)
