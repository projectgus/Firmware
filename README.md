ESP32 AMEPD test for SHA2017Badge
=================================

Read [project documentation](https://orga.sha2017.org/index.php/Projects:Badge) and get involved.

Project updates:

* [Wiki](https://wiki.sha2017.org/w/Projects:Badge)
* [Documentation](https://wiki.sha2017.org/w/Projects:Badge/Documentation)
* [Firmware](https://github.com/SHA2017-badge/Firmware)
* [PCB](https://github.com/SHA2017-badge/PCB)
* [Pad](https://pad.sha2017.org/p/badge)
* [Changelog](CHANGELOG.md)

Based on template application for [Espressif IoT Development Framework (ESP-IDF)](https://github.com/espressif/esp-idf
). Copyright (C) 2016 Espressif Systems, licensed under the Apache License 2.0 as described in the file LICENSE.

[![Build Status](https://travis-ci.org/SHA2017-badge/Firmware.svg?branch=master)](https://travis-ci.org/SHA2017-badge/Firmware)

Debian prerequisites
--------------------

```
sudo apt-get install libncurses5-dev flex bison gperf
```

Compiling and flashing
----------------------

```
git submodule update --init --recursive
source set_env.sh
make defconfig
make flash
```

Configuration
-------------
```
make menuconfig
```

Compiling and flashing MicroPython
----------------------------------

```
cd micropython/mpy-cross
make
cd ../esp32
make deploy
cd ../..
```

Interacting via serial
----------------------
```
make monitor
```

MicroPython
-----------
```
import badge
badge.eink_init()
badge.display_picture(0,-1)
badge.ugfx_init()
badge.ugfx_demo("HACKING")
badge.thickline(1,1,100,100,badge.BLACK,10,5)
badge.flush()
```
