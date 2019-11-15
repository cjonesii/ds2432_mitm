This firmware implements the Maxim DS2432 1k-Bit Protected 1-Wire EEPROM with SHA-1 Engine using a development board based around [STM32 F1 series micro-controller](http://www.st.com/web/en/catalog/mmc/FM141/SC1169/SS1031).

project
=======

summary
-------

Maxim DS2432 1k-Bit Protected 1-Wire EEPROM with SHA-1 Engine based on the datasheet

technology
----------

This is a example application using the 1-Wire slave library

Following DS2432 features are not implemented:

- PF flag
- Load First Secret function command
- Compute Next Secret function command
- Copy Scratchpad function command
- prevent reading secret memory by returning 0xaa or 0x55

board
=====

The current implementation uses a [core board](https://wiki.cuvoodoo.info/doku.php?id=stm32f1xx#core_board).

The underlying template also supports following board:

- [Maple Mini](http://leaflabs.com/docs/hardware/maple-mini.html), based on a STM32F103CBT6
- [System Board](https://wiki.cuvoodoo.info/doku.php?id=stm32f1xx#system_board), based on a STM32F103C8T6
- [blue pill](ihttps://wiki.cuvoodoo.info/doku.php?id=stm32f1xx#blue_pill), based on a STM32F103C8T6
- [core board](https://wiki.cuvoodoo.info/doku.php?id=stm32f1xx#core_board), based on a STM32F103C8T6

**Which board is used is defined in the Makefile**.
This is required to map the user LED and button provided on the board

connections
===========

Connect the peripherals the following way (STM32F10X signal; STM32F10X pin; peripheral pin; peripheral signal; comment):

- *list board to preipheral pin connections*

All pins are configured using `define`s in the corresponding source code.

code
====

dependencies
------------

The source code uses the [libopencm3](http://libopencm3.org/) library.
The projects is already a git submodules.
It will be initialized when compiling the firmware.
Alternatively you can run once: `git submodule init` and `git submodule update`.

firmware
--------

To compile the firmware run `rake`.

documentation
-------------

To generate doxygen documentation run `rake doc`.

flash
-----

There are two firmware images: `bootloader` and `application`.
The `bootloader` image allows to flash the `application` over USB using the DFU protocol.
The `bootloader` is started first and immediately jumps to the `application` if it is valid and the DFU mode is not forced (i.e. by pressing the user button on the board or requesting a DFU detach in the `application`).
The `application` image is the main application and is implemented in `application.c`.
It is up to the application to advertise USB DFU support (i.e. as does the provided USB CDC ACM example).

The `bootlaoder` image will be flashed using SWD (Serial Wire Debug).
For that you need an SWD adapter.
The `Makefile` uses a Black Magic Probe (per default), or a ST-Link V2 along OpenOCD software.
To flash the `booltoader` using SWD run `rake flash_booloader`.

Once the `bootloader` is flashed it is possible to flash the `application` over USB using the DFU protocol by running `rake flash`.
To force the bootloader to start the DFU mode press the user button or short a pin, depending on the board.
It is also possible to flash the `application` image using SWD by running `rake flash_application`.

debug
-----

SWD also allows to debug the code running on the micro-controller using GDB.
To start the debugging session run `rake debug`.

USB
---

The firmware offers serial communication over USART1 and USB (using the CDC ACM device class).

You can also reset the board by setting the serial width to 5 bits over USB.
To reset the board run `rake reset`.
This only works if provided USB CDC ACM is running correctly and the micro-controller isn't stuck.
