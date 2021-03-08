
# STM32 OTA programmer component for ESP32 esp-idf framework

## Introduction

This is a STM32 programmer component used by ESP32 esp-idf framework. It is based on [laukik-hase's OTA_update_STM32_using_ESP32 project](https://github.com/laukik-hase/OTA_update_STM32_using_ESP32) but is a major rewrite on many parts.

Main enhancements:

* Fetch the STM32 firmware via HTTP (no need to upload the firmware first to ESP32's flash memory)
* Downloaded firmware can be temporarily kept in memory if the firmware file is small enough (so no need for separate SPIFFS partition) 
* MD5 checksum can be used to check the integrity of the firmware file (checksum can be downloaded via HTTP from separate file or entered directly)
* Possibility to flash firmware without using RESET and BOOT0 pins (plain UART interface is enough, though with few restrictions)
* Callback functionality to keep track of the flashing progress and errors during flashing procedure
* Better error handling

## Use case: Flashing STM32F030x6 with plain UART (without RESET/BOOT0 pins)

STM32 can be re-programmed via UART/SPI/I2C/CAN interface (in addition to the Serial Wire Debug interface). However this requires that the normal operation of the program is interrupted and execution is jumped to the bootloader which is situated in the ROM/system memory area. Normally this is done by grounding the BOOT0 pin and doing hardware reset via RESET pin.

I wanted to add OTA update functionality to the [custom firmware for the Ikea fyrtur motor module](https://github.com/mjuhanne/fyrtur-motor-board). Unfortunately the board doesn't expose BOOT0 pin so I had to solder the wire directly to the CPU pin. Also the BOOT0 and RESET pins needed to be wired to the WiFi module in addition to the 4-wire UART cable. This is not optimal, so I wanted to find a way to do OTA update using just the existing UART wiring. 

It's possible to jump to the system memory bootloader just by using software instructions, and this can be achieved using UART via user level command. Unfortunately this means also that the STM32 module has to be bootstrapped by flashing a custom firmware (which supports this 'jump to bootloader' command) via SWD interface first. Only after this can OTA updates be done using UART alone.

## Example project

**example-stm32f030/** directory has a simple project for the STM32F030 (using Fyrtur firmware as an example) that can be used as a starting point if you want to integrate OTA update into your own ESP32/STM32 project. 

By default it is in VERIFY_ONLY mode (it will download the firmware and then compare it to the content of the STM32 flash memory). Obviously this will fail if the target firmware is not flashed yet. Change the code to `#define VERIFY_ONLY 0` to enable also erase/write tasks.

A more complex implementation can be found on [the Fyrtur ESP32 WiFi project](https://github.com/mjuhanne/fyrtur-esp).

## Steps for integrating RESET/BOOT0-pinless OTA update to your STM32 firmware

* Create a memory section (e.g. in STM32F030K6TX_FLASH.ld) which holds the variables that won't be wiped out during/after reset:
```
  .noinit(NOLOAD):
    {
    . = ALIGN(4);
    KEEP(*(.noinit));
    KEEP(*(.noinit*));
    . = ALIGN(4);
    } >RAM
```

* Define a variable that will be checked during boot time if jumping to the bootloaded is desired. This variable is inserted into the 'noinit' section that will hold its value even through reset. It's just memory (not flash) so this won't obviously last a power-off/on cycle.

```
__attribute__((section(".noinit"))) uint32_t jump_to_bootloader_magic;
```

* Add the following definitions. Modify the reset vector address to suit your target STM32 chip.
```
#define SYSMEM_RESET_VECTOR             0x1FFFEC00   // STM32F030 System memory address 
#define JUMP_TO_BOOTLOADER_MAGIC_CODE   0xDEADBEEF
```

* Add following code to **SystemInit** function which will be processed straight after boot by ResetHandler:
```
    if (jump_to_bootloader_magic == JUMP_TO_BOOTLOADER_MAGIC_CODE) {
        void (*bootloader)(void) = (void (*)(void)) (*((uint32_t *) (SYSMEM_RESET_VECTOR+4)));
        jump_to_bootloader_magic = 0;
        __set_MSP(*((uint32_t *)(SYSMEM_RESET_VECTOR + 0)));
        bootloader();
    }
```

* Finally, handle the custom 'jump to bootloader' command in your own UART/SPI/I2C/CAN interface processing by setting the magic code and doing a software reset:
```
		case CMD_EXT_ENTER_BOOTLOADER:
			{
		    jump_to_bootloader_magic = JUMP_TO_BOOTLOADER_MAGIC_CODE;

		    // Warning! If RESET PIN is connected with pull-up or driven extenally, hardware reset might needed 
		    // because it's possible for NVIC_SystemReset() to hang!
		    NVIC_SystemReset();	

		    // .. not reached
			}

```

Warning! It's possible to brick your STM32 setup if OTA update is interrupted half-way. This is because your UART/SPI/... interface code might not execute anymore, making it impossible to jump to bootloader without BOOT0/RESET pins.  The only way to recover from this situation then is to connect BOOT0/RESET pins again or re-flash the STM32 via SWD interface.  

Of course another way to prevent this is to create your own bootloader (situated at the beginning of the flash memory) which would not be updated via OTA, and so accessing bootloader would always work and recovering OTA update interruption wouldn't be an issue. This is however outside of the scope of this project.




