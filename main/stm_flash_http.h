#include "stm_flash.h"

/**
 * @brief Download firmware file via HTTP and write it to the memory of STM32Fxx
 * 
 * STM32 bootloader must be first accessed before calling this function. Please see the example-stm32f030 project 
 * 
 * @param URL Firmware file URL
 * @param digest MD5 digest. 
 * 		The MD5 digest can be entered directly in ASCII ormat:  "26cf6a94fcd44c5b44e2e590ad29c308", 
 *		or as URL (first 4 characters must be "http" in order to detect this), 
 *		or as "FETCH" commanad (will download the MD5 file from firmware URL address concatenated with '.md5')
 * @param write_to_file 
 		0: Firmware will be downloaded to memory (SPIFFS partition not necessary). Can be used if the firmware file is small enough
 		1: Download the file to SPIFFS first, then do flashing.
 * @param mass_erase 0: Erase only needed flash pages. 1: Do mass erase
 * @param verify_only 0: Do normal erase/write/verify cycle. 1: Will not erase or write flash memory but only will download the firmware file and compare it to the STM32 flash contents
 *   
 * @return ESP_OK - success, ESP_FAIL - failed
 */

esp_err_t flashSTM_from_URL(const char *URL, const char * digest, uint8_t write_to_file, uint8_t mass_erase, uint8_t verify_only);
