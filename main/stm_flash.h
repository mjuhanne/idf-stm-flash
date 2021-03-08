#ifndef _STM_FLASH_H
#define _STM_FLASH_H

#include "stm_pro_mode.h"

/**
 * @brief Write the code into the flash memory of STM32Fxx
 * 
 * The data from the .bin file is written into the flash memory 
 * of the client, block-by-block 
 * 
 * @param flash_file File pointer of the .bin file to be flashed
 *   
 * @return ESP_OK - success, ESP_FAIL - failed
 */
esp_err_t writeTask(FILE *flash_file, uint32_t size);
esp_err_t writeTask_from_mem(char * buffer, uint32_t size);

/**
 * @brief Read the flash memory of the STM32Fxx, for verification
 * 
 * It reads the flash memory of the STM32 block-by-block and 
 * checks it with the data from the file (with pointer passed)
 * 
 * @param flash_file File pointer of the .bin file to be verified against
 *   
 * @return ESP_OK - success, ESP_FAIL - failed
 */
esp_err_t readTask(FILE *flash_file, uint32_t size);
esp_err_t readTask_from_mem(char * buffer, uint32_t size);


/**
 * @brief Erase the flash memory of the STM32Fxx
 * 
 * It first calculates the needed number of flash pages to erase
 * 
 * @param size Size of the firmware flash file
 *   
 * @return ESP_OK - success, ESP_FAIL - failed
 */
esp_err_t eraseTask(uint32_t size);

/**
 * @brief Flash the .bin file passed, to STM32Fxx, with read verification. Note that enterBootLoader has to be called before this
 * and endConn afterwards.
 *
 * STM32 bootloader must be first accessed before calling these functions. Please see the example-stm32f030 project 
 * 
 * @param file_name name of the .bin to be flashed
 * @param mass_erase If true then mass erase is done before flashing. If not selected, only needed pages will be erased
 * @param verify_only Used for testing (only do read and verify task)
 *   
 * @return ESP_OK - success, ESP_FAIL - failed
 */
esp_err_t flashSTM(const char *file_name, uint8_t mass_erase, uint8_t verify_only);
esp_err_t flashSTM_from_mem(char * buffer, uint32_t size, uint8_t mass_erase, uint8_t verify_only);

// Set STM32 flash page size (must be set if mass erase is not used)
void setFlashPageSize(uint32_t page_size);

//Initialize SPIFFS functionalities
int initSPIFFS(void);


#endif
