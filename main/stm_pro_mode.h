#ifndef _STM_PRO_MODE_H
#define _STM_PRO_MODE_H


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "driver/uart.h"
#include "driver/gpio.h"

#include "esp_log.h"

//Macro for error checking
#define IS_ESP_OK(x) if ((x) != ESP_OK) break;

#define UART_BUF_SIZE 1024
#define UART_CONTROLLER UART_NUM_1

#define FLASH_BLOCK_SIZE 256
#define HIGH 1
#define LOW 0

#define ACK 0x79
#define NACK 0x1F
#define SERIAL_TIMEOUT 5000

#define FILE_PATH_MAX 128
#define BASE_PATH "/spiffs/"

typedef enum {
	// Using RESET and BOOT0 pins to enter bootloader
	STM_BOOTLOADER_METHOD_HARDWARE = 0,

	// User application has instructed STM32 to enter bootloader via custom UART command. 
	// It is assumed that bootloader is already entered and it is waiting for SYNC command, which will be done first (no HW reset).
	STM_BOOTLOADER_METHOD_SOFTWARE = 1
} bootloader_method_t;

typedef enum {
	FLASH_CONFIG = 0,
    FLASH_DOWNLOAD,
    FLASH_DOWNLOAD_MD5,
    FLASH_SYNC,
    FLASH_ERASE,
    FLASH_WRITE,
    FLASH_READ
} flash_task_t;

typedef enum {
    FLASH_STAGE_START = 0,
    FLASH_STAGE_CONTINUE,
    FLASH_STAGE_COMPLETE,
    FLASH_STAGE_ERROR,
    FLASH_STAGE_LOG_INFO,
    FLASH_STAGE_LOG_DEBUG,
    FLASH_STAGE_LOG_ERROR
} flash_stage_t;

#define FLASH_CALLBACK_TYPE void (*func_ptr)(flash_task_t task, flash_stage_t stage, int block, int max_blocks, const char * msg ) 

// Set progress notification callback
void setFlashCallback( FLASH_CALLBACK_TYPE );

//Progress notification using callbakc
void flashNotification( flash_task_t task, flash_stage_t stage, int block, int max_blocks, const char * msg );

// Log using callback
void flashLogInfo( flash_task_t task, const char* format, ... );
void flashLogError( flash_task_t task, const char* format, ... );
void flashLogDebug( flash_task_t task, const char* format, ... );

// Get task name as a text string
const char * flash_task2txt( flash_task_t task );

//Initialize UART functionalities
int initFlashUART(uint8_t txd_pin, uint8_t rxd_pin, uint32_t baud_rate);

// Just reconfigure the UART settings
int configFlashUart(uint32_t baud_rate);

//Initialize GPIO functionalities
int initSTM32_GPIO(uint8_t reset_pin, uint8_t boot0_pin);

//Reset the STM32Fxx
int resetSTM(void);

//Increment the memory address for the next write operation
void incrementLoadAddress(char *loadAddr);

//End the connection with STM32Fxx
// if jump_addr is 0, then do hardware reset. If != 0, jump programmatically to given address
void endConn( uint32_t jump_addr );

//Get in sync with STM32Fxx
int cmdSync(void);

//Get the version and the allowed commands supported by the current version of the bootloader
int cmdGet(void);

//Get the bootloader version and the Read Protection status of the Flash memory
int cmdVersion(void);

//Get the chip ID
int cmdId(void);

// Mass erase of Flash memory pages
int cmdMassErase(void);

// Mass extended erase of Flash memory pages (2-byte addressing mode)
int cmdExtMassErase(void);

// Mass erases specified pages either with normal or extended erase command. Note that before this CmdGet must be called to get which one is supported
int cmdMassErasePages(void);

//Erase from one to all the Flash memory pages using standard (1-byte addressing mode)
int cmdErase(uint8_t startPage, uint8_t count);

//Erases Flash memory pages using 2-byte addressing mode
int cmdExtErase(uint16_t startPage, uint16_t count);

//Erases specified pages either with normal or extended erase command. Note that before this CmdGet must be called to get which one is supported
int cmdErasePage( uint16_t startPage, uint16_t count );

// Jump to address
int cmdGo(uint32_t addr);

// Query the bootloader for more detailed info (executes cmdGet, cmdVersion and cmdId)
int queryBootLoader();

/* Setup STM32Fxx for the 'flashing' process
 * @param method STM_BOOTLOADER_METHOD_HARDWARE -> RESET and BOOT0 pins will be used to enter bootloader. Otherwise it is assumed that  
 * boot loader has been jumped to already in software and this function just does the final SYNC process.
 * 
 * @param bootLoaderDelay (in ms) before bootloader is ready. 
 * 
 * Bootloader delay depends on processor, HSE/LSE installation etc. It can be as long as 2560 milliseconds 
 * on STM32L476RG without LSE! Please see Application note AN2606 for more details.
*/
int enterBootLoader(bootloader_method_t method, int bootLoaderDelay);

//Write data to flash memory address
int cmdWrite(void);

//Read data from flash memory address
int cmdRead(void);

//UART send data to STM32Fxx & wait for response (ignore received data except for ACK)
int sendBytes(flash_task_t task, const char *bytes, int count, int resp);

//UART send data to STM32Fxx & wait for response
int sendReceiveBytes(flash_task_t task, const char *bytes, int count, char * recv_bytes, int resp);

//UART send data byte-by-byte to STM32Fxx
int sendData(flash_task_t task, const char *data, const int count);

//Wait for response from STM32Fxx
int waitForSerialData(int dataCount, int timeout);

//Send the STM32Fxx the memory address, to be written
int loadAddress(flash_task_t task, const char adrMS, const char adrMI, const char adrLI, const char adrLS);

//UART write the flash memory address of the STM32Fxx with blocks of data 
esp_err_t flashBlock(const char *address, const char *data);

//UART read the flash memory address of the STM32Fxx and verify with the given block of data 
esp_err_t readBlock(const char *address, const char *data);

#endif
