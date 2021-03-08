#include "freertos/FreeRTOS.h"
#include "driver/uart.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "nvs_flash.h"

#include "protocol_examples_common.h"

#include "stm_flash.h"
#include "stm_flash_http.h"

static const char *TAG = "MAIN";

#define TXD_PIN (GPIO_NUM_23)	// TX pin to STM32
#define RXD_PIN (GPIO_NUM_5)	// RX pin from STM32

#define RESET_PIN (GPIO_NUM_19)
#define BOOT0_PIN (GPIO_NUM_4) 

#define FIRMWARE_URL "https://github.com/mjuhanne/fyrtur-motor-board/raw/main/bin/fyrtur-0.80.bin"

// Instruct the flasher to fetch the MD5 checksum from this file via HTTP
//#define FIRMWARE_MD5 "https://github.com/mjuhanne/fyrtur-motor-board/raw/main/bin/fyrtur-0.80.bin.md5"

// Instruct the flasher to fetch the MD5 checksum from file via HTTP. The downloaded file will be {given-firmware-url}.md5 
// In this case a file 'https://github.com/mjuhanne/fyrtur-motor-board/raw/main/bin/fyrtur-0.80.bin.md5' would be downloaded
#define FIRMWARE_MD5 "FETCH"

// Alternatively the MD5 checksum can be given here directly
//#define FIRMWARE_MD5 "26cf6a94fcd44c5b44e2e590ad29c308"

// If the flash file is small then it can be downloaded to memory instead of writing it first to SPIFFS 
#define WRITE_TO_FILE 0

// Flash must be erased before writing to it. However in certain cases we want to retain some information 
// (for example configuration bits in the end of the flash memory) so we don't want to do mass erase, so we 
// erase only those needed pages (calculated from the size of the flash file)
#define MASS_ERASE 0

// The flash page size must be set if mass erase of the flash memory is not done before flashing. 
// For STM32F030x6 one page is 1024 bytes. For products having larger flash sizes (>128 kb ?) the page is 2048 bytes
#define STM32_FLASH_PAGE_SIZE 1024

// After reset the STM32 needs some time before bootloader can be accessed. 
// The bootloader delay depends on processor, HSE/LSE installation etc. It can be as long as 2560 milliseconds 
// on STM32L476RG without LSE! Please see Application note AN2606 for more details.
#define STM32F030_BOOTLOADER_DELAY 500

// This is the memory address (incremented with 4) from which bootloader 'Go' command will load the jump address.
// The jump address refers usually to the ResetHandler (pointer is stored at 0x08000004)
// This can be omitted if RESET pin can be used
#define STM32F030_MEMORY_ADDR 0x08000000

// Used for testing the connectivity and download functionality. 
// Will not erase or write flash memory but only will download the firmware file and compare it to the flash content
#define VERIFY_ONLY 1


void Flash_Log(const char* format, ...)
{
    va_list va;
    va_start(va, format);
    vprintf(format, va);
    va_end(va);
    printf("\r\n");
}


void stm_flash_callback( flash_task_t task, flash_stage_t stage, int block, int max_blocks, const char * msg ) {
    switch (stage) {
        case FLASH_STAGE_START: {
            Flash_Log("Starting Flash %s task", flash_task2txt(task));
        } break;

        case FLASH_STAGE_CONTINUE: {
            if (task == FLASH_DOWNLOAD) {
                Flash_Log("Downloading file: %d/%d", block, max_blocks);
            } else if (task == FLASH_DOWNLOAD_MD5) {
                Flash_Log("Downloading MD5 file: %d/%d", block, max_blocks);
            } else if (task == FLASH_ERASE) {
                Flash_Log("Erasing page %d/%d", block, max_blocks);
            } else if (task == FLASH_WRITE) {
                Flash_Log("Writing block %d/%d", block, max_blocks);
            } else if (task == FLASH_READ) {
                Flash_Log("Verifying block %d/%d", block, max_blocks);
            };
        } break;

        case FLASH_STAGE_COMPLETE: {
            Flash_Log("Flash %s task complete!", flash_task2txt(task));
        } break;

        //case FLASH_STAGE_LOG_DEBUG: 
        case FLASH_STAGE_LOG_ERROR: 
            // fall-through
        case FLASH_STAGE_LOG_INFO: {
            if (msg) {
                Flash_Log("Flash %s task: %s", flash_task2txt(task), msg);
            }
        } break;

        case FLASH_STAGE_ERROR: {
            if (msg) {
                Flash_Log("Flash %s task error! %s", flash_task2txt(task), msg);
            } else {
                Flash_Log("Flash %s task undefined error!", flash_task2txt(task));
            }
        } break;

        default: 
            break;
    }
}


void initTask(void)
{
    initFlashUART(TXD_PIN, RXD_PIN, 115200);
    initSTM32_GPIO(RESET_PIN, BOOT0_PIN);
    initSPIFFS();
}


void software_jump_to_stm32_bootloader() {
	// reconfigure UART with settings that are used to communicate with the existing firmware (2400bps, 8 bits, parity disable, 1 stop bit)
    uart_config_t uart_config = {
        .baud_rate = 2400,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_1, &uart_config);
    uart_flush(UART_NUM_1);

	// Send command to STM32 via UART to jump to bootloader. Naturally the existing firmware has to recognize this
	// command and act accordingly, otherwise this will fail and bootloader won't be entered
    //
    // This is just an example command used in Fyrtur motor board firmware (https://github.com/mjuhanne/fyrtur-motor-board)
	static char cmd [] = { 0x00, 0xff, 0x9a, 0xff, 0x00, 0xff};
	uart_write_bytes(UART_NUM_1, cmd, 6);

    // there might be response but we ignore it. 
    vTaskDelay( 1000 / portTICK_PERIOD_MS );

	configFlashUart(115200); // switch back to the UART settings that bootloader understands (115200 bps, 8 bits, even parity, 1 stop bit)
    uart_flush(UART_NUM_1);
}


int do_flash(bootloader_method_t method, const char *url, const char * digest, uint8_t write_to_file, uint8_t mass_erase) {

    if (method == STM_BOOTLOADER_METHOD_SOFTWARE) {
        software_jump_to_stm32_bootloader();
    }

    if (!enterBootLoader(method, STM32F030_BOOTLOADER_DELAY)) {
        ESP_LOGW(TAG, "Bootloader didn't respond to SYNC.. Try if it's already SYNCed");
        if (!cmdId()) {
            ESP_LOGW(TAG, "Bootloader still didn't respond. Exiting..");
            return 0;
        }
    }

    // check the STM32 ID and command set
    if (!queryBootLoader()) {
    	return 0;
    }

    ESP_LOGI(TAG,"OK! 5 second delay before flashing..");
    vTaskDelay( 5000 / portTICK_PERIOD_MS );

    flashSTM_from_URL(url, digest, write_to_file, mass_erase, VERIFY_ONLY);

    if (method == STM_BOOTLOADER_METHOD_SOFTWARE) {
        // Command the bootloader to jump to new firmware code (without having to use hardware reset)
        endConn(STM32F030_MEMORY_ADDR);        
    } else {
        // Do a hardware reset
        endConn(0);
    }
    return 1;
}


void app_main(void)
{
    // Initialize Wifi and connect to WiFi AP (configure it with 'idf.py menuconfig' -> 'Example Connection Configuration')
    ESP_ERROR_CHECK(nvs_flash_init());
    esp_netif_init();
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(example_connect());

    initTask();

    setFlashPageSize(STM32_FLASH_PAGE_SIZE);
    setFlashCallback(&stm_flash_callback);

    ESP_LOGI(TAG, "Starting flashing procedure with software jump to bootloader ...");
    int ret = do_flash(STM_BOOTLOADER_METHOD_SOFTWARE, FIRMWARE_URL, FIRMWARE_MD5, WRITE_TO_FILE, MASS_ERASE);

    if (ret != 1) {
    	ESP_LOGW(TAG,"Software method failed..");
	    ESP_LOGI(TAG, "Starting flashing procedure with hardware method (reset and jump to bootloader using RESET and BOOT0 pins...");
		do_flash(STM_BOOTLOADER_METHOD_HARDWARE, FIRMWARE_URL, FIRMWARE_MD5, WRITE_TO_FILE, MASS_ERASE);
    }

}

