#include "stm_flash.h"
#include "stm_pro_mode.h"

#include <sys/param.h>
#include "esp_err.h"
#include "esp_vfs.h"
#include "esp_spiffs.h"

static const char *TAG_STM_FLASH = "stm_flash";
static uint32_t page_size;

void setFlashPageSize(uint32_t _page_size) {
    flashLogInfo(FLASH_CONFIG, "Setting STM32 Flash page size to %d bytes", _page_size);
    page_size = _page_size;
}

int initSPIFFS(void)
{
    flashLogInfo(FLASH_CONFIG, "Initializing SPIFFS");

    esp_vfs_spiffs_conf_t conf =
        {
            .base_path = "/spiffs",
            .partition_label = NULL,
            .max_files = 5,
            .format_if_mount_failed = true};

    esp_err_t ret = esp_vfs_spiffs_register(&conf);

    if (ret != ESP_OK)
    {
        if (ret == ESP_FAIL)
        {
            flashLogError(FLASH_CONFIG, "Failed to mount or format filesystem");
        }
        else if (ret == ESP_ERR_NOT_FOUND)
        {
            flashLogError(FLASH_CONFIG, "Failed to find SPIFFS partition");
        }
        else
        {
            flashLogError(FLASH_CONFIG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        return 0;
    }

/*  // Formatting SPIFFS - Use only for debugging
    if (esp_spiffs_format(NULL) != ESP_OK)
    {
        flashLogError(FLASH_CONFIG, "Failed to format SPIFFS");
        return 0;
    }
*/

    size_t total, used;
    if (esp_spiffs_info(NULL, &total, &used) == ESP_OK)
    {
        flashLogInfo(FLASH_CONFIG, "Partition size: total: %d, used: %d", total, used);
    }
    return 1;
}


esp_err_t writeTask(FILE *flash_file, uint32_t size)
{
    int blocks_count = size / 256;
    if (size % 256 != 0) blocks_count++;

    flashNotification(FLASH_WRITE, FLASH_STAGE_START, 0, blocks_count, "Write Task");

    char loadAddress[4] = {0x08, 0x00, 0x00, 0x00};
    char block[256] = {0};
    int curr_block = 0, bytes_read = 0;

    fseek(flash_file, 0, SEEK_SET);

    while ((bytes_read = fread(block, 1, 256, flash_file)) > 0)
    {
        flashNotification(FLASH_WRITE, FLASH_STAGE_CONTINUE, curr_block+1, blocks_count, NULL);
        // ESP_LOG_BUFFER_HEXDUMP("Block:  ", block, sizeof(block), ESP_LOG_DEBUG);

        esp_err_t ret = flashBlock(loadAddress, block);
        if (ret == ESP_FAIL)
        {
            flashNotification(FLASH_WRITE, FLASH_STAGE_ERROR, curr_block+1, blocks_count, "Writing block failed!");
            return ESP_FAIL;
        }

        curr_block++;

        incrementLoadAddress(loadAddress);

        memset(block, 0xff, 256);
    }

    flashNotification(FLASH_WRITE, FLASH_STAGE_COMPLETE, blocks_count, blocks_count, "Write Task Complete!");
    return ESP_OK;
}


esp_err_t writeTask_from_mem(char * buffer, uint32_t size)
{
    int blocks_count = size / 256;
    if (size % 256 != 0) blocks_count++;

    flashNotification(FLASH_WRITE, FLASH_STAGE_START, 0, blocks_count, "Write Task");

    char loadAddress[4] = {0x08, 0x00, 0x00, 0x00};
    int curr_block = 0, bytes_read = 0, pos = 0;

    while (((bytes_read = MIN(size-pos, 256)) > 0)) {

        flashNotification(FLASH_WRITE, FLASH_STAGE_CONTINUE, curr_block+1, blocks_count, NULL);

        esp_err_t ret = flashBlock(loadAddress, &buffer[pos]);
        if (ret == ESP_FAIL)
        {
            flashNotification(FLASH_WRITE, FLASH_STAGE_ERROR, curr_block+1, blocks_count, "Writing block failed!");
            return ESP_FAIL;
        }
        pos += bytes_read;

        curr_block++;
        incrementLoadAddress(loadAddress);
    }

    flashNotification(FLASH_WRITE, FLASH_STAGE_COMPLETE, blocks_count, blocks_count, "Write Task Complete!");
    return ESP_OK;
}


esp_err_t readTask(FILE *flash_file, uint32_t size)
{
    char readAddress[4] = {0x08, 0x00, 0x00, 0x00};
    int blocks_count = size / 256;
    if (size % 256 != 0) blocks_count++;

    flashNotification(FLASH_READ, FLASH_STAGE_START, 0, blocks_count, "Read & Verification Task");

    char block[256] = {0};
    char block2[256] = {0};
    int curr_block = 0, bytes_read = 0;

    fseek(flash_file, 0, SEEK_SET);

    while ((bytes_read = fread(block, 1, 256, flash_file)) > 0)
    {
        curr_block++;
        flashNotification(FLASH_READ, FLASH_STAGE_CONTINUE, curr_block, blocks_count, NULL);
        // ESP_LOG_BUFFER_HEXDUMP("Block:  ", block, sizeof(block), ESP_LOG_DEBUG);

        esp_err_t ret = readBlock(readAddress, block2);
        if (ret == ESP_FAIL)
        {
            flashNotification(FLASH_READ, FLASH_STAGE_ERROR, curr_block+1, blocks_count, "Reading block failed!");
            return ESP_FAIL;
        }

        if (memcmp(block, block2, bytes_read) != 0) {
            flashNotification(FLASH_READ, FLASH_STAGE_ERROR, curr_block+1, blocks_count, "Verifying block failed!");
            ESP_LOG_BUFFER_HEXDUMP("Orig Block:  ", block, bytes_read, ESP_LOG_ERROR);
            ESP_LOG_BUFFER_HEXDUMP("Read Block:  ", block2, bytes_read, ESP_LOG_ERROR);
            return ESP_FAIL;            
        }

        incrementLoadAddress(readAddress);

        memset(block, 0xff, 256);
    }

    flashNotification(FLASH_READ, FLASH_STAGE_COMPLETE, blocks_count, blocks_count, "Read & Verification Task Complete!");
    return ESP_OK;
}


esp_err_t readTask_from_mem(char * buffer, uint32_t size)
{
    int blocks_count = size / 256;
    if (size % 256 != 0) blocks_count++;

    flashNotification(FLASH_READ, FLASH_STAGE_START, 0, blocks_count, "Read & Verification Task");
    char readAddress[4] = {0x08, 0x00, 0x00, 0x00};

    char block[256] = {0};
    int curr_block = 0, bytes_read = 0, pos = 0;

    while (((bytes_read = MIN(size-pos, 256)) > 0)) {
        flashNotification(FLASH_READ, FLASH_STAGE_CONTINUE, curr_block+1, blocks_count, NULL);
        // ESP_LOG_BUFFER_HEXDUMP("Block:  ", block, sizeof(block), ESP_LOG_DEBUG);

        esp_err_t ret = readBlock(readAddress, block);
        if (ret == ESP_FAIL)
        {
            flashNotification(FLASH_READ, FLASH_STAGE_ERROR, curr_block+1, blocks_count, "Reading block failed!");
            return ESP_FAIL;
        }
        if (memcmp(block, &buffer[pos], bytes_read ) != 0) {
            flashNotification(FLASH_READ, FLASH_STAGE_ERROR, curr_block+1, blocks_count, "Verifying block failed!");
            ESP_LOG_BUFFER_HEXDUMP("Orig Block:  ", &buffer[pos], bytes_read, ESP_LOG_ERROR);
            ESP_LOG_BUFFER_HEXDUMP("Read Block:  ", block, bytes_read, ESP_LOG_ERROR);
            return ESP_FAIL;
        }
        pos += bytes_read;
        curr_block++;

        incrementLoadAddress(readAddress);

        memset(block, 0xff, 256);
    }

    flashNotification(FLASH_READ, FLASH_STAGE_COMPLETE, blocks_count, blocks_count, "Read & Verification Task Complete!");
    return ESP_OK;
}


esp_err_t eraseTask(uint32_t size)
{
    esp_err_t ret;
    int pages;
 
    if (size == 0) {
        flashNotification(FLASH_ERASE, FLASH_STAGE_START, 0, 1, "Mass Erase Task");
        pages = 1;
        ret = cmdMassErasePages();

    } else {

        if (page_size == 0) {
            flashNotification(FLASH_ERASE, FLASH_STAGE_ERROR, 0, 0, "Flash page size not set!");
            return 0;
        }
        flashNotification(FLASH_ERASE, FLASH_STAGE_START, 0, 1, "Erase Task");
        // calculate the needed number of flash pages to erase
        pages = size / page_size;
        if (size > page_size * pages) {
            // one more partly filled page needed
            pages++;
        }
        for (int i=0;i<pages;i++) {
            flashNotification(FLASH_ERASE, FLASH_STAGE_CONTINUE, i+1, pages, "Erase Page");
            if (!cmdErasePage(i, 1)) {
                flashNotification(FLASH_ERASE, FLASH_STAGE_ERROR, i+1, pages, "Erase Page failed!");
                return ESP_FAIL;
            }
        }
        ret = ESP_OK;
    }
    flashNotification(FLASH_ERASE, FLASH_STAGE_COMPLETE, pages, pages, "Erase Task Complete!");
    return ret;
}


esp_err_t flashSTM(const char *file_name, uint8_t mass_erase, uint8_t verify_only)
{
    esp_err_t err = ESP_FAIL;

    char file_path[FILE_PATH_MAX];
    sprintf(file_path, "%s%s", BASE_PATH, file_name);
    ESP_LOGI(TAG_STM_FLASH, "File name: %s", file_path);

    FILE *flash_file = fopen(file_path, "rb");
    if (flash_file != NULL)
    {

        fseek(flash_file, 0, SEEK_END); // seek to end of file
        uint32_t file_size = ftell(flash_file); // get file size
        fseek(flash_file, 0, SEEK_SET); // rewind

        // This while loop executes only once and breaks if any of the functions do not return ESP_OK
        do
        {        
            if (!verify_only) {
                ESP_LOGI(TAG_STM_FLASH, "Erasing STM32 Memory");
                IS_ESP_OK(eraseTask( mass_erase ? 0 : file_size));                

                ESP_LOGI(TAG_STM_FLASH, "Writing STM32 Memory");
                IS_ESP_OK(writeTask(flash_file, file_size));
            }

            ESP_LOGI(TAG_STM_FLASH, "Reading STM32 Memory");
            IS_ESP_OK(readTask(flash_file, file_size));

            err = ESP_OK;
        } while (0);

        if (err == ESP_OK) 
            ESP_LOGI(TAG_STM_FLASH, "STM32 Flashed Successfully!!!");
    }

    ESP_LOGI(TAG_STM_FLASH, "Closing file");
    fclose(flash_file);

    return err;
}


esp_err_t flashSTM_from_mem(char * buffer, uint32_t size, uint8_t mass_erase, uint8_t verify_only)
{
    esp_err_t err = ESP_FAIL;

    // This while loop executes only once and breaks if any of the functions do not return ESP_OK
    do
    {
        if (!verify_only) {
            ESP_LOGI(TAG_STM_FLASH, "Erasing STM32 Memory");
            IS_ESP_OK(eraseTask( mass_erase ? 0 : size));

            ESP_LOGI(TAG_STM_FLASH, "Writing STM32 Memory");
            IS_ESP_OK(writeTask_from_mem(buffer, size));
        }

        ESP_LOGI(TAG_STM_FLASH, "Reading STM32 Memory");
        IS_ESP_OK(readTask_from_mem(buffer, size));

        err = ESP_OK;
    } while (0);

    if (err == ESP_OK) 
        ESP_LOGI(TAG_STM_FLASH, "STM32 Flashed Successfully!!!");

    return err;
}
