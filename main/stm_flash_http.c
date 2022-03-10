#include "stm_flash_http.h"
#include "esp_http_client.h"
#include "esp_tls.h"
#include "esp_rom_md5.h"

#define TAG "stm_flash_http"
#define BASE_PATH "/spiffs/"

struct MD5Context myContext;
static unsigned char digest[16];
static unsigned char file_digest[16];
static FILE * fd;

static char *output_buffer;  // Buffer to store response of http request from event handler
static int output_len;       // Stores number of bytes read
static int output_buffer_size; // File length

static char *flash_buffer;  // Buffer to store downloaded flash file in case it isn't written to file
static int flash_buffer_len;       // Stores number of bytes read

flash_task_t current_task;

esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
    switch(evt->event_id) {
        case HTTP_EVENT_ERROR:
            ESP_LOGE(TAG, "HTTP_EVENT_ERROR");
            break;
        case HTTP_EVENT_ON_CONNECTED:
            ESP_LOGI(TAG, "HTTP_EVENT_ON_CONNECTED");

			memset(&myContext,0x00,sizeof(myContext));
			esp_rom_md5_init(&myContext);
            if (output_buffer) {
                free(output_buffer);
                output_buffer = NULL;
            }
            output_len = 0;

            break;
        case HTTP_EVENT_HEADER_SENT:
            ESP_LOGI(TAG, "HTTP_EVENT_HEADER_SENT");
            break;
        case HTTP_EVENT_ON_HEADER:
            //ESP_LOGI(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
            flashLogDebug(current_task, "HTTP header: key=%s, value=%s", evt->header_key, evt->header_value);
            break;
        case HTTP_EVENT_ON_DATA:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
            // Check if the data is actually the downloaded file (it could be HTTP 302 Redirect as well, so we don't want to save that)
            if (esp_http_client_get_status_code(evt->client) == 200) {            
                if (!esp_http_client_is_chunked_response(evt->client)) {
                    output_buffer_size = esp_http_client_get_content_length(evt->client);
                    if (evt->user_data) {
                    	// write to file
                    	fwrite(evt->data, 1, evt->data_len, fd);
                    } else {
                        if (output_buffer == NULL) {
                            output_buffer = (char *) malloc(output_buffer_size);
                            output_len = 0;
                            if (output_buffer == NULL) {
                                ESP_LOGE(TAG, "Failed to allocate memory for output buffer");
                                return ESP_FAIL;
                            }
                        }
                        memcpy(output_buffer + output_len, evt->data, evt->data_len);
                    }
    				output_len += evt->data_len;
                    flashNotification(current_task, FLASH_STAGE_CONTINUE, output_len, output_buffer_size, "Chunk received");
                    ESP_LOGD(TAG,"Chunk received. Total len %d", output_len);
    				esp_rom_md5_update(&myContext, (unsigned char*)evt->data, evt->data_len);
                }
            }

            break;
        case HTTP_EVENT_ON_FINISH:
            ESP_LOGI(TAG, "HTTP_EVENT_ON_FINISH");
            if (esp_http_client_get_status_code(evt->client) == 200) {            
                ESP_LOGI(TAG,"Total data len %d", output_len);
                esp_rom_md5_final(digest, &myContext);
            }
            break;
        case HTTP_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "HTTP_EVENT_DISCONNECTED");
            int mbedtls_err = 0;
            esp_err_t err = esp_tls_get_and_clear_last_error(evt->data, &mbedtls_err, NULL);
            if (err != 0) {
                if (output_buffer != NULL) {
                    free(output_buffer);
                    output_buffer = NULL;
                }
                output_len = 0;
                ESP_LOGI(TAG, "Last esp error code: 0x%x", err);
                ESP_LOGI(TAG, "Last mbedtls failure: 0x%x", mbedtls_err);
            }
            break;
    }
    return ESP_OK;
}


esp_err_t download_file(const char * url, uint8_t write_to_file, int * http_status_code ) {
    esp_err_t ret = ESP_OK;

    ESP_LOGI(TAG, "Downloading file : %s...", url);


    esp_http_client_config_t config = {
        .url = url,
        .event_handler = _http_event_handler,
    };
    if (write_to_file) {
        config.user_data = (void*)1;
    } else {        
        config.user_data = (void*)0;
    }

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client != NULL) {

        ret = esp_http_client_perform(client);

        *http_status_code = 404;

        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "HTTP chunk encoding Status = %d, content_length = %d",
                    esp_http_client_get_status_code(client),
                    esp_http_client_get_content_length(client));
            *http_status_code = esp_http_client_get_status_code(client);
        } else {
            ESP_LOGE(TAG, "Error perform http request %s", esp_err_to_name(ret));
        }
        esp_http_client_cleanup(client);
    } else {
        ret = ESP_FAIL;
    }
    return ret;
}


esp_err_t flashSTM_from_URL(const char *url, const char * _digest, uint8_t write_to_file, uint8_t mass_erase, uint8_t verify_only) {
    char filepath[FILE_PATH_MAX];
    int http_status_code;
    esp_err_t ret;

    struct stat file_stat;
    const char * filename;
    uint8_t valid_checksum = 0;

    ESP_LOGI(TAG,"URL: %s", url);

    flash_buffer = NULL;

    size_t pathlen = strlen(url);
    if (pathlen == 0)
    	return ESP_FAIL;
    if (url[pathlen-1] == '/')
    	return ESP_FAIL;

    // extract file name by finding last /
    int i = pathlen - 1;
    while ( (url[i] != '/') && (i>0)) {
    	i--;
    }
    if (url[i] == '/') {
    	i++;
    }

    filename = &url[i];

    if (write_to_file) {

        strncpy(filepath,BASE_PATH, FILE_PATH_MAX-1);
        strncat(filepath,filename, FILE_PATH_MAX-strlen(filepath)-1);

        ESP_LOGI(TAG,"File path: %s", filepath);

        if (stat(filepath, &file_stat) == 0)
        {
            ESP_LOGE(TAG, "File already exists : %s ", filepath);
            ESP_LOGI(TAG, "Deleting file : %s", filepath);
            /* Delete file */
            unlink(filepath);
        }

        fd = fopen(filepath, "w");
        if (!fd)
        {
            ESP_LOGE(TAG, "Failed to create file : %s", filepath);
            return ESP_FAIL;
        }
    }


    flashNotification(FLASH_DOWNLOAD, FLASH_STAGE_START, -1, -1, "Starting flash file download");
    current_task = FLASH_DOWNLOAD;
    ret = download_file(url, write_to_file, &http_status_code);

    if (write_to_file) {
        fclose(fd);
    } else {
        flash_buffer = output_buffer;
        flash_buffer_len = output_len;
        output_buffer = NULL;
    }

    if ( (ret == ESP_OK) && (http_status_code == 200)) {

        flashNotification(FLASH_DOWNLOAD, FLASH_STAGE_COMPLETE, -1, -1, "Flash file downloaded!");

        memcpy(file_digest, digest, 16);
        ESP_LOGI(TAG, "File MD5:");
        ESP_LOG_BUFFER_HEXDUMP(TAG, file_digest, 16, ESP_LOG_INFO);

    	if (_digest == NULL) {
    		// do not enforce checksum
            flashLogInfo(FLASH_DOWNLOAD_MD5, "Skipping MD5 calculation");
    		valid_checksum = 1;
    	} else {
            flashLogInfo(FLASH_DOWNLOAD_MD5, "MD5 parameter: '%s'", _digest);
    		if ( (strcmp(_digest,"FETCH") == 0) || (strncmp(_digest,"http", 4) == 0) ) {

                if (strcmp(_digest,"FETCH") == 0) {
                    // fetch [URL].md5 file
                    strncpy(filepath, url, FILE_PATH_MAX-1);
                    strncat(filepath, ".md5", FILE_PATH_MAX-strlen(filepath)-1);
                } else {
                    // _digest is the MD5 file url 
                    strncpy(filepath, _digest, FILE_PATH_MAX-1);
                }

                output_buffer = NULL;
                output_len = 0;

                flashNotification(FLASH_DOWNLOAD_MD5, FLASH_STAGE_START, -1, -1, "Starting MD5 file download");
                current_task = FLASH_DOWNLOAD_MD5;

                ret = download_file( filepath, 0, &http_status_code);

                if ( (ret == ESP_OK) && (http_status_code == 200) ) {

                    flashNotification(FLASH_DOWNLOAD, FLASH_STAGE_COMPLETE, -1, -1, "MD5 file downloaded!");

					if (output_len < 2*16) {
                        flashNotification(FLASH_DOWNLOAD, FLASH_STAGE_ERROR, -1, -1, "Valid CRC not found!");
					    ret = ESP_ERR_NOT_FOUND;
					} else {                                
    					char value[3];
    					for (int i=0;i<16;i++) {
    						value[0] = output_buffer[i*2 + 0];
    						value[1] = output_buffer[i*2 + 1];
    						value[2] = 0;
    						digest[i] = strtol(value, NULL, 16);
    					}
                    }
			    } else {
                    flashNotification(FLASH_DOWNLOAD, FLASH_STAGE_ERROR, -1, -1, "Error downloading MD5 file!");
				    ret = ESP_ERR_NOT_FOUND;			    	
			    }


    		} else {

                // validate with given MD5

    			if (strlen(_digest) < 2*16) {
                    flashNotification(FLASH_DOWNLOAD, FLASH_STAGE_ERROR, -1, -1, "Valid CRC not given!");
    			    ret = ESP_ERR_NOT_FOUND;
    			} else {                    
        			char value[3];
        			for (int i=0;i<16;i++) {
        				value[0] = _digest[i*2 + 0];
        				value[1] = _digest[i*2 + 1];
        				value[2] = 0;
        				digest[i] = strtol(value, NULL, 16);
        			}
                }
    		}

            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "Input MD5:");
                ESP_LOG_BUFFER_HEXDUMP(TAG, digest, 16, ESP_LOG_INFO);

                valid_checksum = 1;
                for (int i=0;i<16;i++) {
                    if (file_digest[i] != digest[i])
                        valid_checksum = 0;
                }                    
            }

    	}
    } else {
        flashNotification(FLASH_DOWNLOAD, FLASH_STAGE_ERROR, -1, -1, "Error fetching file!");
        ret = ESP_ERR_NOT_FOUND;        
    }

    if (ret == ESP_OK) {

    	if (!valid_checksum) {
            flashNotification(FLASH_DOWNLOAD, FLASH_STAGE_ERROR, -1, -1, "MD5 checksum failed!");
    		ret = ESP_ERR_INVALID_CRC;
    	} else {

            if (write_to_file) {
                ret = flashSTM(filename, mass_erase, verify_only);
            } else {
                ret = flashSTM_from_mem(flash_buffer, flash_buffer_len, mass_erase, verify_only);
            }
            if (ret != ESP_OK) {
                ESP_LOGE(TAG,"Target flashing failed");
            }
        }
    }

    if (output_buffer != NULL) {
        free(output_buffer);
        output_buffer = NULL;
    }
    if (flash_buffer != NULL) {
        free(flash_buffer);
        flash_buffer = NULL;
    }


    return ret;
}
