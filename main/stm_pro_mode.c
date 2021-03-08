#include "stm_pro_mode.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#define STM_BOOTLOADER_SYNC             0x7F

#define STM_BOOTLOADER_CMD_GET          0x00
#define STM_BOOTLOADER_CMD_GET_VERSION  0x01
#define STM_BOOTLOADER_CMD_GET_ID       0x02
#define STM_BOOTLOADER_CMD_READ_MEM     0x11
#define STM_BOOTLOADER_CMD_GO           0x21
#define STM_BOOTLOADER_CMD_WRITE_MEM    0x31
#define STM_BOOTLOADER_CMD_ERASE        0x43
#define STM_BOOTLOADER_CMD_EXT_ERASE    0x44
#define STM_BOOTLOADER_CMD_WRITE_PROT   0x63
#define STM_BOOTLOADER_CMD_WRITE_UNPROT 0x73
#define STM_BOOTLOADER_CMD_READOUT_PROT 0x82
#define STM_BOOTLOADER_CMD_READOUT_UNPROT 0x92

#define STM_BOOTLOADER_ERASE_TYPE_UNDEFINED 0x00
#define STM_BOOTLOADER_ERASE_TYPE_ERASE     0x01
#define STM_BOOTLOADER_ERASE_TYPE_EXT_ERASE 0x02

static const char *TAG_STM_PRO = "stm_pro_mode";
static const char * task2txt[] = { "Config", "Download", "Download MD5", "Sync", "Erase", "Write", "Read"};

static uint8_t reset_pin = 255;
static uint8_t boot0_pin = 255;
static uint8_t erase_type;

static void (*flash_callback)() = NULL;

void flashNotification( flash_task_t task, flash_stage_t stage, int block, int max_blocks, const char * msg ) {
    if (flash_callback) {
        (*flash_callback)(task, stage, block, max_blocks, msg);
    }
}

void flashLogInfo( flash_task_t task, const char* format, ... ) {
    char buf[64];
    if (flash_callback) {
        va_list va;
        va_start(va, format);
        vsnprintf(buf, 64, format, va);
        va_end(va);
        (*flash_callback)(task, FLASH_STAGE_LOG_INFO, -1, -1, buf);
    }
}

void flashLogError( flash_task_t task, const char* format, ... ) {
    char buf[64];
    if (flash_callback) {
        va_list va;
        va_start(va, format);
        vsnprintf(buf, 64, format, va);
        va_end(va);
        (*flash_callback)(task, FLASH_STAGE_LOG_ERROR, -1, -1, buf);
    }
}

void flashLogDebug( flash_task_t task, const char* format, ... ) {
    char buf[64];
    if (flash_callback) {
        va_list va;
        va_start(va, format);
        vsnprintf(buf, 64, format, va);
        va_end(va);
        (*flash_callback)(task, FLASH_STAGE_LOG_DEBUG, -1, -1, buf);
    }
}

void setFlashCallback( FLASH_CALLBACK_TYPE ) {
    flash_callback = func_ptr;
}

const char * flash_task2txt( flash_task_t task ) {
    if (task < sizeof(task2txt))
        return task2txt[task];
    return "Undefined";
}

int configFlashUart(uint32_t baud_rate) {
    const uart_config_t uart_config = {
        .baud_rate = baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_EVEN,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
    return (uart_param_config(UART_CONTROLLER, &uart_config) == ESP_OK);
}

int initFlashUART( uint8_t txd_pin, uint8_t rxd_pin, uint32_t baud_rate )
{
    if (uart_driver_install(UART_CONTROLLER, UART_BUF_SIZE * 2, 0, 0, NULL, 0) != ESP_OK) {
        flashLogError(FLASH_CONFIG,"UART driver install error!" );
        return 0;        
    }
    if (!configFlashUart(baud_rate)) {
        flashLogError(FLASH_CONFIG,"UART config error!" );
        return 0;
    }
    if (uart_set_pin(UART_CONTROLLER, txd_pin, rxd_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK) {
        flashLogError(FLASH_CONFIG,"UART set pin error!" );
        return 0;
    }
    uart_set_rts(UART_CONTROLLER, 1);

    flashLogInfo(FLASH_CONFIG,"Initialized Flash UART" );
    return 1;
}

int initSTM32_GPIO(uint8_t _reset_pin, uint8_t _boot0_pin)
{
    reset_pin = _reset_pin;
    boot0_pin = _boot0_pin;
    esp_err_t err = gpio_set_direction(reset_pin, GPIO_MODE_OUTPUT);
    if (err != ESP_OK) {
        flashLogError(FLASH_CONFIG,"Reset pin config error!" );
        return 0;
    }

    err = gpio_set_level(reset_pin, HIGH);
    if (err != ESP_OK) {
        flashLogError(FLASH_CONFIG,"Reset pin config error!" );
        return 0;
    }
    err = gpio_set_direction(boot0_pin, GPIO_MODE_OUTPUT);
    if (err != ESP_OK) {
        flashLogError(FLASH_CONFIG,"Boot0 pin config error!" );
        return 0;
    }
    err = gpio_set_level(boot0_pin, LOW);
    if (err != ESP_OK) {
        flashLogError(FLASH_CONFIG,"Boot0 pin config error!" );
        return 0;
    }

    flashLogInfo(FLASH_CONFIG,"GPIO Initialized" );
    ESP_LOGI(TAG_STM_PRO, "RESET pin %d, BOOT0 pin %d", reset_pin, boot0_pin);
    return 1;
}

int resetSTM(void)
{
    if (reset_pin == 255) {
        flashLogError(FLASH_SYNC,"Reset pin not defined!" );
        return 0;
    }

    flashLogInfo(FLASH_SYNC,"Starting HW RESET Procedure" );
 
    gpio_set_level(reset_pin, LOW);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    gpio_set_level(reset_pin, HIGH);
    vTaskDelay(500 / portTICK_PERIOD_MS);

    flashLogInfo(FLASH_SYNC,"Finished HW RESET Procedure" );
    return 1;
}

int enterBootLoader(bootloader_method_t method, int bootLoaderDelay)
{
    if (method == STM_BOOTLOADER_METHOD_HARDWARE) {
        if (boot0_pin != 255) {
            flashLogInfo(FLASH_SYNC,"Entering bootloader by setting BOOT0 pin" );

            // Enter boot loader via boot0 pin
            gpio_set_level(boot0_pin, HIGH);
            vTaskDelay(10 / portTICK_PERIOD_MS);

        } else {
            flashLogError(FLASH_SYNC,"BOOT0 pin not defined!" );
            return 0;
        }
    }
    // Even if software method is used, we will use RESET pins to enforce the reset if the pin number is defined.
    // This is because if reset pin is connected with pull-up, STM32 may not be able to do a software reset and will hang indefinetely!
    if (reset_pin != 255) {
        flashLogInfo(FLASH_SYNC,"RESET pin is defined so initiating HW reset just in case.." );
        resetSTM();
    }
    flashLogInfo(FLASH_SYNC,"Waiting for bootloader to be ready.." );
    vTaskDelay(bootLoaderDelay / portTICK_PERIOD_MS);        
    if (!cmdSync()) {
        flashLogError(FLASH_SYNC," Could not sync with bootloader!" );
        return 0;
    }
    flashLogInfo(FLASH_SYNC,"Bootloader ready" );
    return 1;
}

void endConn( uint32_t jump_addr )
{
    if (jump_addr == 0) {
        if (reset_pin != 255) {
            flashLogInfo(FLASH_SYNC,"Ending Connection with HW reset" );
            if (boot0_pin != 255) {
                gpio_set_level(boot0_pin, LOW);
                vTaskDelay(50 / portTICK_PERIOD_MS);
            }      
            resetSTM();
        } else {
            flashLogError(FLASH_SYNC,"RESET pin not defined!" );
        }
    } else {
        flashLogInfo(FLASH_SYNC,"Jumping to Reset Handler" );
        cmdGo(jump_addr);
    }
}

int queryBootLoader() {
    cmdGet();
    cmdVersion();
    cmdId();
    return 1;
}

int cmdSync(void)
{
    flashLogInfo(FLASH_SYNC, "Cmd: SYNC");
    char bytes[] = {STM_BOOTLOADER_SYNC};
    int resp = 1;
    return sendBytes(FLASH_SYNC, bytes, sizeof(bytes), resp);
}


int cmdGet(void)
{
    flashLogInfo(FLASH_SYNC, "Cmd: GET");

    char bytes[] = {STM_BOOTLOADER_CMD_GET, ~STM_BOOTLOADER_CMD_GET};
    int resp = 15;
    char recv_bytes[resp];
    int rx_bytes = sendReceiveBytes(FLASH_SYNC, bytes, sizeof(bytes), recv_bytes, resp);
    if (rx_bytes == resp) {
        for (int i=2;i<rx_bytes;i++) {
            switch (recv_bytes[i]) {
                case STM_BOOTLOADER_CMD_ERASE: {
                    flashLogInfo(FLASH_SYNC, "Erase CMD supported");
                    erase_type = STM_BOOTLOADER_ERASE_TYPE_ERASE;
                } 
                break;
                case STM_BOOTLOADER_CMD_EXT_ERASE: {
                    flashLogInfo(FLASH_SYNC, "Extended erase CMD supported");
                    erase_type = STM_BOOTLOADER_ERASE_TYPE_EXT_ERASE;
                }
                break;

                default: 
                break;
            }
        }

    } else {
        flashLogError(FLASH_SYNC, "Error executing CmdGet!");
        return 0;
    }
    return 1;
}

int cmdVersion(void)
{
    flashLogInfo(FLASH_SYNC, "Cmd: GETVERSION");

    char bytes[] = {STM_BOOTLOADER_CMD_GET_VERSION, ~STM_BOOTLOADER_CMD_GET_VERSION};
    int resp = 5;
    char recv_bytes[resp];
    int rx_bytes = sendReceiveBytes(FLASH_SYNC, bytes, sizeof(bytes), recv_bytes, resp);
    if (rx_bytes == resp) {
        flashLogInfo(FLASH_SYNC, "Bootloader version: %d.%d (0x%x)", recv_bytes[1] >> 4, recv_bytes[1] & 0xF, recv_bytes[1]);
        flashLogInfo(FLASH_SYNC, "Option byte #1: 0x%x", recv_bytes[2]);
        flashLogInfo(FLASH_SYNC, "Option byte #2: 0x%x", recv_bytes[3]);
        return 1;
    } 
    flashLogError(FLASH_SYNC, "Error executing CmdVersion!");
    return 0;        
}

int cmdId(void)
{
    flashLogInfo(FLASH_SYNC, "Cmd: GET_ID");

    char bytes[] = {STM_BOOTLOADER_CMD_GET_ID, ~STM_BOOTLOADER_CMD_GET_ID};
    int resp = 5;
    char recv_bytes[resp];
    int rx_bytes = sendReceiveBytes(FLASH_SYNC, bytes, sizeof(bytes), recv_bytes, resp);
    if (rx_bytes == resp) {
        if (recv_bytes[1] == 1) {
            flashLogInfo(FLASH_SYNC,  "Chip ID byte #1: 0x%02x%02x", recv_bytes[2], recv_bytes[3]);
        } else {
            flashLogError(FLASH_SYNC, "Unexpected byte length %d! Not STM32 ?!", recv_bytes[1]);
            return 0;
        }
        return 1;
    }
    flashLogError(FLASH_SYNC, "Error executing CmdGetId");
    return 0;        
}


int cmdMassErase(void)
{
    flashLogInfo(FLASH_ERASE, "Cmd: MASS ERASE");
    char bytes[] = {STM_BOOTLOADER_CMD_ERASE, ~STM_BOOTLOADER_CMD_ERASE};
    int resp = 1;
    int a = sendBytes(FLASH_ERASE, bytes, sizeof(bytes), resp);

    if (a == 1)
    {
        char params[] = {0xFF, 0x00};
        resp = 1;

        return sendBytes(FLASH_ERASE, params, sizeof(params), resp);
    }
    return 0;
}


int cmdExtMassErase(void)
{
    flashLogInfo(FLASH_ERASE, "Cmd: EXTENDED MASS ERASE");
    char bytes[] = {STM_BOOTLOADER_CMD_EXT_ERASE, ~STM_BOOTLOADER_CMD_EXT_ERASE};
    int resp = 1;
    int a = sendBytes(FLASH_ERASE, bytes, sizeof(bytes), resp);

    if (a == 1)
    {
        char params[] = {0xFF, 0xFF, 0x00};
        resp = 1;

        return sendBytes(FLASH_ERASE, params, sizeof(params), resp);
    }
    return 0;
}

int cmdMassErasePages(void) {
    if (erase_type == STM_BOOTLOADER_ERASE_TYPE_ERASE) {
        return cmdMassErase();
    } else if (erase_type == STM_BOOTLOADER_ERASE_TYPE_EXT_ERASE) {
        return cmdExtMassErase();
    }
    flashLogError(FLASH_ERASE,"Erase type not defined!");
    return 0;
}


int cmdErase(uint8_t startPage, uint8_t count)
{
    flashLogInfo(FLASH_ERASE, "Cmd: ERASE");
    char bytes[] = {0x43, 0xBC};
    int resp = 1;
    int a = sendBytes(FLASH_ERASE, bytes, sizeof(bytes), resp);

    if (a == 1)
    {
        char params[1 + count +1];  // Number of pages (N = 1 byte) + page list + checksum (1 byte) 
        params[0] = count - 1; // N+1 pages will be erased, so decrease by one
        uint8_t checksum = count;
        for (int i=0;i<count;i++) {
            params[1+i] = i;
            checksum ^= i;
        }
        params[count+1] = checksum;

        resp = 1;
        return sendBytes(FLASH_ERASE, params, sizeof(params), resp);
    }
    return 0;
}


int cmdExtErase(uint16_t startPage, uint16_t count)
{
    flashLogInfo(FLASH_ERASE, "Cmd: EXT ERASE");
    char bytes[] = {STM_BOOTLOADER_CMD_EXT_ERASE, ~STM_BOOTLOADER_CMD_EXT_ERASE};
    int resp = 1;
    int a = sendBytes(FLASH_ERASE, bytes, sizeof(bytes), resp);

    if (a == 1)
    {
        char params[2 + count*2 + 1];   // 2 byte length (N), 2 bytes for each page address and 1 checksum byte
        uint16_t checksum = 0;
        int idx = 2;

        params[0] = (count-1) >> 8;  // N+1 pages will be erased, so decrease by one
        checksum ^= params[0];
        params[1] = (count-1) & 0xFF;
        checksum ^= params[1];
        for (int i = startPage; i < startPage+count; i++) {
            params[idx + 0] = i >> 8;
            params[idx + 1] = i & 0xFF;
            checksum ^= params[idx + 0];
            checksum ^= params[idx + 1];
            idx += 2;
        }
        params[idx] = checksum;

        resp = 1;

        return sendBytes(FLASH_ERASE, params, sizeof(params), resp);
    }
    return 0;
}


int cmdErasePage( uint16_t startPage, uint16_t count )
{
    flashLogInfo(FLASH_ERASE, "Cmd: ERASE PAGES %d-%d (count %d)", startPage, startPage+count-1, count);
    if (erase_type == STM_BOOTLOADER_ERASE_TYPE_ERASE) {
        if (startPage + count <= 256) {
            return cmdErase(startPage, count);        
        } else {
            flashLogError(FLASH_ERASE, "Pages out of bounds!");
            return 0;
        }
    } else if (erase_type == STM_BOOTLOADER_ERASE_TYPE_EXT_ERASE) {
        return cmdExtErase(startPage, count);
    }
    flashLogError(FLASH_ERASE, "Erase type not defined!");
    return 0;
}


int cmdGo(uint32_t addr) {
    flashLogInfo(FLASH_SYNC, "CmdGo: JUMP TO ADDRESS 0x%x", addr);
    char bytes[] = {STM_BOOTLOADER_CMD_GO, ~STM_BOOTLOADER_CMD_GO};
    int resp = 1;
    int a = sendBytes(FLASH_SYNC, bytes, sizeof(bytes), resp);

    if (a == 1)
    {
        char params[5];
        params[0] = (addr >> 24) & 0xFF;
        params[1] = (addr >> 16) & 0xFF;
        params[2] = (addr >> 8) & 0xFF;
        params[3] = (addr >> 0) & 0xFF;
        params[4] = params[0] ^ params[1] ^  params[2] ^  params[3]; // XOR checksum
        resp = 1;

        return sendBytes(FLASH_SYNC, params, sizeof(params), resp);
    }
    return 0;    
}


int cmdWrite(void)
{
    flashLogDebug(FLASH_WRITE, "Cmd: WRITE MEM");
    char bytes[2] = {STM_BOOTLOADER_CMD_WRITE_MEM, ~STM_BOOTLOADER_CMD_WRITE_MEM};
    int resp = 1;
    return sendBytes(FLASH_WRITE, bytes, sizeof(bytes), resp);
}

int cmdRead(void)
{
    flashLogDebug(FLASH_WRITE, "Cmd: READ MEM");
    char bytes[2] = {STM_BOOTLOADER_CMD_READ_MEM, ~STM_BOOTLOADER_CMD_READ_MEM};
    int resp = 1;
    return sendBytes(FLASH_READ, bytes, sizeof(bytes), resp);
}

int loadAddress(flash_task_t task, const char adrMS, const char adrMI, const char adrLI, const char adrLS)
{
    char xor = adrMS ^ adrMI ^ adrLI ^ adrLS;
    char params[] = {adrMS, adrMI, adrLI, adrLS, xor};
    int resp = 1;

    // ESP_LOG_BUFFER_HEXDUMP("LOAD ADDR", params, sizeof(params), ESP_LOG_DEBUG);
    return sendBytes(task, params, sizeof(params), resp);
}


int sendBytes(flash_task_t task, const char *bytes, int count, int resp) {
    char data[resp];
    return sendReceiveBytes( task, bytes, count, data, resp);
}


int sendReceiveBytes(flash_task_t task, const char *bytes, int count, char * recv_bytes, int resp)
{
    sendData(task, bytes, count);
    int length = waitForSerialData(resp, SERIAL_TIMEOUT);

    if (length > 0) {
        if (length > resp) {
            flashLogError(task, "sendBytes: Too much data received (%d/%d). Out of sync!?", length, resp);
            return 0;
        } else if (length < resp) {
            flashLogError(task, "sendBytes: Insufficient number of data received (%d/%d)!", length, resp);
            return 0;
        }
        const int rxBytes = uart_read_bytes(UART_CONTROLLER, recv_bytes, length, 1000 / portTICK_RATE_MS);

        //ESP_LOGW(TAG_STM_PRO, "Got bytes: %d/%d", length, resp);

        if (rxBytes > 0 && recv_bytes[0] == ACK) {
            flashLogDebug(task, "Sync Success");
            //ESP_LOG_BUFFER_HEXDUMP("SYNC", recv_bytes, rxBytes, ESP_LOG_WARN);
            return rxBytes;
        } else {
            if (recv_bytes[0] == NACK) {
                flashLogError(task, "NACK received");
            } else {                
                flashLogError(task, "Sync Failure!");
            }
            for (int i=0;i<rxBytes;i++) {
                flashLogError(task, "RX byte %d/%d : 0x%x, ", i+1, rxBytes, recv_bytes[i]);
            }
            if (rxBytes>=0) {
                //ESP_LOG_BUFFER_HEXDUMP("SYNC", recv_bytes, rxBytes, ESP_LOG_ERROR);
            }
            return 0;
        }
    }
    else
    {
        flashLogError(task, "Serial Timeout");
        return 0;
    }

    return 0;
}


int sendData(flash_task_t task, const char *data, const int count)
{
    //ESP_LOG_BUFFER_HEXDUMP("SEND", data, count, ESP_LOG_WARN);
    const int txBytes = uart_write_bytes(UART_CONTROLLER, data, count);
    //logD(logName, "Wrote %d bytes", count);
    return txBytes;
}

int waitForSerialData(int dataCount, int timeout)
{
    int timer = 0;
    int length = 0;
    timeout /= 10;
    while (timer < timeout)
    {
        uart_get_buffered_data_len(UART_CONTROLLER, (size_t *)&length);
        if (length >= dataCount)
        {
            return length;
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
        timer++;
    }
    return 0;
}

void incrementLoadAddress(char *loadAddr)
{
    loadAddr[2] += 0x1;

    if (loadAddr[2] == 0)
    {
        loadAddr[1] += 0x1;

        if (loadAddr[1] == 0)
        {
            loadAddr[0] += 0x1;
        }
    }
}

esp_err_t flashBlock(const char *address, const char *data)
{
    flashLogInfo(FLASH_WRITE, "Flashing block: addr 0x%02x%02x%02x%02x", address[0], address[1], address[2], address[3]);

    cmdWrite();

    loadAddress(FLASH_WRITE, address[0], address[1], address[2], address[3]);

    //ESP_LOG_BUFFER_HEXDUMP("FLASH PAGE", data, FLASH_BLOCK_SIZE, ESP_LOG_DEBUG);

    char xor = 0xFF;
    char sz = 0xFF;

    sendData(FLASH_WRITE, &sz, 1);

    sendData(FLASH_WRITE, data, FLASH_BLOCK_SIZE);

    for (int i = 0; i < FLASH_BLOCK_SIZE; i++) {
        xor ^= data[i];
    }

    if (sendBytes( FLASH_WRITE, &xor, 1, 1)) {
        return ESP_OK;
    }
    flashLogError(FLASH_WRITE, "Error writing block!");

    return ESP_FAIL;
}

esp_err_t readBlock(const char *address, const char *data)
{
    flashLogInfo(FLASH_READ, "Reading block: addr 0x%02x%02x%02x%02x", address[0], address[1], address[2], address[3]);
    char param[] = {0xFF, 0x00};    // reading FLASH_BLOCK_SIZE bytes
    char recv_bytes[1+FLASH_BLOCK_SIZE];

    cmdRead();

    loadAddress(FLASH_READ, address[0], address[1], address[2], address[3]);

    if (!sendReceiveBytes( FLASH_READ, param, sizeof(param), recv_bytes, 1+FLASH_BLOCK_SIZE)) {
        flashLogError(FLASH_READ, "Error reading block!");
        return ESP_FAIL;
    }

    // Copy the result (omitting the 1st ACK byte)
    memcpy((void *)data, &recv_bytes[1], FLASH_BLOCK_SIZE);

    return ESP_OK;
}
