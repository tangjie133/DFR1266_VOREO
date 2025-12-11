#include "PeripheralCode.h"
#include "led_strip.h"
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "soc/uart_periph.h"
#include "esp_rom_gpio.h"
#include "driver/gpio.h"
#include "hal/gpio_hal.h"
#include "esp_console.h"
#include "linenoise/linenoise.h"
#include "driver/i2c_slave.h"
#include <string.h>
#include "esp_log.h"

#define TAG "USER_PERIPHERAL"

// CRC16-CCITT 多项式 (0x1021)
#define CRC16_POLYNOMIAL 0x1021
#define CRC16_INITIAL_VALUE 0xFFFF

// Static GAT data list instance
static gat_data_list_t *gatDataList = NULL;

static int commMode = UART_MODE;
static i2c_slave_dev_handle_t slave_handle;

// GAT解析状态机状态（需要保持跨函数调用）
static struct {
    uint8_t cmdState;      // 当前解析状态
    uint16_t dataLen;      // 期望的数据长度（用于ASR_DATA命令）
    uint8_t lastCmd;       // 上一个命令（用于非ASR_DATA命令）
} gatParseState = {
    .cmdState = GAT_UART_CMD_HEAD0,
    .dataLen = 0,
    .lastCmd = 0
},userParseState = {
    .cmdState = USER_UART_CMD_HEAD,
    .dataLen = 0,
    .lastCmd = 0
};

static QueueHandle_t userUartQueue = NULL;
static QueueHandle_t gtaUartQueue = NULL;

static led_strip_t strip =
{
    .type = LED_STRIP_WS2812,
    .length = 1,
    .gpio = RGB_GPIO,
    .buf = NULL,
#ifdef LED_STRIP_BRIGHTNESS
    .brightness = 255,
#endif
};

/**
 * @brief 计算CRC16-CCITT校验值
 * @param data 数据缓冲区
 * @param len 数据长度
 * @return CRC16校验值
 */
static uint16_t calculateCRC16(const uint8_t *data, size_t len)
{
    uint16_t crc = CRC16_INITIAL_VALUE;
    
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)(data[i] << 8);
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ CRC16_POLYNOMIAL;
            } else {
                crc <<= 1;
            }
        }
    }
    
    return crc;
}
//发送数据包
static void sendBuf(uint8_t *data, size_t len, uint8_t cmd)
{
    uint8_t buf[len + 6];
    buf[0] = USER_UART_CMD_HEAD;
    buf[1] = (len << 8) & 0xff;
    buf[2] = len & 0xff;    
    buf[3] = cmd;
    memcpy(buf + 4, data, len);
    uint16_t crc = calculateCRC16(buf, len + 4);
    buf[len + 4] = crc >> 8;
    buf[len + 5] = crc & 0xFF;
    if(commMode == UART_MODE){
        uart_write_bytes(USERUART_PORT, buf, len + 6);
    }else if(commMode == I2C_MODE){
        i2c_slave_transmit(&slave_handle, buf, len + 6, -1);
    }
}

static void userParseData(const uint8_t *data, size_t len)
{
    if (!data || len == 0) {
        ESP_LOGW(TAG, "Invalid USER data: NULL pointer or zero length");
        return;
    }

    for(int i = 0; i < len; i++)
    {
        if(userParseState.cmdState == USER_UART_CMD_HEAD){
            if(data[i] == USER_UART_CMD_HEAD)
            {
                userParseState.cmdState = UART_CMD_LEN_H;
            }
        }else if(userParseState.cmdState == UART_CMD_LEN_H){
            userParseState.dataLen = (uint16_t)data[i] << 8;
            userParseState.cmdState = UART_CMD_LEN_L;
        }else if(userParseState.cmdState == UART_CMD_LEN_L){
            userParseState.dataLen |= (uint16_t)data[i];
            userParseState.cmdState = UART_CMD_CMD;
        }else if(userParseState.cmdState == UART_CMD_CMD){
            switch(data[i])
            {
                case USER_UART_CMD_QUERY_TEXT:
                    ESP_LOGI(TAG, "USER query text command received");
                    userParseState.cmdState = USER_UART_CMD_HEAD;
                    uint8_t sendData = 0;
                    if(gatDataListGetCount(gatDataList) > 0){
                        sendData = 1;
                    }
                    sendBuf(&sendData, 1, USER_UART_CMD_QUERY_TEXT);
                    break;
                case USER_UART_CMD_GET_TEXT:
                    ESP_LOGI(TAG, "USER get text command received");
                    userParseState.cmdState = USER_UART_CMD_HEAD;
                    break;
                case USER_UART_CMD_SEND_TEXT:
                    ESP_LOGI(TAG, "USER send text command received");
                    userParseState.cmdState = USER_UART_CMD_HEAD;
                    break;
                case USER_UART_CMD_ANGLE:
                    userParseState.cmdState = USER_UART_CMD_HEAD;
                    break;
                default:
                    ESP_LOGW(TAG, "Unknown USER command received: 0x%02X, reset state", data[i]);
                    userParseState.cmdState = USER_UART_CMD_HEAD;
                    break;
            }
        }
    }
}

static void userUartTask(void *pvParameters)
{
    uart_event_t event;
    int dataLen = 0;
    uint8_t* dtmp = (uint8_t*) malloc(UART_READ_BUF_SIZE);
    assert(dtmp);
    while (1)
    {
        if(commMode == UART_MODE){
            dataLen = uart_read_bytes(USERUART_PORT, dtmp, UART_READ_BUF_SIZE, 0);
        }else if(commMode == I2C_MODE){
            dataLen = i2c_slave_receive(slave_handle, dtmp, UART_READ_BUF_SIZE);
            if(dataLen == ESP_OK){
                ESP_LOGI(TAG, "I2C_SLAVE_NUM receive data success");
                dataLen = (dtmp[1] << 8 | dtmp[2]) + 5;
            }else{
                dataLen = 0;
                ESP_LOGE(TAG, "I2C_SLAVE_NUM receive data failed");
            }
        }
        if (dataLen > 0) {
            // CRC16需要至少3个字节（1字节数据 + 2字节CRC）
            if (dataLen < 3) {
                ESP_LOGW(TAG, "USER UART data too short for CRC16: %d bytes", dataLen);
                continue;
            }
            
            // 计算CRC16（不包括最后2个字节，它们是CRC16值）
            uint16_t calculated_crc = calculateCRC16(dtmp, dataLen - 2);
            
            // 从数据包中提取接收到的CRC16（高字节在前，低字节在后）
            uint16_t received_crc = (uint16_t)dtmp[dataLen - 2] << 8 | dtmp[dataLen - 1];
            
            // 验证CRC16
            if (calculated_crc == received_crc) {
                // 校验成功，解析数据（不包括CRC16的2个字节）
                userParseData(dtmp, dataLen - 2);
            } else {
                // 校验失败，跳过该数据包
                ESP_LOGW(TAG, "USER UART CRC16 error: calculated=0x%04X, received=0x%04X", 
                         calculated_crc, received_crc);
            }
        } 
        // No data available, yield CPU
        vTaskDelay(1);
    }
}
/**
 * @brief Parse GTA communication data
 * @param data Pointer to received data buffer
 * @param len Length of received data
 */
static void gtaParseData(const uint8_t *data, size_t len)
{
    if (!data || len == 0) {
        ESP_LOGW(TAG, "Invalid GTA data: NULL pointer or zero length");
        return;
    }
    
    ESP_LOGI(TAG, "GTA data received, length: %d, current state: %d", len, gatParseState.cmdState);
    
    for(int i = 0; i < len; i++)
    {
        if(gatParseState.cmdState == GAT_UART_CMD_HEAD0){
            if(data[i] == GAT_CMD_HEAD0)
            {
                gatParseState.cmdState = GAT_UART_CMD_HEAD1;
            }
            // 如果不是HEAD0，继续保持在HEAD0状态，等待下一个字节
        }
        else if(gatParseState.cmdState == GAT_UART_CMD_HEAD1){
            if(data[i] == GAT_CMD_HEAD1)
            {
                gatParseState.cmdState = UART_CMD_CMD;
            }else{
                // 如果HEAD1不匹配，检查当前字节是否是新的HEAD0
                if(data[i] == GAT_CMD_HEAD0) {
                    gatParseState.cmdState = GAT_UART_CMD_HEAD1; // 可能是新的数据包开始
                } else {
                    gatParseState.cmdState = GAT_UART_CMD_HEAD0; // 重置状态
                }
            }
        }else if(gatParseState.cmdState == UART_CMD_CMD){
           switch(data[i])
           {
            case GAT_CMD_WAKEUP:
                ESP_LOGI(TAG, "GTA wakeup command received");
                setRGBColor(RGB_GREEN);
                gatParseState.lastCmd = GAT_CMD_WAKEUP;
                gatParseState.cmdState = GAT_UART_CMD_END;
                break;
            case GAT_CMD_DETECTED:
                ESP_LOGI(TAG, "GTA detected command received");
                setRGBColor(RGB_BLUE);
                gatParseState.lastCmd = GAT_CMD_DETECTED;
                gatParseState.cmdState = GAT_UART_CMD_END;
                break;
            case GAT_CMD_ASR_DATA:
                gatParseState.cmdState = UART_CMD_LEN_H;
                gatParseState.dataLen = 0;
                ESP_LOGI(TAG, "GTA ASR data command received");
                break;
            case GAT_CMD_READ_ERROR:
                ESP_LOGI(TAG, "GTA read error command received");
                gatParseState.lastCmd = GAT_CMD_READ_ERROR;
                gatParseState.cmdState = GAT_UART_CMD_END;
                break;
            default:
                ESP_LOGW(TAG, "Unknown GTA command received: 0x%02X, reset state", data[i]);
                gatParseState.cmdState = GAT_UART_CMD_HEAD0; // 重置状态，避免卡住
                break;
           }
        }else if(gatParseState.cmdState == UART_CMD_LEN_H){
            gatParseState.dataLen = (uint16_t)data[i] << 8;
            gatParseState.cmdState = UART_CMD_LEN_L;
        }else if(gatParseState.cmdState == UART_CMD_LEN_L){
            gatParseState.dataLen |= (uint16_t)data[i];
            gatParseState.cmdState = GAT_UART_CMD_DATA;
            // Create a new node for ASR data and add to list
            if (gatDataList && gatParseState.dataLen > 0) {
                gat_data_node_t *node = gatDataNodeCreate(GAT_CMD_ASR_DATA, gatParseState.dataLen);
                if (node) {
                    gatDataListAdd(gatDataList, node);
                    ESP_LOGI(TAG, "Created GAT data node, expected length: %d", gatParseState.dataLen);
                } else {
                    ESP_LOGE(TAG, "Failed to create GAT data node, reset state");
                    gatParseState.cmdState = GAT_UART_CMD_HEAD0; // Reset on error
                    gatParseState.dataLen = 0;
                }
            } else {
                ESP_LOGW(TAG, "Invalid data length or list not initialized, reset state");
                gatParseState.cmdState = GAT_UART_CMD_HEAD0;
                gatParseState.dataLen = 0;
            }
        }else if(gatParseState.cmdState == GAT_UART_CMD_DATA){
            // Get the current incomplete node and append data
            if (gatDataList) {
                gat_data_node_t *node = gatDataListGetIncomplete(gatDataList);
                if (node) {
                    int appended = gatDataNodeAppend(node, &data[i], 1);
                    if (appended > 0) {
                        if (gatDataNodeIsComplete(node)) {
                            ESP_LOGI(TAG, "GAT data packet complete, total length: %d", node->received_len);
                            gatParseState.cmdState = GAT_UART_CMD_END;
                        }
                        // 如果数据未完成，继续保持在DATA状态
                    } else {
                        ESP_LOGW(TAG, "Failed to append data to node, reset state");
                        gatParseState.cmdState = GAT_UART_CMD_HEAD0; // Reset on error
                        gatParseState.dataLen = 0;
                    }
                } else {
                    ESP_LOGW(TAG, "No incomplete node found for data, reset state");
                    gatParseState.cmdState = GAT_UART_CMD_HEAD0; // Reset on error
                    gatParseState.dataLen = 0;
                }
            } else {
                ESP_LOGW(TAG, "GAT data list not initialized, reset state");
                gatParseState.cmdState = GAT_UART_CMD_HEAD0;
                gatParseState.dataLen = 0;
            }
        }
        else if(gatParseState.cmdState == GAT_UART_CMD_END){
            if(data[i] == GAT_CMD_END)
            {
                // 数据包解析完成，重置状态准备接收下一个数据包
                ESP_LOGI(TAG, "GAT packet end received, command: 0x%02X", gatParseState.lastCmd);
                gatParseState.cmdState = GAT_UART_CMD_HEAD0;
                gatParseState.dataLen = 0;
                gatParseState.lastCmd = 0;
                // 不continue，继续处理后续字节（可能包含下一个数据包）
            }else{
                // 期望收到结束符但收到其他字节，可能是错误或下一个数据包的开始
                ESP_LOGW(TAG, "Expected END (0xFF) but received 0x%02X, reset state", data[i]);
                if(data[i] == GAT_CMD_HEAD0) {
                    // 可能是下一个数据包开始
                    gatParseState.cmdState = GAT_UART_CMD_HEAD1;
                } else {
                    gatParseState.cmdState = GAT_UART_CMD_HEAD0;
                }
            }
        }
    }
}

// ==================== GAT Data List Implementation ====================

gat_data_list_t* gatDataListInit(void)
{
    gat_data_list_t *list = (gat_data_list_t*)malloc(sizeof(gat_data_list_t));
    if (!list) {
        ESP_LOGE(TAG, "Failed to allocate GAT data list");
        return NULL;
    }
    list->head = NULL;
    list->tail = NULL;
    list->count = 0;
    return list;
}

gat_data_node_t* gatDataNodeCreate(uint8_t cmd, uint16_t data_len)
{
    if (data_len == 0 || data_len > GAT_DATA_BUFFER_MAX_SIZE) {
        ESP_LOGE(TAG, "Invalid data length: %d", data_len);
        return NULL;
    }
    
    gat_data_node_t *node = (gat_data_node_t*)malloc(sizeof(gat_data_node_t));
    if (!node) {
        ESP_LOGE(TAG, "Failed to allocate GAT data node");
        return NULL;
    }
    
    node->data = (uint8_t*)malloc(data_len);
    if (!node->data) {
        ESP_LOGE(TAG, "Failed to allocate data buffer");
        free(node);
        return NULL;
    }
    
    node->cmd = cmd;
    node->data_len = data_len;
    node->received_len = 0;
    node->next = NULL;
    
    return node;
}

int gatDataListAdd(gat_data_list_t *list, gat_data_node_t *node)
{
    if (!list || !node) {
        return -1;
    }
    
    if (list->head == NULL) {
        list->head = node;
        list->tail = node;
    } else {
        list->tail->next = node;
        list->tail = node;
    }
    list->count++;
    return 0;
}

int gatDataNodeAppend(gat_data_node_t *node, const uint8_t *data, size_t len)
{
    if (!node || !data || len == 0) {
        return -1;
    }
    
    size_t remaining = node->data_len - node->received_len;
    size_t to_copy = (len < remaining) ? len : remaining;
    
    if (to_copy > 0) {
        memcpy(node->data + node->received_len, data, to_copy);
        node->received_len += to_copy;
    }
    
    return to_copy;
}

bool gatDataNodeIsComplete(gat_data_node_t *node)
{
    if (!node) {
        return false;
    }
    return (node->received_len >= node->data_len);
}

int gatDataListRemove(gat_data_list_t *list, gat_data_node_t *node)
{
    if (!list || !node) {
        return -1;
    }
    
    if (list->head == node) {
        list->head = node->next;
        if (list->tail == node) {
            list->tail = NULL;
        }
    } else {
        gat_data_node_t *prev = list->head;
        while (prev && prev->next != node) {
            prev = prev->next;
        }
        if (prev) {
            prev->next = node->next;
            if (list->tail == node) {
                list->tail = prev;
            }
        } else {
            return -1; // Node not found in list
        }
    }
    
    list->count--;
    gatDataNodeFree(node);
    return 0;
}

gat_data_node_t* gatDataListGetIncomplete(gat_data_list_t *list)
{
    if (!list) {
        return NULL;
    }
    
    gat_data_node_t *current = list->head;
    while (current) {
        if (!gatDataNodeIsComplete(current)) {
            return current;
        }
        current = current->next;
    }
    return NULL;
}

gat_data_node_t* gatDataListGetComplete(gat_data_list_t *list)
{
    if (!list) {
        return NULL;
    }
    
    gat_data_node_t *current = list->head;
    while (current) {
        if (gatDataNodeIsComplete(current)) {
            return current;
        }
        current = current->next;
    }
    return NULL;
}

void gatDataNodeFree(gat_data_node_t *node)
{
    if (node) {
        if (node->data) {
            free(node->data);
            node->data = NULL;
        }
        free(node);
    }
}

void gatDataListFree(gat_data_list_t *list)
{
    if (!list) {
        return;
    }
    
    gat_data_node_t *current = list->head;
    while (current) {
        gat_data_node_t *next = current->next;
        gatDataNodeFree(current);
        current = next;
    }
    
    free(list);
}

uint32_t gatDataListGetCount(gat_data_list_t *list)
{
    if (!list) {
        return 0;
    }
    return list->count;
}

static void gtaUartTask(void *pvParameters)
{
    uint8_t* dtmp = (uint8_t*) malloc(UART_READ_BUF_SIZE + 1);
    assert(dtmp);
    while (1)
    {
        // Option 1: Polling mode - read available data (non-blocking)
        // This reads up to UART_READ_BUF_SIZE bytes, returns immediately if less data available
        int len = uart_read_bytes(GTA_UART_PORT, dtmp, UART_READ_BUF_SIZE, 0);
        if (len > 0) {
            gtaParseData(dtmp, len);
        } else {
            // No data available, yield CPU
            vTaskDelay(1);
        }
    }
}


int peripheralInit(void)
{
    //初始化RGB灯
    led_strip_install();
    int ret = led_strip_init(&strip);
    
    //初始化GAT数据链表
    gatDataList = gatDataListInit();
    if (!gatDataList) {
        ESP_LOGE(TAG, "Failed to initialize GAT data list");
        return -1;
    }
    ESP_LOGI(TAG, "GAT data list initialized");
    uart_config_t config = {
        .baud_rate = UART_BAUDRATE,
        .data_bits = UART_DATA_BITS,
        .stop_bits = UART_STOP_BITS,
        .parity = UART_PARITY,
        .flow_ctrl = UART_FLOW_CTRL,
        .source_clk = UART_SOURCE_CLK,
    };
    //初始化GTA UART
    uart_driver_install(GTA_UART_PORT, UART_BUF_SIZE * 2, UART_BUF_SIZE * 2, 0, NULL, 0);
    uart_param_config(GTA_UART_PORT, &config);
    uart_set_pin(GTA_UART_PORT, GTA_UART_TX_GPIO, GTA_UART_RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    xTaskCreate(gtaUartTask, "gtaUartTask", 8*1024, NULL, 10, NULL);
    // 配置 GPIO 为输入
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,      // 不使用中断
        .mode = GPIO_MODE_INPUT,             // 输入模式
        .pin_bit_mask = (1ULL << MODE_GPIO),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE     // 启动内部上拉（适合按键）
    };
    gpio_config(&io_conf);

    commMode = gpio_get_level(MODE_GPIO);
    if (commMode == I2C_MODE) {
        i2c_slave_config_t i2c_slv_config = {
            .addr_bit_len = I2C_ADDR_BIT_LEN_7,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .i2c_port = I2C_SLAVE_NUM,
            .send_buf_depth = 256,
            .scl_io_num = USERUART_RX_SCL_GPIO,
            .sda_io_num = USERUART_TX_SDA_GPIO,
            .slave_addr = USERI2C_ADDRESS,
        };
    
        
        i2c_new_slave_device(&i2c_slv_config, &slave_handle);
        ESP_LOGI(TAG, "I2C_SLAVE_NUM is initialized");
    } else {
        uart_driver_install(USERUART_PORT, UART_BUF_SIZE * 2, UART_BUF_SIZE * 2, 0, NULL, 0);
        uart_param_config(USERUART_PORT, &config);
        uart_set_pin(USERUART_PORT, USERUART_TX_SDA_GPIO, USERUART_RX_SCL_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
        ESP_LOGI(TAG, "USERUART_PORT is initialized");
    }
    xTaskCreate(userUartTask, "userUartTask", 8*1024, NULL, 10, NULL);
    return ret;
}

int setRGBColor(rgb_t color)
{
    led_strip_set_pixel(&strip, 0, color);
    return led_strip_flush(&strip);
}


