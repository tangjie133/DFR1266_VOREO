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

// GAT数据链表实例
static gat_data_list_t *gatDataList = NULL;

static int commMode = UART_MODE;
extern float angle; 

static i2c_slave_dev_handle_t slave_handle = NULL;

static QueueHandle_t slaveReceiveQueue = NULL;
static QueueHandle_t i2cDataQueue = NULL;  // I2C接收数据队列，用于传递给userUartTask
static i2c_slave_rx_done_event_data_t rx_data;
static uint8_t i2cSlaveData[UART_READ_BUF_SIZE];
static uint8_t i2cSlaveSendData[UART_READ_BUF_SIZE];
static uint16_t i2cSlaveDataLen = 0;
static uint16_t i2cSlaveSendDataLne = 0;
static uint8_t i2cSlaveSendStatic = I2C_SLAVE_SEND_STATIC_ERROR;

// I2C分包接收缓冲区
static uint8_t i2cRxBuffer[UART_READ_BUF_SIZE];  // 接收缓冲区
static uint16_t i2cRxBufferLen = 0;              // 当前接收缓冲区中的数据长度
static uint16_t i2cExpectedPacketLen = 0;         // 期望的数据包总长度（根据包头和长度字段计算）

// I2C接收数据结构
typedef struct {
    uint8_t data[UART_READ_BUF_SIZE];
    uint16_t len;
} i2c_received_data_t;

typedef enum {
    I2C_SLAVE_EVT_RX,
    I2C_SLAVE_EVT_TX
} i2c_slave_event_t;

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
    .brightness = 100,
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
        // 先转换为uint16_t再左移，避免符号扩展问题
        crc ^= ((uint16_t)data[i]) << 8;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = ((crc << 1) ^ CRC16_POLYNOMIAL) & 0xFFFF;
            } else {
                crc = (crc << 1) & 0xFFFF;
            }
        }
    }
    
    return crc;
}
/**
 * @brief 发送数据包（自动添加包头和CRC16校验）
 * @param data 要发送的数据缓冲区
 * @param len 数据长度
 * @param cmd 命令类型
 */
static void sendBuf(uint8_t *data, size_t len, uint8_t cmd)
{
    // 限制最大数据长度，避免栈溢出
    const size_t MAX_PACKET_SIZE = GAT_DATA_BUFFER_MAX_SIZE + 6;
    if (len > GAT_DATA_BUFFER_MAX_SIZE) {
        ESP_LOGE(TAG, "数据长度超出限制: %d > %d", len, GAT_DATA_BUFFER_MAX_SIZE);
        return;
    }
    
    // 使用固定大小的缓冲区，避免变长数组导致的栈溢出
    static uint8_t send_buffer[GAT_DATA_BUFFER_MAX_SIZE + 6];
    size_t total_len = len + 6;
    
    send_buffer[0] = USER_UART_CMD_HEAD;
    send_buffer[1] = (len >> 8) & 0xff;
    send_buffer[2] = len & 0xff;    
    send_buffer[3] = cmd;
    memcpy(send_buffer + 4, data, len);
    uint16_t crc = calculateCRC16(send_buffer, len + 4);
    send_buffer[len + 4] = (uint8_t)(crc >> 8);
    send_buffer[len + 5] = (uint8_t)(crc & 0xFF);
    
    if(commMode == UART_MODE){
        // 使用非阻塞方式写入，避免阻塞导致看门狗超时
        int written = uart_write_bytes(USERUART_PORT, send_buffer, total_len);
        if (written < 0) {
            ESP_LOGE(TAG, "UART写入失败");
        } else if ((size_t)written < total_len) {
            ESP_LOGW(TAG, "UART部分写入: %d/%d", written, total_len);
        }
        // 刷新发送缓冲区，确保数据发送完成
        uart_wait_tx_done(USERUART_PORT, pdMS_TO_TICKS(100));
    }else if(commMode == I2C_MODE){
        ESP_LOGI(TAG, "I2C写入数据，长度: %d", total_len);
        i2cSlaveSendStatic = I2C_SLAVE_SEND_STATIC_LEN;
        i2cSlaveSendDataLne = total_len;
        memcpy(i2cSlaveSendData, send_buffer, total_len);
        //uint32_t write_len = 0;
        //i2c_slave_write(slave_handle, send_buffer, total_len, &write_len, pdMS_TO_TICKS(1000));
        
    }
}

/**
 * @brief 解析用户UART数据（状态机实现）
 * @param data 接收到的数据缓冲区
 * @param len 数据长度
 */
static void userParseData(const uint8_t *data, size_t len)
{
    uint8_t sendData = 0;
    if (!data || len == 0) {
        ESP_LOGW(TAG, "无效的用户数据：空指针或长度为零");
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
                    
                    // 检查是否有完整的USER_UART_CMD_SEND_TEXT节点
                    sendData = 0;
                    if (gatDataList) {
                        gat_data_node_t *current = gatDataList->head;
                        while (current) {
                            if (current->cmd == USER_UART_CMD_SEND_TEXT && 
                                gatDataNodeIsComplete(current) && 
                                current->data && 
                                current->received_len > 0) {
                                sendData = 1;
                                break;
                            }
                            current = current->next;
                        }
                    }
                    sendBuf(&sendData, 1, USER_UART_CMD_QUERY_TEXT);
                    break;
                case USER_UART_CMD_GET_TEXT:
                    ESP_LOGI(TAG, "USER get text command received");
                    userParseState.cmdState = USER_UART_CMD_HEAD;
                    // 从链表中查找完整的USER_UART_CMD_SEND_TEXT节点
                    if (gatDataList) {
                        gat_data_node_t *node = NULL;
                        // 遍历链表查找USER_UART_CMD_SEND_TEXT命令的完整节点
                        gat_data_node_t *current = gatDataList->head;
                        while (current) {
                            if (current->cmd == USER_UART_CMD_SEND_TEXT && 
                                gatDataNodeIsComplete(current) && 
                                current->data && 
                                current->received_len > 0) {
                                node = current;
                                break;
                            }
                            current = current->next;
                        }
                        
                        if (node) {
                            // 发送数据（sendBuf会自动添加包头和CRC16）
                            sendBuf(node->data, node->received_len, USER_UART_CMD_GET_TEXT);
                            ESP_LOGI(TAG, "Sent text data, length: %d", node->received_len);
                            // 从链表中移除已发送的节点（gatDataListRemove内部会释放节点）
                            gatDataListRemove(gatDataList, node);
                        } else {
                            // 没有可用的文本数据，发送空数据
                            uint8_t emptyData[1] = {0};
                            sendBuf(emptyData, 1, USER_UART_CMD_GET_TEXT);
                            ESP_LOGW(TAG, "No text data available to send");
                        }
                    } else {
                        // 链表未初始化，发送空数据
                        uint8_t emptyData[1] = {0};
                        sendBuf(emptyData, 1, USER_UART_CMD_GET_TEXT);
                        ESP_LOGE(TAG, "GAT data list not initialized");
                    }
                    break;
                case USER_UART_CMD_SEND_TEXT:
                    ESP_LOGI(TAG, "USER send text command received, expected data length: %d", userParseState.dataLen);
                    // 如果有数据需要接收，创建节点并进入数据接收状态
                    if (userParseState.dataLen > 0) {
                        if (gatDataList) {
                            gat_data_node_t *node = gatDataNodeCreate(USER_UART_CMD_SEND_TEXT, userParseState.dataLen);
                            if (node) {
                                gatDataListAdd(gatDataList, node);
                                userParseState.cmdState = USER_UART_CMD_DATA;
                                ESP_LOGI(TAG, "Created USER text data node, expected length: %d", userParseState.dataLen);
                            } else {
                                ESP_LOGE(TAG, "Failed to create USER text data node, reset state");
                                userParseState.cmdState = USER_UART_CMD_HEAD;
                                userParseState.dataLen = 0;
                            }
                        } else {
                            ESP_LOGE(TAG, "GAT data list not initialized, reset state");
                            userParseState.cmdState = USER_UART_CMD_HEAD;
                            userParseState.dataLen = 0;
                        }
                    } else {
                        // 没有数据，直接返回
                        ESP_LOGW(TAG, "USER send text command with zero data length");
                        userParseState.cmdState = USER_UART_CMD_HEAD;
                    }
                    break;
                case USER_UART_CMD_ANGLE:
                    userParseState.cmdState = USER_UART_CMD_HEAD;   
                    uint8_t sendBufData[2];
                    sendBufData[0] = ((uint16_t)angle >> 8) & 0xff;
                    sendBufData[1] = ((uint16_t)angle & 0xff);
                    sendBuf(sendBufData, 2, USER_UART_CMD_ANGLE);   
                    break;
                case USER_UART_CMD_BEGIN:
                    ESP_LOGI(TAG, "USER begin command received");
                    userParseState.cmdState = USER_UART_CMD_HEAD;
                    sendData = 1;
                    sendBuf(&sendData, 1, USER_UART_CMD_BEGIN);
                    break;
                default:
                    ESP_LOGW(TAG, "Unknown USER command received: 0x%02X, reset state", data[i]);
                    userParseState.cmdState = USER_UART_CMD_HEAD;
                    break;
            }
        } else if(userParseState.cmdState == USER_UART_CMD_DATA){
            // 接收文本数据并存储到节点中
            if (gatDataList) {
                gat_data_node_t *node = gatDataListGetIncomplete(gatDataList);
                if (node && node->cmd == USER_UART_CMD_SEND_TEXT) {
                    int appended = gatDataNodeAppend(node, &data[i], 1);
                    if (appended > 0) {
                        if (gatDataNodeIsComplete(node)) {
                            ESP_LOGI(TAG, "USER text data packet complete, total length: %d", node->received_len);
                            userParseState.cmdState = USER_UART_CMD_HEAD;
                            userParseState.dataLen = 0;
                        }
                        // 如果数据未完成，继续保持在DATA状态
                    } else {
                        ESP_LOGW(TAG, "Failed to append data to USER text node, reset state");
                        userParseState.cmdState = USER_UART_CMD_HEAD;
                        userParseState.dataLen = 0;
                    }
                } else {
                    ESP_LOGW(TAG, "No incomplete USER text node found, reset state");
                    userParseState.cmdState = USER_UART_CMD_HEAD;
                    userParseState.dataLen = 0;
                }
            } else {
                ESP_LOGW(TAG, "GAT data list not initialized, reset state");
                userParseState.cmdState = USER_UART_CMD_HEAD;
                userParseState.dataLen = 0;
            }
        }
    }
}

/**
 * @brief I2C从机任务（处理I2C接收和发送事件）
 * @param pvParameters 任务参数（未使用）
 */
static void i2cSlaveTask(void *pvParameters)
{
    i2c_slave_event_t event;
    uint32_t write_len = 0;
    static uint16_t write_data_len = 0;  // 静态变量，保持发送进度
    static uint8_t last_rx_cmd = 0;  // 保存最近接收到的命令
    
    while (1)
    {
        if(xQueueReceive(slaveReceiveQueue, &event, portMAX_DELAY) == pdTRUE){
            if(event == I2C_SLAVE_EVT_TX){
                ESP_LOGI(TAG, "I2C从机请求发送数据");
                setRGBColor(RGB_YELLOW);
                
                if(i2cSlaveSendStatic == I2C_SLAVE_SEND_STATIC_LEN){
                    if(last_rx_cmd == I2C_SLAVE_CMD_GET_LEN){
                        ESP_LOGI(TAG, "I2C从机请求发送数据，长度: %d", i2cSlaveSendDataLne);
                        uint8_t getLenData[2] = {0};
                        getLenData[0] = (uint8_t)(i2cSlaveSendDataLne >> 8);
                        getLenData[1] = (uint8_t)(i2cSlaveSendDataLne & 0xff);
                        i2c_slave_write(slave_handle, getLenData, 2, &write_len, pdMS_TO_TICKS(1000));
                        i2cSlaveSendStatic = I2C_SLAVE_SEND_STATIC_DATA;
                        write_data_len = 0;  // 重置写入长度
                        last_rx_cmd = 0;  // 清除命令
                    }else{
                        ESP_LOGW(TAG, "I2C从机请求发送数据失败，未收到GET_LEN命令");
                        setRGBColor(RGB_BLUE);
                        uint8_t errorData[2] = {0xff, 0xff};
                        i2c_slave_write(slave_handle, errorData, 2, &write_len, pdMS_TO_TICKS(1000));
                        i2cSlaveSendStatic = I2C_SLAVE_SEND_STATIC_ERROR;
                    }
                    
                }else if(i2cSlaveSendStatic == I2C_SLAVE_SEND_STATIC_DATA){
                    if(i2cSlaveSendDataLne < 32){
                        i2c_slave_write(slave_handle, i2cSlaveSendData, i2cSlaveSendDataLne, &write_len, pdMS_TO_TICKS(1000));
                        i2cSlaveSendStatic = I2C_SLAVE_SEND_STATIC_ERROR;
                    }else{
                        // 计算剩余需要发送的数据长度
                        uint16_t remaining = i2cSlaveSendDataLne - write_data_len;
                        if(remaining > 32){
                            // 还有超过32字节的数据，发送32字节
                            ESP_LOGI(TAG, "I2C从机请求发送数据，长度: %d", 32);
                            i2c_slave_write(slave_handle, i2cSlaveSendData + write_data_len, 32, &write_len, pdMS_TO_TICKS(1000));
                            // 使用实际写入的长度更新进度
                            write_data_len += write_len;
                        }else{
                            // 发送末尾数据（剩余数据 <= 32字节）
                            ESP_LOGI(TAG, "I2C从机请求发送数据，长度: %d", remaining);
                            i2c_slave_write(slave_handle, i2cSlaveSendData + write_data_len, remaining, &write_len, pdMS_TO_TICKS(1000));
                            // 使用实际写入的长度更新进度
                            write_data_len += write_len;
                            // 检查是否发送完成
                            if(write_data_len >= i2cSlaveSendDataLne){
                                i2cSlaveSendStatic = I2C_SLAVE_SEND_STATIC_ERROR;
                                write_data_len = 0;  // 重置写入长度
                            }
                        }
                    }
                }
                else{
                    ESP_LOGW(TAG, "I2C从机请求发送数据失败，状态错误");
                    setRGBColor(RGB_BLUE);
                    uint8_t errorData[2] = {0xff, 0xff};
                    write_len = i2c_slave_write(slave_handle, errorData, 2, &write_len, pdMS_TO_TICKS(1000));
                    i2cSlaveSendStatic = I2C_SLAVE_SEND_STATIC_ERROR;
                }
            }else if(event == I2C_SLAVE_EVT_RX){
                ESP_LOGI(TAG, "I2C从机接收数据成功，长度: %d", i2cSlaveDataLen);
                setRGBColor(RGB_GREEN);
                
                // 检查是否是GET_LEN命令（单字节命令，且缓冲区为空）
                if(i2cSlaveDataLen == 1 && i2cRxBufferLen == 0 && i2cSlaveData[0] == I2C_SLAVE_CMD_GET_LEN){
                    // 保存命令，等待TX事件处理
                    last_rx_cmd = I2C_SLAVE_CMD_GET_LEN;
                    ESP_LOGI(TAG, "收到I2C GET_LEN命令");
                    i2cSlaveDataLen = 0;
                    continue;
                }
                
                // 分包接收处理
                if(i2cSlaveDataLen > 0){
                    // 检查缓冲区是否有足够空间
                    if(i2cRxBufferLen + i2cSlaveDataLen > UART_READ_BUF_SIZE){
                        ESP_LOGW(TAG, "I2C接收缓冲区溢出，重置接收状态");
                        i2cRxBufferLen = 0;
                        i2cExpectedPacketLen = 0;
                    }else{
                        // 将新接收的数据追加到缓冲区
                        memcpy(i2cRxBuffer + i2cRxBufferLen, i2cSlaveData, i2cSlaveDataLen);
                        i2cRxBufferLen += i2cSlaveDataLen;
                        
                        // 如果还没有确定期望长度，且缓冲区至少有3个字节（包头+长度字段）
                        if(i2cExpectedPacketLen == 0 && i2cRxBufferLen >= 3){
                            // 检查包头
                            if(i2cRxBuffer[0] == USER_UART_CMD_HEAD){
                                // 解析数据包长度字段（字节1-2，高字节在前）
                                uint16_t dataLen = ((uint16_t)i2cRxBuffer[1] << 8) | i2cRxBuffer[2];
                                // 总长度 = 包头(1) + 长度(2) + 命令(1) + 数据(dataLen) + CRC(2) = 6 + dataLen
                                i2cExpectedPacketLen = 6 + dataLen;
                                ESP_LOGI(TAG, "I2C开始接收数据包，数据长度: %d，期望总长度: %d，当前长度: %d", 
                                         dataLen, i2cExpectedPacketLen, i2cRxBufferLen);
                            }else{
                                // 包头错误，重置
                                ESP_LOGW(TAG, "I2C接收数据包头错误: 0x%02X，重置接收状态", i2cRxBuffer[0]);
                                i2cRxBufferLen = 0;
                                i2cExpectedPacketLen = 0;
                            }
                        }
                        
                        // 如果已经确定了期望长度，检查数据包是否完整
                        if(i2cExpectedPacketLen > 0 && i2cRxBufferLen >= i2cExpectedPacketLen){
                            // 数据包完整，发送到处理队列进行CRC校验
                            if(i2cDataQueue != NULL){
                                i2c_received_data_t received_data;
                                memcpy(received_data.data, i2cRxBuffer, i2cExpectedPacketLen);
                                received_data.len = i2cExpectedPacketLen;
                                if(xQueueSend(i2cDataQueue, &received_data, pdMS_TO_TICKS(100)) != pdTRUE){
                                    ESP_LOGW(TAG, "I2C数据队列已满，丢弃数据包");
                                }else{
                                    ESP_LOGI(TAG, "I2C数据包接收完成，长度: %d", i2cExpectedPacketLen);
                                }
                            }
                            
                            // 如果还有剩余数据，保留在缓冲区（可能是下一个数据包的开头）
                            if(i2cRxBufferLen > i2cExpectedPacketLen){
                                uint16_t remaining = i2cRxBufferLen - i2cExpectedPacketLen;
                                memmove(i2cRxBuffer, i2cRxBuffer + i2cExpectedPacketLen, remaining);
                                i2cRxBufferLen = remaining;
                                i2cExpectedPacketLen = 0;  // 重置期望长度，重新解析
                                
                                // 检查剩余数据是否包含完整的数据包头部
                                if(i2cRxBufferLen >= 3 && i2cRxBuffer[0] == USER_UART_CMD_HEAD){
                                    uint16_t dataLen = ((uint16_t)i2cRxBuffer[1] << 8) | i2cRxBuffer[2];
                                    // 总长度 = 包头(1) + 长度(2) + 命令(1) + 数据(dataLen) + CRC(2) = 6 + dataLen
                                    i2cExpectedPacketLen = 6 + dataLen;
                                    ESP_LOGI(TAG, "I2C检测到下一个数据包，数据长度: %d，期望总长度: %d", 
                                             dataLen, i2cExpectedPacketLen);
                                }
                            }else{
                                // 数据包处理完成，重置状态
                                i2cRxBufferLen = 0;
                                i2cExpectedPacketLen = 0;
                            }
                        }
                    }
                }
                i2cSlaveDataLen = 0;
            }
        }
    }
}

/**
 * @brief I2C数据处理任务（处理I2C接收到的数据，CRC校验和解析）
 * @param pvParameters 任务参数（未使用）
 */
static void i2cDataProcessTask(void *pvParameters)
{
    i2c_received_data_t received_data;
    uint8_t* dtmp = (uint8_t*) malloc(UART_READ_BUF_SIZE + 1);
    assert(dtmp);
    
    while (1)
    {
        // 从队列接收I2C数据
        if(i2cDataQueue != NULL && xQueueReceive(i2cDataQueue, &received_data, portMAX_DELAY) == pdTRUE){
            int dataLen = received_data.len;
            memcpy(dtmp, received_data.data, dataLen);
            
            if (dataLen > 0) {
                // CRC16需要至少3个字节（1字节数据 + 2字节CRC）
                if (dataLen < 3) {
                    ESP_LOGW(TAG, "I2C数据太短，无法进行CRC16校验：%d 字节", dataLen);
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
                    ESP_LOGW(TAG, "I2C CRC16校验错误：计算值=0x%04X，接收值=0x%04X", 
                             calculated_crc, received_crc);
                }
            }
        }
    }
}

/**
 * @brief 用户UART任务（仅处理UART数据）
 * @param pvParameters 任务参数（未使用）
 */
static void userUartTask(void *pvParameters)
{
    int dataLen = 0;
    uint8_t* dtmp = (uint8_t*) malloc(UART_READ_BUF_SIZE + 1);
    assert(dtmp);
    
    while (1)
    {
        // 只处理UART模式
        dataLen = uart_read_bytes(USERUART_PORT, dtmp, UART_READ_BUF_SIZE, 0);
        
        if (dataLen > 0) {
            // CRC16需要至少3个字节（1字节数据 + 2字节CRC）
            if (dataLen < 3) {
                ESP_LOGW(TAG, "用户UART数据太短，无法进行CRC16校验：%d 字节", dataLen);
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
                ESP_LOGW(TAG, "用户UART CRC16校验错误：计算值=0x%04X，接收值=0x%04X", 
                         calculated_crc, received_crc);
            }
        } 
        // 没有数据可用，让出CPU
        vTaskDelay(1);
    }
}
/**
 * @brief 解析GTA通信数据（状态机实现）
 * @param data 接收到的数据缓冲区指针
 * @param len 接收到的数据长度
 */
static void gtaParseData(const uint8_t *data, size_t len)
{
    if (!data || len == 0) {
        ESP_LOGW(TAG, "无效的GTA数据：空指针或长度为零");
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

// ==================== GAT数据链表实现 ====================

/**
 * @brief 初始化GAT数据链表
 * @return 返回初始化后的链表指针，失败返回NULL
 */
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

/**
 * @brief 创建新的GAT数据节点
 * @param cmd 命令类型
 * @param data_len 期望的数据长度
 * @return 返回新节点的指针，失败返回NULL
 */
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

/**
 * @brief 向链表添加节点
 * @param list 链表指针
 * @param node 要添加的节点指针
 * @return 成功返回0，失败返回-1
 */
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

/**
 * @brief 向节点追加数据
 * @param node 目标节点指针
 * @param data 要追加的数据缓冲区
 * @param len 要追加的数据长度
 * @return 返回实际追加的字节数
 */
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

/**
 * @brief 检查节点数据是否接收完整
 * @param node 节点指针
 * @return 完整返回true，否则返回false
 */
bool gatDataNodeIsComplete(gat_data_node_t *node)
{
    if (!node) {
        return false;
    }
    return (node->received_len >= node->data_len);
}

/**
 * @brief 从链表中移除节点并释放内存
 * @param list 链表指针
 * @param node 要移除的节点指针
 * @return 成功返回0，失败返回-1
 */
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

/**
 * @brief 获取链表中第一个未完成的节点
 * @param list 链表指针
 * @return 返回未完成节点指针，如果没有则返回NULL
 */
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

/**
 * @brief 获取链表中第一个已完成的节点
 * @param list 链表指针
 * @return 返回已完成节点指针，如果没有则返回NULL
 */
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

/**
 * @brief 释放节点内存
 * @param node 要释放的节点指针
 */
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

/**
 * @brief 释放整个链表及其所有节点
 * @param list 要释放的链表指针
 */
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

/**
 * @brief 获取链表中节点的数量
 * @param list 链表指针
 * @return 返回节点数量
 */
uint32_t gatDataListGetCount(gat_data_list_t *list)
{
    if (!list) {
        return 0;
    }
    return list->count;
}

/**
 * @brief GTA UART任务（轮询模式接收数据）
 * @param pvParameters 任务参数（未使用）
 */
static void gtaUartTask(void *pvParameters)
{
    uint8_t* dtmp = (uint8_t*) malloc(UART_READ_BUF_SIZE + 1);
    assert(dtmp);
    while (1)
    {
        // 轮询模式：读取可用数据（非阻塞）
        // 最多读取UART_READ_BUF_SIZE字节，如果数据不足则立即返回
        int len = uart_read_bytes(GTA_UART_PORT, dtmp, UART_READ_BUF_SIZE, 0);
        if (len > 0) {
            gtaParseData(dtmp, len);
        } else {
            // 没有数据可用，让出CPU
            vTaskDelay(1);
        }
    }
}

// I2C从机接收完成回调函数
static bool i2cSlaveReceiveDoneCb(i2c_slave_dev_handle_t i2c_slave, const i2c_slave_rx_done_event_data_t *evt_data, void *arg)
{
    QueueHandle_t receive_queue = (QueueHandle_t)arg;  // 修复：使用arg而不是user_data
    i2c_slave_event_t evt = I2C_SLAVE_EVT_RX;
    BaseType_t xTaskWoken = 0;
    // 发送接收事件到队列
    //ESP_LOGI(TAG, "I2C从机接收数据成功，长度: %d", evt_data->rx_data_len);
    memcpy(i2cSlaveData, evt_data->buffer, evt_data->length);
    i2cSlaveDataLen = evt_data->length;
    xQueueSendFromISR(receive_queue, &evt, &xTaskWoken);
    return xTaskWoken;
}

// I2C从机请求完成回调函数（用于发送数据）
static bool i2cSlaveRequestDoneCb(i2c_slave_dev_handle_t i2c_slave, const i2c_slave_request_event_data_t *evt_data, void *arg)
{
    QueueHandle_t receive_queue = (QueueHandle_t)arg;  // 修复：使用arg而不是user_data
    i2c_slave_event_t evt = I2C_SLAVE_EVT_TX;
    BaseType_t xTaskWoken = 0;
    // 发送请求事件到队列
    xQueueSendFromISR(receive_queue, &evt, &xTaskWoken);
    return xTaskWoken;
}


/**
 * @brief 初始化外设（RGB灯、UART、I2C等）
 * @return 成功返回0，失败返回-1
 */
int peripheralInit(void)
{
    // 初始化RGB灯
    led_strip_install();
    int ret = led_strip_init(&strip);
    
    // 初始化GAT数据链表
    gatDataList = gatDataListInit();
    if (!gatDataList) {
        ESP_LOGE(TAG, "初始化GAT数据链表失败");
        return -1;
    }
    ESP_LOGI(TAG, "GAT数据链表初始化成功");
    uart_config_t config = {
        .baud_rate = UART_BAUDRATE,
        .data_bits = UART_DATA_BITS,
        .stop_bits = UART_STOP_BITS,
        .parity = UART_PARITY,
        .flow_ctrl = UART_FLOW_CTRL,
        .source_clk = UART_SOURCE_CLK,
    };
    // 初始化GTA UART
    uart_driver_install(GTA_UART_PORT, UART_BUF_SIZE * 2, UART_BUF_SIZE * 2, 0, NULL, 0);
    uart_param_config(GTA_UART_PORT, &config);
    uart_set_pin(GTA_UART_PORT, GTA_UART_TX_GPIO, GTA_UART_RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    xTaskCreate(gtaUartTask, "gtaUartTask", 8*1024, NULL, 10, NULL);
    // 配置GPIO为输入模式
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,      // 不使用中断
        .mode = GPIO_MODE_INPUT,             // 输入模式
        .pin_bit_mask = (1ULL << MODE_GPIO)|(1ULL << GAT_VDD_EN_GPIO)|(1ULL << GAT_RESET_GPIO)|(1ULL << GAT_SPI0_DO_GPIO),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE     // 启动内部上拉（适合按键）
    };
    gpio_config(&io_conf);

    commMode = gpio_get_level(MODE_GPIO);
    if (commMode == I2C_MODE) {
        i2c_slave_config_t i2c_slv_config = {
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .i2c_port = I2C_SLAVE_NUM,
            .send_buf_depth = 100,
            .receive_buf_depth = 100,
            .scl_io_num = USERUART_RX_SCL_GPIO,
            .sda_io_num = USERUART_TX_SDA_GPIO,
            .slave_addr = USERI2C_ADDRESS,
            .flags.enable_internal_pullup = true,
        };
        
        // i2c_new_slave_device的第一个参数可能是端口号，而不是配置结构
        esp_err_t err = i2c_new_slave_device(&i2c_slv_config, &slave_handle);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "创建I2C从机设备失败: %s", esp_err_to_name(err));
            return -1;
        }
        slaveReceiveQueue = xQueueCreate(10, sizeof(i2c_slave_event_t));
        i2cDataQueue = xQueueCreate(10, sizeof(i2c_received_data_t));
        if (slaveReceiveQueue == NULL || i2cDataQueue == NULL) {
            ESP_LOGE(TAG, "创建I2C队列失败");
            return -1;
        }
        i2c_slave_event_callbacks_t callbacks = {
            .on_receive = i2cSlaveReceiveDoneCb,
            .on_request = i2cSlaveRequestDoneCb,
        };
        ESP_ERROR_CHECK(i2c_slave_register_event_callbacks(slave_handle, &callbacks, slaveReceiveQueue));
        // 配置I2C从机引脚（如果API支持）
        // 注意：ESP-IDF 5.3.4的I2C从机API可能需要单独配置引脚
        ESP_LOGI(TAG, "I2C从机设备初始化成功，地址: 0x%02X", USERI2C_ADDRESS);
        // 创建I2C从机任务（处理TX/RX事件）
        xTaskCreate(i2cSlaveTask, "i2cSlaveTask", 8*1024, NULL, 6, NULL);
        // 创建I2C数据处理任务（处理接收到的数据）
        xTaskCreate(i2cDataProcessTask, "i2cDataProcessTask", 8*1024, NULL, 5, NULL);
    } else {
        uart_driver_install(USERUART_PORT, UART_BUF_SIZE * 2, UART_BUF_SIZE * 2, 0, NULL, 0);
        uart_param_config(USERUART_PORT, &config);
        uart_set_pin(USERUART_PORT, USERUART_TX_SDA_GPIO, USERUART_RX_SCL_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
        ESP_LOGI(TAG, "USERUART_PORT is initialized");
        xTaskCreate(userUartTask, "userUartTask", 8*1024, NULL, 5, NULL);
    }
    
    return ret;
}

/**
 * @brief 设置RGB灯颜色
 * @param color RGB颜色值
 * @return 成功返回0，失败返回非0
 */
int setRGBColor(rgb_t color)
{
    led_strip_set_pixel(&strip, 0, color);
    return led_strip_flush(&strip);
}


