#ifndef USER_CONFIG_H
#define USER_CONFIG_H

#define ES7210
//#define ES7243E

#define ESP_MN_CH_MODEL 1 //使能中文自定义唤醒词
#define ESP_MN_EN_MODEL 0 //使能英文自定义唤醒词

#define RGB_GPIO                        9 //RGB指示灯引脚
#define MODE_GPIO                       21 //通信模式选择按键引脚

#define GAT_WAKEUP_INT_GPIO             47 //GAT唤醒引脚
#define GAT_VDD_EN_GPIO                 7 //GAT电源使能引脚
#define GAT_RESET_GPIO                  45 //GAT复位引脚
#define GAT_SPI0_DO_GPIO                12 //GATSPI0数据输出引脚
#define GAT_WAKEUP_GPIO                 8 

#define USERUART_PORT                   UART_NUM_2
#define USERUART_TX_SDA_GPIO            14
#define USERUART_RX_SCL_GPIO            13

#define GTA_UART_PORT                   UART_NUM_1
#define GTA_UART_TX_GPIO                38
#define GTA_UART_RX_GPIO                39

#define UART_BAUDRATE                   9600
#define UART_BUF_SIZE                   1024

#define USERI2C_ADDRESS                 0x1f
#define I2C_SLAVE_NUM                   I2C_NUM_1



#endif
