// 临时空文件 - HEAD~1版本不需要此功能
#include "Power_Limit_communication.h"
#include <stdbool.h>
#include <string.h>
#include "usart.h"

// 全局变量定义
uint8_t uart7_Power[RX_BUFFER_SIZE] = {0};
volatile rx_PowerLimit_t rx_PowerLimit = {0};
volatile bool rx_power_valid = false;
uint8_t test122 = sizeof(rx_PowerLimit);

void PowerLimit_USART7_IRQHandler(void)
{
    // 空实现
}

bool PowerLimit_read_data(uint8_t *buffer, uint16_t len)
{
    return false;
}

void PowerLimit_init(void)
{
    // 空实现
}
