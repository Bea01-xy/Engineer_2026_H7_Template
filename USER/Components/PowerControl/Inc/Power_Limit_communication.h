// 临时空文件 - HEAD~1版本不需要此功能
#ifndef POWER_LIMIT_COMMUNICATION_H
#define POWER_LIMIT_COMMUNICATION_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32h7xx_hal.h"

#define RX_BUFFER_SIZE      64
#define RX_FRAME_TOTAL_SIZE 16
#define RX_FRAME_START_BYTE 0xAA
#define RX_FRAME_END_BYTE   0x55

typedef struct {
    float voltage;
    float current;
    float power;
    float energy;
} rx_PowerLimit_t;

extern uint8_t uart7_Power[RX_BUFFER_SIZE];
extern volatile rx_PowerLimit_t rx_PowerLimit;
extern volatile bool rx_power_valid;
extern uint8_t test122;

void PowerLimit_USART7_IRQHandler(void);
bool PowerLimit_read_data(uint8_t *buffer, uint16_t len);
void PowerLimit_init(void);

#endif // POWER_LIMIT_COMMUNICATION_H
