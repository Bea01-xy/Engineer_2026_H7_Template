/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Servo.c
  * @brief          : 总线舵机驱动 (UART) - 协议 v2.0.2
  * @author         : GrassFan Wang
  * @date           : 2025/04/27
  * @version        : v1.0
  ******************************************************************************
  */
/* USER CODE END Header */

#include "Servo.h"
#include "usart.h"              /* huart7 */

/* ---------- 协议常量 ---------- */
#define FRAME_HEADER0           0xFF
#define FRAME_HEADER1           0xFF
#define CMD_PING                0x01
#define CMD_READ                0x02
#define CMD_WRITE               0x03

#define ADDR_TORQUE             0x28
#define ADDR_TARGET_POS         0x2A
#define ADDR_CURRENT_POS        0x38
#define ADDR_MODE               0x1C

#define SERVO_TX_BUF_SIZE       32

/* ---------- 本地变量 ---------- */
__attribute__((section(".AXI_SRAM"))) static uint8_t  servo_tx_buf[SERVO_TX_BUF_SIZE];
static volatile bool servo_busy = false;

/* ---------- 静态函数 ---------- */

/**
  * @brief  单字节和校验 (大端)
  *         checksum = ~(ID + data_len + cmd + params...) & 0xFF
  */
static uint8_t SERVO_Checksum(uint8_t *data, uint8_t len)
{
    uint16_t sum = 0;
    for (uint8_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return (~(uint8_t)sum) & 0xFF;
}

/**
  * @brief  构造并发送一帧指令
  * @param  id        舵机 ID
  * @param  cmd       指令类型 (CMD_WRITE / CMD_READ / ...)
  * @param  params    参数缓冲区指针
  * @param  param_len 参数长度 (字节)
  */
static void SERVO_SendFrame(uint8_t id, uint8_t cmd, uint8_t *params, uint8_t param_len)
{
    if (servo_busy) {
        return;     /* DMA 正在发送, 丢弃本次指令 */
    }

    /* 数据长度 = ID(1) + cmd(1) + param(N) */
    uint8_t data_len = 2 + param_len;
    uint8_t idx = 0;

    servo_tx_buf[idx++] = FRAME_HEADER0;
    servo_tx_buf[idx++] = FRAME_HEADER1;
    servo_tx_buf[idx++] = id;
    servo_tx_buf[idx++] = data_len;
    servo_tx_buf[idx++] = cmd;

    for (uint8_t i = 0; i < param_len; i++) {
        servo_tx_buf[idx++] = params[i];
    }

    /* 校验和: ~(ID + data_len + cmd + params...) & 0xFF */
    servo_tx_buf[idx] = SERVO_Checksum(&servo_tx_buf[2], 1 + 1 + param_len); /* ID+len+cmd+params */

    servo_busy = true;
    HAL_UART_Transmit_DMA(&huart7, servo_tx_buf, idx + 1);
}

/* ============================ 公有 API ============================ */

/**
  * @brief  设置舵机目标位置
  *         指令: FF FF ID 07 03 2A posH posL timeH timeL CS
  */
void Servo_SetPosition(uint8_t id, uint16_t position, uint16_t run_time)
{
    /* ponytail: 钳位到有效范围, 不做上层校验 */
    if (position > SERVO_POS_MAX) position = SERVO_POS_MAX;

    uint8_t params[5] = {
        ADDR_TARGET_POS,
        (uint8_t)(position >> 8),
        (uint8_t)(position & 0xFF),
        (uint8_t)(run_time >> 8),
        (uint8_t)(run_time & 0xFF)
    };
    SERVO_SendFrame(id, CMD_WRITE, params, 5);
}

/**
  * @brief  使能 / 关闭扭矩
  */
void Servo_SetTorque(uint8_t id, bool enable)
{
    uint8_t params[2] = {ADDR_TORQUE, enable ? 0x01 : 0x00};
    SERVO_SendFrame(id, CMD_WRITE, params, 2);
}

/**
  * @brief  设置舵机模式
  */
void Servo_SetMode(uint8_t id, uint8_t mode)
{
    uint8_t params[2] = {ADDR_MODE, mode};
    SERVO_SendFrame(id, CMD_WRITE, params, 2);
}

/**
  * @brief  读取当前位置 (异步)
  */
void Servo_ReadPosition(uint8_t id)
{
    uint8_t params[2] = {ADDR_CURRENT_POS, 0x02};  /* 地址 0x38, 读 2 字节 */
    SERVO_SendFrame(id, CMD_READ, params, 2);
}

bool Servo_IsBusy(void)
{
    return servo_busy;
}

/**
  * @brief  DMA 发送完成回调
  *         在 bsp_uart.c 的 HAL_UART_TxCpltCallback 中调用:
  *         if (huart == &huart7) Servo_TxCpltCallback();
  */
void Servo_TxCpltCallback(void)
{
    servo_busy = false;
}
