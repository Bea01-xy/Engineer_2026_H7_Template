/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Servo.h
  * @brief          : 总线舵机驱动 (UART)
  * @author         : GrassFan Wang
  * @date           : 2025/04/27
  * @version        : v1.0
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef DEVICE_SERVO_H
#define DEVICE_SERVO_H

#include "stdint.h"
#include "stdbool.h"

/* 位置范围 0~4095 (对应 -180°~180°) */
#define SERVO_POS_MIN       0
#define SERVO_POS_MAX       4095
#define SERVO_POS_MID       2048

/* ID 范围 */
#define SERVO_ID_MIN        1
#define SERVO_ID_MAX        250
#define SERVO_ID_BROADCAST  254

/**
  * @brief  设置舵机目标位置
  * @param  id       舵机 ID (1~250)
  * @param  position 目标位置 (0~4095)
  * @param  run_time 运行时间 (ms), 0=最快速度
  * @retval None
  */
void Servo_SetPosition(uint8_t id, uint16_t position, uint16_t run_time);

/**
  * @brief  使能/关闭舵机扭矩输出
  * @param  id     舵机 ID
  * @param  enable true=扭矩开, false=扭矩关
  */
void Servo_SetTorque(uint8_t id, bool enable);

/**
  * @brief  设置舵机模式
  * @param  id   舵机 ID
  * @param  mode 0x01=舵机模式, 0x00=电机模式
  */
void Servo_SetMode(uint8_t id, uint8_t mode);

/**
  * @brief  读取当前位置 (异步, 结果通过 UART RX 返回)
  * @param  id 舵机 ID
  */
void Servo_ReadPosition(uint8_t id);

/**
  * @brief  查询舵机是否正在 DMA 发送
  * @retval true=忙, false=空闲
  */
bool Servo_IsBusy(void);

/**
  * @brief  DMA 发送完成回调 (由 HAL_UART_TxCpltCallback 调用)
  */
void Servo_TxCpltCallback(void);

#endif /* DEVICE_SERVO_H */
