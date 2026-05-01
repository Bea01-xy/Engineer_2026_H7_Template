/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Robotic_Arm_Config.h
  * @brief          : Basic configuration for robotic arm
  *                   机械臂基础配置参数定义头文件
  * @author         : Wang Xinyuan
  * @date           : 2026/03/12
  * @version        : v1.0
  ******************************************************************************
  * @attention      : To be perfected
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef ROBOTIC_ARM_CONFIG_H
#define ROBOTIC_ARM_CONFIG_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdbool.h"

/* Exported constants --------------------------------------------------------*/
#define ROBOTIC_ARM_DOF 6u

#define J1 0u
#define J2 1u
#define J3 2u
#define J4 3u
#define J5 4u
#define J6 5u

#define J1_MIN_ANGLE_DEG -180.0f
#define J1_MAX_ANGLE_DEG 180.0f
#define J2_MIN_ANGLE_DEG -180.0f
#define J2_MAX_ANGLE_DEG 180.0f
#define J3_MIN_ANGLE_DEG -180.0f
#define J3_MAX_ANGLE_DEG 180.0f
#define J4_MIN_ANGLE_DEG -180.0f
#define J4_MAX_ANGLE_DEG 180.0f
#define J5_MIN_ANGLE_DEG -180.0f
#define J5_MAX_ANGLE_DEG 180.0f
#define J6_MIN_ANGLE_DEG -180.0f
#define J6_MAX_ANGLE_DEG 180.0f

typedef enum
{
  HAND_OPEN = 0U,
  HAND_CLOSE = 1U,
} Hand_State_e;

#define J1_INITIAL_POS 0.00f
#define J2_INITIAL_POS 2.44f
#define J3_INITIAL_POS -2.44f
#define J4_INITIAL_POS 0.00f
#define J5_INITIAL_POS 0.00f
#define J6_INITIAL_POS 0.00f 

#define ROBOTIC_ARM_MOVING_TIME 1000u //ms

#define GRIPPER_OPEN_POS -21.0f
#define GRIPPER_CLOSE_POS 133.8f

#define GRIPPER_KP 50.5f
#define GRIPPER_KI 0.1f
#define GRIPPER_KD 30.5f
#define GRIPPER_Alpha 0.9f
#define GRIPPER_Deadband 1.5f
#define GRIPPER_LimitIntegral 3000u
#define GRIPPER_LimitOutput 3000u

/* ----------------------------------------------------------------------------
 * 机械臂关节 MIT 前馈项 PID(用于消除关节稳态误差)
 *  - 输入: Target_Position - Position (单位: rad)
 *  - 输出: 直接作为 MIT 模式 Feedforward 力矩 (单位: N·m)
 *  - 注意: J3/J4/J6 电机正转方向与关节正转方向相反, 使用时需对前馈值取反
 *  - 注意: PID.c 中 LimitIntegral 限的是累加误差 Integral 本身, Iout = KI * Integral
 *          所以为了让 Iout 能达到 LimitOutput, 需保证 KI * LimitIntegral >= LimitOutput
 *
 *  分组:
 *      Group A (大关节, 靠近基座, 承重大): J1, J2, J3
 *      Group B (小关节, 末端三轴, 负载小): J4, J5, J6
 * -------------------------------------------------------------------------- */

/* ---------- Group A: J1 / J2 / J3 ---------- */
#define ROBOTIC_ARM_FF_A_KP              0.15f
#define ROBOTIC_ARM_FF_A_KI              0.01f
#define ROBOTIC_ARM_FF_A_KD              1.7f
#define ROBOTIC_ARM_FF_A_Alpha           0.5f
#define ROBOTIC_ARM_FF_A_Deadband        0.02f      /* rad, 约 1.15° */
#define ROBOTIC_ARM_FF_A_LimitIntegral   400.0f     /* 单位: rad·tick (累加误差) */
#define ROBOTIC_ARM_FF_A_LimitOutput     9.0f       /* N·m */

/* ---------- Group B: J4 / J5 / J6 ---------- */
#define ROBOTIC_ARM_FF_B_KP              0.4f
#define ROBOTIC_ARM_FF_B_KI              0.02f
#define ROBOTIC_ARM_FF_B_KD              0.3f
#define ROBOTIC_ARM_FF_B_Alpha           0.5f
#define ROBOTIC_ARM_FF_B_Deadband        0.02f      /* rad, 约 1.15° */
#define ROBOTIC_ARM_FF_B_LimitIntegral   100.0f      /* 单位: rad·tick (累加误差) */
#define ROBOTIC_ARM_FF_B_LimitOutput     1.5f       /* N·m */

#endif /* ROBOTIC_ARM_CONFIG_H */

