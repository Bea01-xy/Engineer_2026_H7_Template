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
#include "math.h"

/* Exported constants --------------------------------------------------------*/

/**
  * @brief Robotic arm basic parameters
  *        机械臂基础参数
  */
#define ROBOTIC_ARM_DOF                     6U      /* 自由度数量，后续按实际机械臂修改 */

/* 单位：度 */
#define ROBOTIC_ARM_JOINT_MIN_ANGLE_DEG     (-180.0f)
#define ROBOTIC_ARM_JOINT_MAX_ANGLE_DEG     (180.0f)

/* 单位：deg/s */
#define ROBOTIC_ARM_MAX_JOINT_VEL_DEG_S     (180.0f)

/* 单位：deg/s^2 */
#define ROBOTIC_ARM_MAX_JOINT_ACC_DEG_S2    (360.0f)

/* 位置 PID 参数（示例值，按实际调参） */
#define ROBOTIC_ARM_POS_KP                  10.0f
#define ROBOTIC_ARM_POS_KI                  0.0f
#define ROBOTIC_ARM_POS_KD                  0.2f

/* 电流/力矩输出限制 */
#define ROBOTIC_ARM_OUTPUT_LIMIT            15000.0f
#define ROBOTIC_ARM_I_OUT_LIMIT             500.0f

/* 死区、积分限幅等通用参数 */
#define ROBOTIC_ARM_DEADBAND_DEG            0.5f
#define ROBOTIC_ARM_LIMIT_INTEGRAL          2000.0f
#define ROBOTIC_ARM_LIMIT_OUTPUT            ROBOTIC_ARM_OUTPUT_LIMIT

/* Remote control / command mapping (示例，按实际通道与比例调整) */
#define MAX_ARM_CMD_VALUE                   660.0f

#define CMD_TO_JOINT_VEL                    (ROBOTIC_ARM_MAX_JOINT_VEL_DEG_S / MAX_ARM_CMD_VALUE)

#define J1 0
#define J2 1
#define J3 2
#define J4 3
#define J5 4
#define J6 5

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
/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
  * @brief Robotic arm work mode
  *        机械臂工作模式
  */
typedef enum
{
    ROBOTIC_ARM_DISABLE = 0U,      /* 关闭 / 失能 */
    ROBOTIC_ARM_MANUAL,            /* 手动控制（遥控器/上位机直接发关节指令） */
    ROBOTIC_ARM_AUTO_TRAJ,         /* 按既定轨迹自动运行 */
    ROBOTIC_ARM_HOLD,              /* 保持当前位置 */
} Robotic_Arm_Mode_e;

/**
  * @brief Robotic arm state info
  *        机械臂状态信息
  */
typedef struct
{
    Robotic_Arm_Mode_e mode;
    Robotic_Arm_Mode_e last_mode;

    /* 关节当前角度、目标角度（单位：deg） */
    float joint_angle_deg[ROBOTIC_ARM_DOF];
    float joint_target_angle_deg[ROBOTIC_ARM_DOF];

    /* 关节当前/目标速度（单位：deg/s） */
    float joint_vel_deg_s[ROBOTIC_ARM_DOF];
    float joint_target_vel_deg_s[ROBOTIC_ARM_DOF];

    /* 是否已经初始化到零位/标定位置 */
    bool  homed_flag;

    /* 整体激活标志：允许输出力矩/电流 */
    bool  activated_flag;
} Robotic_Arm_Info_Typedef;

#endif /* ROBOTIC_ARM_CONFIG_H */

