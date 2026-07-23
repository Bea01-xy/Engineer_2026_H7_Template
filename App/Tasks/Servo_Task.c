/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Servo_Task.c
  * @brief          : Bus servo control task — sinusoidal motion
  * @note           : Drives servo #1 in a smooth sine wave pattern.
  *                   Period 4 s, amplitude ±135° (~1536 LSB).
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "Servo.h"
#include "Servo_Task.h"
#include <math.h>
#include "Robotic_Arm_Config.h"
#include "Motor_DM.h"
#include "bsp_uart.h"
#include "Config.h"
/* ========================= 全局共享状态 ========================= */

volatile ServoDisplayState servo_display = {0};

/* ========================= 应答回调 ========================= */

static void Servo_StateCallback(const Servo_ResponsePacket *resp)
{
    servo_display.resp_id        = resp->id;
    servo_display.resp_status    = resp->status;
    servo_display.last_resp_tick = osKernelSysTick();
    servo_display.has_response   = true;

    /* ReadPosition 应答: params[0..1] = 位置值 (大端) */
    if (resp->param_len == 2) {
        servo_display.resp_position =
            (uint16_t)(resp->params[0] << 8) | resp->params[1];
    }
}

/* ========================= 主任务 ========================= */

void Servo_Task(void const *argument)
{
    (void)argument;

    /* 注册应答回调 */
    Servo_SetResponseCallback(Servo_StateCallback);

    /* ---- 舵机初始化 ---- */
    Servo_SetMode(1, 0x01);
    osDelay(20);
    Servo_SetTorque(1, 1);
    osDelay(20);
    Servo_SetMode(2, 0x01);
    osDelay(20);
    Servo_SetTorque(2, 1);
    osDelay(20);

    for (;;)
    {
        float target_yaw_angle   = (- Robotic_Arm_Motor[J1].Data.Joint_Position) * 180.0f / 3.1415926f;
        float target_pitch_angle = (- Robotic_Arm_Motor[J2].Data.Joint_Position - Robotic_Arm_Motor[J3].Data.Joint_Position) * 180.0f / 3.1415926f;
        //USART10_Vofa_SendFloat(&target_pitch_angle, 1);
        VAL_LIMIT(target_pitch_angle, SERVO_PITCH_MIN, SERVO_PITCH_MAX);
        Servo_SetPosition(SERVO_YAW, (uint16_t)SERVO_ANGLE_TO_POS(target_yaw_angle + SERVO_YAW_OFFSET), 10);
        Servo_SetPosition(SERVO_PITCH, (uint16_t)SERVO_ANGLE_TO_POS(target_pitch_angle + SERVO_PITCH_OFFSET), 10);
        osDelay(5);
    }
}
