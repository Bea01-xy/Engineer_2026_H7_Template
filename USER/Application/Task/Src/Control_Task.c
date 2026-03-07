/**
  ******************************************************************************
  * @file           : Control_Task.c
  * @brief          : Control task
  * @author         : GrassFan Wang
  * @date           : 2025/01/22
  * @version        : v1.1
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "Control_Task.h"
#include "cmsis_os.h"
#include "Control_Task.h"
#include "bsp_uart.h"
#include "Remote_Control.h"
#include "PID.h"
#include "Motor.h"
#include "arm_math.h"
#include "Chassis_Config.h"
#include "INS_Task.h"

static void Control_Init(void);

static void chassis_lifting_handler(void);
static void chassis_disabled_handler(void);
static void chassis_only_handler(void);

static void chassis_set(bool acticated);

Chassis_Info_Typedef chassis_info;

static float M3508_PID_Param[PID_PARAMETER_NUM] = {M3508_KP, M3508_KI, M3508_KD, 0.5f, 2.f, 2000.f, 15000.f};
static float Chassis_Direction_PID_Param[PID_PARAMETER_NUM] = {2, 0.05f, 0.3f, 0.2f, 2.f, 1.f, 3.f};

PID_Info_TypeDef M3508_PID[4];
PID_Info_TypeDef Chassis_Direction_PID;

TickType_t Control_Task_SysTick = 0;
void Control_Task(void)
{
    /* USER CODE BEGIN Control_Task */
	Control_Init();
    /* Infinite loop */
	for(;;)
    {
		Control_Task_SysTick = osKernelSysTick();

        switch (chassis_info.mode)
        {
            case CHASSIS_DISABLE:
                chassis_disabled_handler();
                break;
            case CHASSIS_ONLY:
                chassis_only_handler();
                break;
            case CHASSIS_LIFT:
                chassis_lifting_handler();
                break;
            default: break;
        }
	    chassis_set(chassis_info.activated_flag);
		osDelay(1);
    }
}
  /* USER CODE END Control_Task */

static void Control_Init(void)
{
    PID_Init(&M3508_PID[LF],PID_POSITION,M3508_PID_Param);
    PID_Init(&M3508_PID[LB],PID_POSITION,M3508_PID_Param);
    PID_Init(&M3508_PID[RB],PID_POSITION,M3508_PID_Param);
    PID_Init(&M3508_PID[RF],PID_POSITION,M3508_PID_Param);

    PID_Init(&Chassis_Direction_PID,PID_POSITION,Chassis_Direction_PID_Param);

    chassis_info.activated_flag = false;
    chassis_info.mode = CHASSIS_DISABLE;
    chassis_info.last_mode = CHASSIS_DISABLE;
    chassis_info.lift_mode = AUTO_LIFT_STAGE_1;
    chassis_info.target_direction = 0;
}

static float SmootherStep(float NowTime,float UseTime)
{
	float Time = (NowTime/UseTime);

    return 10*powf(Time,3) - 15*powf(Time,4) + 6*powf(Time,5);
}

static void chassis_lifting_handler(void)
{
    if (Control_Task_SysTick % 500 == 0)
    {
        HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
        HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_14);
    }
    chassis_info.activated_flag = true;
}

static void chassis_disabled_handler(void)
{
    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,GPIO_PIN_SET);
    chassis_info.activated_flag = false;
    chassis_info.target_vx = 0.0f;
    chassis_info.target_vy = 0.0f;
    chassis_info.target_vw = 0.0f;
    M3508_Motor[LF].Data.Final_Output = 0u;
    M3508_Motor[LB].Data.Final_Output = 0u;
    M3508_Motor[RB].Data.Final_Output = 0u;
    M3508_Motor[RF].Data.Final_Output = 0u;
}

static void chassis_only_handler(void)
{
    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,GPIO_PIN_RESET);
    chassis_info.activated_flag = true;
}

static void chassis_set(const bool acticated)
{
    if (acticated) {
        PID_Calculate(&M3508_PID[LF], M3508_Motor[LF].Data.Target_Velocity, M3508_Motor[LF].Data.Velocity);
        PID_Calculate(&M3508_PID[LB], M3508_Motor[LB].Data.Target_Velocity, M3508_Motor[LB].Data.Velocity);
        PID_Calculate(&M3508_PID[RB], M3508_Motor[RB].Data.Target_Velocity, M3508_Motor[RB].Data.Velocity);
        PID_Calculate(&M3508_PID[RF], M3508_Motor[RF].Data.Target_Velocity, M3508_Motor[RF].Data.Velocity);

        Single_Angle_PID_Calculate(&Chassis_Direction_PID, chassis_info.target_direction, INS_Info.Yaw_Angle);
        chassis_info.target_vw = Chassis_Direction_PID.Output;

        //just for now
        M3508_Motor[LF].Data.Final_Output = M3508_PID[LF].Output;
        M3508_Motor[LB].Data.Final_Output = M3508_PID[LB].Output;
        M3508_Motor[RB].Data.Final_Output = M3508_PID[RB].Output;
        M3508_Motor[RF].Data.Final_Output = M3508_PID[RF].Output;
    }
}
