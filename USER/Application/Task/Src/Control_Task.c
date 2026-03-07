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

static void Control_Init(void);
static void Control_Info_Update(void);

Chassis_Info_Typedef chassis_info;

static float M3508_PID_Param[PID_PARAMETER_NUM] = {M3508_KP, M3508_KI, M3508_KD, 0.5f, 2.f, 2000.f, 15000.f};
PID_Info_TypeDef M3508_PID[4];

void Control_Task(void)
{
    /* USER CODE BEGIN Control_Task */
    TickType_t Control_Task_SysTick = 0;
 
	Control_Init();
    /* Infinite loop */
	for(;;)
    {
		Control_Task_SysTick = osKernelSysTick();
        if (Control_Task_SysTick % 500 == 0) {
        	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);
        }

		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);

        Control_Info_Update();
		
		osDelay(1);
    }
}
  /* USER CODE END Control_Task */

static void Control_Init(void)
{
    PID_Init(&M3508_PID[0],PID_POSITION,M3508_PID_Param);
    PID_Init(&M3508_PID[1],PID_POSITION,M3508_PID_Param);
    PID_Init(&M3508_PID[2],PID_POSITION,M3508_PID_Param);
    PID_Init(&M3508_PID[3],PID_POSITION,M3508_PID_Param);
}

static void Control_Info_Update(void)
{
    PID_Calculate(&M3508_PID[0], Chassis_Motor[0].Data.Target_Velocity, Chassis_Motor[0].Data.Velocity);
    PID_Calculate(&M3508_PID[1], Chassis_Motor[1].Data.Target_Velocity, Chassis_Motor[1].Data.Velocity);
    PID_Calculate(&M3508_PID[2], Chassis_Motor[2].Data.Target_Velocity, Chassis_Motor[2].Data.Velocity);
    PID_Calculate(&M3508_PID[3], Chassis_Motor[3].Data.Target_Velocity, Chassis_Motor[3].Data.Velocity);
}

static float SmootherStep(float NowTime,float UseTime)
{
	float Time = (NowTime/UseTime);

    return 10*powf(Time,3) - 15*powf(Time,4) + 6*powf(Time,5);
}

