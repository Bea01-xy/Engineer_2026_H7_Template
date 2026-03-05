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

static void Control_Init(Control_Info_Typedef *Control_Info);
static void Control_Measure_Update(Control_Info_Typedef *Control_Info);
static void Control_Target_Update(Control_Info_Typedef *Control_Info);
static void Control_Info_Update(Control_Info_Typedef *Control_Info);

Control_Info_Typedef Control_Info;
//                                   KP   KI   KD  Alpha Deadband  I_MAX   Output_MAX
static float Chassis_PID_Param[7] = {13.f,0.1f,0.f,0.f,  0.f,      5000.f,  12000.f};

/* PID instances */
PID_Info_TypeDef angle_outer_pid;
PID_Info_TypeDef speed_inner_pid;
/* PID Parameters: [KP, KI, KD, Alpha, Deadband, LimitIntegral, LimitOutput] */
// Note: Alpha is for D-term LPF
float angle_params[PID_PARAMETER_NUM] = {12.5f, 0.2f, 0.01f, 0.8f, 0.1f, 500.0f, 1000.0f}; 
float speed_params[PID_PARAMETER_NUM] = {12.2f, 0.1f, 0.10f, 0.0f, 0.0f, 500.0f, 9000.0f};

PID_Info_TypeDef Chassis_PID;

void Control_Task(void)
{
    /* USER CODE BEGIN Control_Task */
    TickType_t Control_Task_SysTick = 0;
 
	Control_Init(&Control_Info);
    /* Infinite loop */
	for(;;)
    {
		Control_Task_SysTick = osKernelSysTick();

		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);
	    Control_Measure_Update(&Control_Info);
		Control_Target_Update(&Control_Info);
        Control_Info_Update(&Control_Info);
        //USART_Vofa_Justfloat_Transmit(Control_Info.Measure.Chassis_Velocity,0.f,0.f);
		
		osDelay(1);
    }
}
  /* USER CODE END Control_Task */

static void Control_Init(Control_Info_Typedef *Control_Info){

    PID_Init(&Chassis_PID,PID_POSITION,Chassis_PID_Param);

    // Initialize Outer Loop (Position/Angle)
    PID_Init(&angle_outer_pid, PID_POSITION, angle_params);

    // Initialize Inner Loop (Velocity)
    PID_Init(&speed_inner_pid, PID_POSITION, speed_params);
}

static void Control_Measure_Update(Control_Info_Typedef *Control_Info){

	Control_Info->Measure.Chassis_Velocity = Chassis_Motor[0].Data.Velocity;

}

static void Control_Target_Update(Control_Info_Typedef *Control_Info){

    Control_Info->Target.Chassis_Velocity = remote_ctrl.rc.ch[3] * 5.f;

}
float cascade_pid_output;

static void Control_Info_Update(Control_Info_Typedef *Control_Info){

    PID_Calculate(&Chassis_PID, Control_Info->Target.Chassis_Velocity, Control_Info->Measure.Chassis_Velocity);
    Control_Info->SendValue[0] = (int16_t)(Chassis_PID.Output);

    cascade_pid_output = Cascade_PID_Control(&angle_outer_pid, 
                                                   &speed_inner_pid, 
                                                   90.0f, 
                                                   DJI_Yaw_Motor.Data.Angle,
                                                    DJI_Yaw_Motor.Data.Velocity);
}

static float FivePower(float NowTime,float UseTime){

	float Time = (NowTime/UseTime);

    return 10*powf(Time,3) - 15*powf(Time,4) + 6*powf(Time,5);

}

