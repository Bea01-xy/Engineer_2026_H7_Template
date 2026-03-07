/**
  ******************************************************************************
  * @file           : CAN_Task.c
  * @brief          : CAN task
  * @author         : GrassFam Wang
  * @date           : 2025/1/22
  * @version        : v1.1
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "CAN_Task.h"
#include "Control_Task.h"
#include "INS_Task.h"
#include "Motor.h"
#include "bsp_can.h"
#include "bsp_uart.h"
#include "Remote_Control.h"
#include "Control_Task.h"
#include "Motor_drv.h"
#include "fdcan.h"
#include "Minipc.h"
#include "Bmi088.h"
#include "Chassis_Config.h"
#include "PID.h"
/* USER CODE BEGIN Header_CAN_Task */
/**
* @brief Function implementing the StartCANTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CAN_Task */
void DM6006_Handler(bool activated);
void M3508_Handler(void);

extern Chassis_Info_Typedef chassis_info;

void CAN_Task(void)
{
    TickType_t CAN_Task_SysTick = 0;
	DM_Motor_Command(&FDCAN1_TxFrame,&DM_6006_Motor[LF],Motor_Enable);
    osDelay(30);
	DM_Motor_Command(&FDCAN1_TxFrame,&DM_6006_Motor[LB],Motor_Enable);
    osDelay(30);
    DM_Motor_Command(&FDCAN1_TxFrame,&DM_6006_Motor[RB],Motor_Enable);
    osDelay(30);
	DM_Motor_Command(&FDCAN1_TxFrame,&DM_6006_Motor[RF],Motor_Enable);
    osDelay(30);
	for(;;)
    {
		CAN_Task_SysTick = osKernelSysTick();
	    if(CAN_Task_SysTick % 2 == 0){
	    }
        DM6006_Handler(chassis_info.activated_flag);
        M3508_Handler();
        osDelay(1);
    }
}

void DM6006_Handler(const bool activated)
{
    if (activated) {
		DM_Motor_CAN_TxMessage(&FDCAN1_TxFrame,&DM_6006_Motor[LF],2,0,0,0.3,0.1);
    }
    else {
	    DM_Motor_Command(&FDCAN1_TxFrame,&DM_6006_Motor[LF],Motor_Disable);
    }
}

void M3508_Handler(void)
{
    M3508_motor_crt_ctrl(&hfdcan2, 0x200, M3508_Motor[LF].Data.Final_Output,
        M3508_Motor[LB].Data.Final_Output,M3508_Motor[RB].Data.Final_Output,M3508_Motor[RF].Data.Final_Output);
}
