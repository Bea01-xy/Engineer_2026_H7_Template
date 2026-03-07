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

extern float cascade_pid_output;
extern PID_Info_TypeDef M3508_PID[4];
void CAN_Task(void)
{
    TickType_t CAN_Task_SysTick = 0;
	DM_Motor_Command(&FDCAN2_TxFrame,&DM_8009_Motor[0],Motor_Enable);
    osDelay(30);
	DM_Motor_Command(&FDCAN2_TxFrame,&DM_8009_Motor[1],Motor_Enable);
    osDelay(30);
    DM_Motor_Command(&FDCAN3_TxFrame,&DM_8009_Motor[2],Motor_Enable);
    osDelay(30);
	DM_Motor_Command(&FDCAN3_TxFrame,&DM_8009_Motor[3],Motor_Enable);
    osDelay(30);
	for(;;)
    {
		CAN_Task_SysTick = osKernelSysTick();

		DM_Motor_CAN_TxMessage(&FDCAN2_TxFrame,&DM_8009_Motor[0],2,0,0,0.3,0.1);

        //M3508_motor_crt_ctrl(&hfdcan2, 0x200, M3508_PID[LF].Output, M3508_PID[LB].Output,
                  														  //M3508_PID[RB].Output, M3508_PID[RF].Output);
        M3508_motor_crt_ctrl(&hfdcan2, 0x200, M3508_Motor[LF].Data.Final_Output,
        M3508_Motor[LB].Data.Final_Output,M3508_Motor[RB].Data.Final_Output,M3508_Motor[RF].Data.Final_Output);
	    if(CAN_Task_SysTick % 2 == 0){


	    }
		osDelay(1);
    }
}

