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
void Elevator_set(bool activated);
void M3508_set(void);

extern Chassis_Info_Typedef chassis_info;

void CAN_Task(void)
{
	DM_Motor_Command(&FDCAN1_TxFrame,&Elevator_Motor[LF],Motor_Save_Zero_Position);
	DM_Motor_Command(&FDCAN1_TxFrame,&Elevator_Motor[LB],Motor_Save_Zero_Position);
    osDelay(1);
	DM_Motor_Command(&FDCAN1_TxFrame,&Elevator_Motor[RB],Motor_Save_Zero_Position);
	DM_Motor_Command(&FDCAN1_TxFrame,&Elevator_Motor[RF],Motor_Save_Zero_Position);
    TickType_t CAN_Task_SysTick = 0;
	for(;;)
    {
		CAN_Task_SysTick = osKernelSysTick();
	    if(CAN_Task_SysTick % 2 == 0){
	    }
        Elevator_set(chassis_info.activated_flag);
        M3508_set();
        osDelay(1);
    }
}

void Elevator_set(const bool activated)
{
    if (activated) {
        DM_Motor_Command(&FDCAN1_TxFrame,&Elevator_Motor[LF],Motor_Enable);
	    DM_Motor_Command(&FDCAN1_TxFrame,&Elevator_Motor[LB],Motor_Enable);
        osDelay(1);
        DM_Motor_Command(&FDCAN1_TxFrame,&Elevator_Motor[RB],Motor_Enable);
	    DM_Motor_Command(&FDCAN1_TxFrame,&Elevator_Motor[RF],Motor_Enable);

        DM_Motor_CAN_TxMessage(&FDCAN1_TxFrame,&Elevator_Motor[LF],Elevator_Motor[LF].Data.Temp_Target_Position, MIT_NO_USE,ELEVATOR_KP,MIT_NO_USE,Elevator_Motor[LF].Data.Feedforward);
        DM_Motor_CAN_TxMessage(&FDCAN1_TxFrame,&Elevator_Motor[LB],Elevator_Motor[LB].Data.Temp_Target_Position, MIT_NO_USE,ELEVATOR_KP,MIT_NO_USE,Elevator_Motor[LB].Data.Feedforward);
        osDelay(1);
        DM_Motor_CAN_TxMessage(&FDCAN1_TxFrame,&Elevator_Motor[RB],Elevator_Motor[RB].Data.Temp_Target_Position, MIT_NO_USE,ELEVATOR_KP,MIT_NO_USE,Elevator_Motor[RB].Data.Feedforward);
        DM_Motor_CAN_TxMessage(&FDCAN1_TxFrame,&Elevator_Motor[RF],Elevator_Motor[RF].Data.Temp_Target_Position, MIT_NO_USE,ELEVATOR_KP,MIT_NO_USE,Elevator_Motor[RF].Data.Feedforward);

        //new motor to control
        DM_Motor_Command(&FDCAN3_TxFrame,&Robotic_Arm_Motor[0],Motor_Enable);
        DM_Motor_CAN_TxMessage(&FDCAN3_TxFrame,&Robotic_Arm_Motor[0],MIT_NO_USE,
            0.4,MIT_NO_USE,1,0);
    }
    else {
	    DM_Motor_Command(&FDCAN1_TxFrame,&Elevator_Motor[LF],Motor_Disable);
	    DM_Motor_Command(&FDCAN1_TxFrame,&Elevator_Motor[LB],Motor_Disable);
        osDelay(1);
        DM_Motor_Command(&FDCAN1_TxFrame,&Elevator_Motor[RB],Motor_Disable);
	    DM_Motor_Command(&FDCAN1_TxFrame,&Elevator_Motor[RF],Motor_Disable);

        DM_Motor_Command(&FDCAN3_TxFrame,&Robotic_Arm_Motor[0],Motor_Disable);
    }
}

void M3508_set(void)
{
    M3508_motor_crt_ctrl(&hfdcan2, 0x200, Chassis_Motor[LF].Data.Final_Output,
        Chassis_Motor[LB].Data.Final_Output,Chassis_Motor[RB].Data.Final_Output,Chassis_Motor[RF].Data.Final_Output);

    M2006_motor_crt_ctrl(&hfdcan3, 0x200, 3000,
        0, 0,0);
}
