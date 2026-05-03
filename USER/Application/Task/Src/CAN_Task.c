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
#include "Robotic_Arm_Config.h"
#include "PID.h"
#include <math.h>
#include <stdint.h>

/* USER CODE BEGIN Header_CAN_Task */
/**
* @brief Function implementing the StartCANTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CAN_Task */
void Elevator_set(bool activated);
void Robotic_Arm_set(int part, bool activated);
void Chassis_set(bool activated);

extern Chassis_Info_Typedef chassis_info;
extern uint8_t hand_state;

void CAN_Task(void)
{
	DM_Motor_Command(&FDCAN1_TxFrame,&Elevator_Motor[LF],Motor_Save_Zero_Position);
	DM_Motor_Command(&FDCAN1_TxFrame,&Elevator_Motor[LB],Motor_Save_Zero_Position);
    osDelay(1);
	DM_Motor_Command(&FDCAN1_TxFrame,&Elevator_Motor[RB],Motor_Save_Zero_Position);
	DM_Motor_Command(&FDCAN1_TxFrame,&Elevator_Motor[RF],Motor_Save_Zero_Position);

	//DM_Motor_Command(&FDCAN3_TxFrame,&Robotic_Arm_Motor[J1],Motor_Save_Zero_Position);
	//DM_Motor_Command(&FDCAN3_TxFrame,&Robotic_Arm_Motor[J5],Motor_Save_Zero_Position);
	//DM_Motor_Command(&FDCAN3_TxFrame,&Robotic_Arm_Motor[J6],Motor_Save_Zero_Position);
	static int robotic_arm_part = 0;
	for(;;)
    {
		Robotic_Arm_set(robotic_arm_part, chassis_info.activated_flag);
		robotic_arm_part = (robotic_arm_part + 1) % 3;

        Chassis_set(chassis_info.activated_flag);

        Elevator_set(chassis_info.activated_flag);

        osDelay(1);
        //USART_Vofa_Justfloat_Transmit(Robotic_Arm_Motor[J2].Data.Temp_Target_Position, Robotic_Arm_Motor[J3].Data.Temp_Target_Position, Robotic_Arm_Motor[J3].Data.Temp_Target_Position);
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
    }
    else {
	    DM_Motor_Command(&FDCAN1_TxFrame,&Elevator_Motor[LF],Motor_Disable);
	    DM_Motor_Command(&FDCAN1_TxFrame,&Elevator_Motor[LB],Motor_Disable);
        osDelay(1);
        DM_Motor_Command(&FDCAN1_TxFrame,&Elevator_Motor[RB],Motor_Disable);
	    DM_Motor_Command(&FDCAN1_TxFrame,&Elevator_Motor[RF],Motor_Disable);
    }
}

void Robotic_Arm_set(const int part, const bool activated)
{
	if(activated){
		if (part == 0) {
	    	DM_Motor_Command(&FDCAN3_TxFrame,&Robotic_Arm_Motor[J1],Motor_Enable);
	    	DM_Motor_Command(&FDCAN3_TxFrame,&Robotic_Arm_Motor[J2],Motor_Enable);
			DM_Motor_CAN_TxMessage(&FDCAN3_TxFrame, &Robotic_Arm_Motor[J1], Robotic_Arm_Motor[J1].Data.Temp_Target_Position, MIT_NO_USE, 38.0f, 4.2f, Robotic_Arm_Motor[J1].Data.Feedforward);
			DM_Motor_CAN_TxMessage(&FDCAN3_TxFrame, &Robotic_Arm_Motor[J2], Robotic_Arm_Motor[J2].Data.Temp_Target_Position, MIT_NO_USE, 45.0f, 12.2f, Robotic_Arm_Motor[J2].Data.Feedforward);
		} else if (part == 1) {
	    	DM_Motor_Command(&FDCAN3_TxFrame,&Robotic_Arm_Motor[J3],Motor_Enable);
	    	DM_Motor_Command(&FDCAN3_TxFrame,&Robotic_Arm_Motor[J4],Motor_Enable);
			DM_Motor_CAN_TxMessage(&FDCAN3_TxFrame, &Robotic_Arm_Motor[J3], Robotic_Arm_Motor[J3].Data.Temp_Target_Position, MIT_NO_USE, 45.0f, 12.2f, Robotic_Arm_Motor[J3].Data.Feedforward);
			DM_Motor_CAN_TxMessage(&FDCAN3_TxFrame, &Robotic_Arm_Motor[J4], Robotic_Arm_Motor[J4].Data.Temp_Target_Position, MIT_NO_USE, 18.0f, 1.2f, Robotic_Arm_Motor[J4].Data.Feedforward);
		} else if (part == 2) {
	    	DM_Motor_Command(&FDCAN3_TxFrame,&Robotic_Arm_Motor[J5],Motor_Enable);
	    	DM_Motor_Command(&FDCAN3_TxFrame,&Robotic_Arm_Motor[J6],Motor_Enable);
			DM_Motor_CAN_TxMessage(&FDCAN3_TxFrame, &Robotic_Arm_Motor[J5], Robotic_Arm_Motor[J5].Data.Temp_Target_Position, MIT_NO_USE, 20.0f, 1.2f, Robotic_Arm_Motor[J5].Data.Feedforward);
			DM_Motor_CAN_TxMessage(&FDCAN3_TxFrame, &Robotic_Arm_Motor[J6], Robotic_Arm_Motor[J6].Data.Temp_Target_Position, MIT_NO_USE, 12.0f, 1.2f, Robotic_Arm_Motor[J6].Data.Feedforward);
		}
	}
	else {	
		if (part == 0) {
	    	DM_Motor_Command(&FDCAN3_TxFrame,&Robotic_Arm_Motor[J1],Motor_Disable);
	    	DM_Motor_Command(&FDCAN3_TxFrame,&Robotic_Arm_Motor[J2],Motor_Disable);
		} else if (part == 1) {
	    	DM_Motor_Command(&FDCAN3_TxFrame,&Robotic_Arm_Motor[J3],Motor_Disable);
	    	DM_Motor_Command(&FDCAN3_TxFrame,&Robotic_Arm_Motor[J4],Motor_Disable);
		} else if (part == 2) {
	    	DM_Motor_Command(&FDCAN3_TxFrame,&Robotic_Arm_Motor[J5],Motor_Disable);
	    	DM_Motor_Command(&FDCAN3_TxFrame,&Robotic_Arm_Motor[J6],Motor_Disable);
		}
	}
}

void Chassis_set(const bool activated)
{
	if (activated) {
		M3508_motor_crt_ctrl(&hfdcan2, 0x200,
							 Chassis_Motor[LF].Data.Final_Output,
							 Chassis_Motor[LB].Data.Final_Output,
							 Chassis_Motor[RB].Data.Final_Output,
							 Chassis_Motor[RF].Data.Final_Output);
		if (hand_state == HAND_OPEN) {
			M2006_motor_crt_ctrl(&hfdcan2, 0x1FF, 1000, 0, 0, 0);
		} else {
			M2006_motor_crt_ctrl(&hfdcan2, 0x1FF, -2500, 0, 0, 0);
		}
	} else {
		M3508_motor_crt_ctrl(&hfdcan2, 0x200, 0, 0, 0, 0);
		M2006_motor_crt_ctrl(&hfdcan2, 0x1FF, 0, 0, 0, 0);
	}
}
