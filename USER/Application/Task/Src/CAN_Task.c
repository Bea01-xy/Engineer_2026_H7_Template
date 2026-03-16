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

#define CAN_ID_SLAVE_J6_GRIPPER_CMD  0x2F0U

/* 由 Control_Task 写入，本任务发送给从板 */
int16_t Slave_J6_Output = 0;

float J6_Output = 0;
float Gripper_Output = 0;

/* USER CODE BEGIN Header_CAN_Task */
/**
* @brief Function implementing the StartCANTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CAN_Task */
void Elevator_set(bool activated);
/** part: 0/1/2，分三次发送不同关节，减轻 CAN3 负载 */
void Robotic_Arm_set(bool activated, int part);
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
	static int robotic_arm_part = 0;  /* 每循环递增取 0/1/2，保证连续三次不重复 */
	for(;;)
    {
		Robotic_Arm_set(chassis_info.activated_flag, robotic_arm_part);
		robotic_arm_part = (robotic_arm_part + 1) % 3;  /* 0->1->2->0 */
        Chassis_set(chassis_info.activated_flag);
        Elevator_set(chassis_info.activated_flag);
        
        osDelay(1);
        //USART_Vofa_Justfloat_Transmit(Robotic_Arm_Motor[J1].Data.Position, Robotic_Arm_Motor[J2].Data.Position, Robotic_Arm_Motor[J4].Data.Position);
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

void Robotic_Arm_set(const bool activated, const int part)
{
	/* 分三部分发送，避免一次性控制全部关节 */
	if (part == 0) {
		/* part 0: J1,J2 */
		if (activated) {
			DM_Motor_Command(&FDCAN3_TxFrame, &Robotic_Arm_Motor[J1], Motor_Enable);
			DM_Motor_Command(&FDCAN3_TxFrame, &Robotic_Arm_Motor[J2], Motor_Enable);
			osDelay(1);
			// KP=14.0f*2=28.0f, KD=2.1f*2=4.2f
			DM_Motor_CAN_TxMessage(&FDCAN3_TxFrame, &Robotic_Arm_Motor[J1], Robotic_Arm_Motor[J1].Data.Temp_Target_Position, MIT_NO_USE, 28.0f, 4.2f, Robotic_Arm_Motor[J1].Data.Feedforward);
			DM_Motor_CAN_TxMessage(&FDCAN3_TxFrame, &Robotic_Arm_Motor[J2], Robotic_Arm_Motor[J2].Data.Temp_Target_Position, MIT_NO_USE, 28.0f, 4.2f, Robotic_Arm_Motor[J2].Data.Feedforward);
		} else {
			DM_Motor_Command(&FDCAN3_TxFrame, &Robotic_Arm_Motor[J1], Motor_Disable);
			DM_Motor_Command(&FDCAN3_TxFrame, &Robotic_Arm_Motor[J2], Motor_Disable);
		}
	} else if (part == 1) {
		/* part 1: J3,J4 */
		if (activated) {
			DM_Motor_Command(&FDCAN3_TxFrame, &Robotic_Arm_Motor[J3], Motor_Enable);
			DM_Motor_Command(&FDCAN3_TxFrame, &Robotic_Arm_Motor[J4], Motor_Enable);
			osDelay(1);
			// KP=14.0f*2=28.0f, KD=2.1f*2=4.2f for J3; KP=5.0f*2=10.0f, KD=0.1f*2=0.2f for J4
			DM_Motor_CAN_TxMessage(&FDCAN3_TxFrame, &Robotic_Arm_Motor[J3], Robotic_Arm_Motor[J3].Data.Temp_Target_Position, MIT_NO_USE, 28.0f, 4.2f, Robotic_Arm_Motor[J3].Data.Feedforward);
			DM_Motor_CAN_TxMessage(&FDCAN3_TxFrame, &Robotic_Arm_Motor[J4], Robotic_Arm_Motor[J4].Data.Temp_Target_Position, MIT_NO_USE, 10.0f, 0.2f, Robotic_Arm_Motor[J4].Data.Feedforward);
		} else {
			DM_Motor_Command(&FDCAN3_TxFrame, &Robotic_Arm_Motor[J3], Motor_Disable);
			DM_Motor_Command(&FDCAN3_TxFrame, &Robotic_Arm_Motor[J4], Motor_Disable);
		}
	} else if (part == 2) {
		/* part 2: J5,J6 */
		if (activated) {
			DM_Motor_Command(&FDCAN3_TxFrame, &Robotic_Arm_Motor[J5], Motor_Enable);
			DM_Motor_Command(&FDCAN3_TxFrame, &Robotic_Arm_Motor[J6], Motor_Enable);
			osDelay(1);
			// KP=5.0f*2=10.0f, KD=0.1f*2=0.2f
			DM_Motor_CAN_TxMessage(&FDCAN3_TxFrame, &Robotic_Arm_Motor[J5], Robotic_Arm_Motor[J5].Data.Temp_Target_Position, MIT_NO_USE, 10.0f, 0.2f, Robotic_Arm_Motor[J5].Data.Feedforward);
			DM_Motor_CAN_TxMessage(&FDCAN3_TxFrame, &Robotic_Arm_Motor[J6], Robotic_Arm_Motor[J6].Data.Temp_Target_Position, MIT_NO_USE, 10.0f, 0.2f, Robotic_Arm_Motor[J6].Data.Feedforward);
		} else {
			DM_Motor_Command(&FDCAN3_TxFrame, &Robotic_Arm_Motor[J5], Motor_Disable);
			DM_Motor_Command(&FDCAN3_TxFrame, &Robotic_Arm_Motor[J6], Motor_Disable);
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
			M2006_motor_crt_ctrl(&hfdcan2, 0x1FF, -1000,
							 0, 0, 0);
		} else {
			M2006_motor_crt_ctrl(&hfdcan2, 0x1FF, 3000,
							 0, 0, 0);
		}
	} else {
		M3508_motor_crt_ctrl(&hfdcan2, 0x200, 0, 0, 0, 0);
		M2006_motor_crt_ctrl(&hfdcan2, 0x1FF, 0,
							 0, 0, 0);
	}
}
