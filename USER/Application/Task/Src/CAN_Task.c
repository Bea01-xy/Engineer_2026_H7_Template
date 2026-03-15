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
void Robotic_Arm_set(bool activated);
void Chassis_set(void);

extern Chassis_Info_Typedef chassis_info;

void CAN_Task(void)
{
	DM_Motor_Command(&FDCAN1_TxFrame,&Elevator_Motor[LF],Motor_Save_Zero_Position);
	DM_Motor_Command(&FDCAN1_TxFrame,&Elevator_Motor[LB],Motor_Save_Zero_Position);
    osDelay(1);
	DM_Motor_Command(&FDCAN1_TxFrame,&Elevator_Motor[RB],Motor_Save_Zero_Position);
	DM_Motor_Command(&FDCAN1_TxFrame,&Elevator_Motor[RF],Motor_Save_Zero_Position);
 
	//DM_Motor_Command(&FDCAN3_TxFrame,&Robotic_Arm_Motor[J1],Motor_Save_Zero_Position);
	//DM_Motor_Command(&FDCAN3_TxFrame,&Robotic_Arm_Motor[J2],Motor_Save_Zero_Position);
	//DM_Motor_Command(&FDCAN3_TxFrame,&Robotic_Arm_Motor[J3],Motor_Save_Zero_Position);
    TickType_t CAN_Task_SysTick = 0;
	for(;;)
    {
		CAN_Task_SysTick = osKernelSysTick();
	    if(CAN_Task_SysTick % 2 == 0){
	    }
        Elevator_set(chassis_info.activated_flag);
        Robotic_Arm_set(chassis_info.activated_flag);
        Chassis_set();
        
        osDelay(1);

        USART_Vofa_Justfloat_Transmit(Robotic_Arm_Motor[J1].Data.Position, Robotic_Arm_Motor[J2].Data.Position, Robotic_Arm_Motor[J4].Data.Position);
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

void Robotic_Arm_set(const bool activated)
{
    if (activated) {
        DM_Motor_Command(&FDCAN3_TxFrame,&Robotic_Arm_Motor[J1],Motor_Enable);
        DM_Motor_Command(&FDCAN3_TxFrame,&Robotic_Arm_Motor[J2],Motor_Enable);
        DM_Motor_Command(&FDCAN3_TxFrame,&Robotic_Arm_Motor[J3],Motor_Enable);
        osDelay(1);
        DM_Motor_Command(&FDCAN3_TxFrame,&Robotic_Arm_Motor[J4],Motor_Enable);
        DM_Motor_Command(&FDCAN3_TxFrame,&Robotic_Arm_Motor[J5],Motor_Enable);

        /* 仅发送 CAN：目标位置等由 Control_Task 写入 */
        // DM_Motor_CAN_TxMessage(&FDCAN3_TxFrame,&Robotic_Arm_Motor[J1],Robotic_Arm_Motor[J1].Data.Temp_Target_Position, MIT_NO_USE,10.0f,2.0f,Robotic_Arm_Motor[J1].Data.Feedforward);
        // DM_Motor_CAN_TxMessage(&FDCAN3_TxFrame,&Robotic_Arm_Motor[J2],Robotic_Arm_Motor[J2].Data.Temp_Target_Position, MIT_NO_USE,20.0f,2.0f,Robotic_Arm_Motor[J2].Data.Feedforward);
        // DM_Motor_CAN_TxMessage(&FDCAN3_TxFrame,&Robotic_Arm_Motor[J3],Robotic_Arm_Motor[J3].Data.Temp_Target_Position, MIT_NO_USE,20.0f,2.0f,Robotic_Arm_Motor[J3].Data.Feedforward);
        // osDelay(1);
        // DM_Motor_CAN_TxMessage(&FDCAN3_TxFrame,&Robotic_Arm_Motor[J4],Robotic_Arm_Motor[J4].Data.Temp_Target_Position, MIT_NO_USE,3.0f,0.1f,Robotic_Arm_Motor[J4].Data.Feedforward);
        // DM_Motor_CAN_TxMessage(&FDCAN3_TxFrame,&Robotic_Arm_Motor[J5],Robotic_Arm_Motor[J5].Data.Temp_Target_Position, MIT_NO_USE,3.0f,0.1f,Robotic_Arm_Motor[J5].Data.Feedforward);

        /* 发往从板：J6 用 Slave_J6_Output（Control_Task 写入），夹爪用 Gripper_Output */
        J6_Output = (float)Slave_J6_Output;
        osDelay(1);
        Slave_Board_J6_Gripper_Send(&hfdcan3, CAN_ID_SLAVE_J6_GRIPPER_CMD,
            J6_Output, Gripper_Output);
    }
    else {
        DM_Motor_Command(&FDCAN3_TxFrame,&Robotic_Arm_Motor[J1],Motor_Disable);
        DM_Motor_Command(&FDCAN3_TxFrame,&Robotic_Arm_Motor[J2],Motor_Disable);
        DM_Motor_Command(&FDCAN3_TxFrame,&Robotic_Arm_Motor[J3],Motor_Disable);
        osDelay(1);
        DM_Motor_Command(&FDCAN3_TxFrame,&Robotic_Arm_Motor[J4],Motor_Disable);
        DM_Motor_Command(&FDCAN3_TxFrame,&Robotic_Arm_Motor[J5],Motor_Disable);
    }
}

void Chassis_set(void)
{
    M3508_motor_crt_ctrl(&hfdcan2, 0x200, Chassis_Motor[LF].Data.Final_Output,
        Chassis_Motor[LB].Data.Final_Output,Chassis_Motor[RB].Data.Final_Output,Chassis_Motor[RF].Data.Final_Output);

}
