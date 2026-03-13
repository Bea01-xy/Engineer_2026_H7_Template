/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Detect_Task.h
  * @brief          : Detect task
  * @author         : GrassFan Wang
  * @date           : 2025/01/22
  * @version        : v1.0
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "Detect_Task.h"
#include "Control_Task.h"
#include "Remote_Control.h"
#include "bsp_gpio.h"
#include "Bmi088.h"
#include "INS_Task.h"
#include "usbd_cdc_if.h"
#include "Chassis_Config.h"
#include "bsp_uart.h"
#include "Motor.h"
#include "PID.h"

/* USER CODE BEGIN Header_Detect_Task */
static void chassis_set_mode(Chassis_Info_Typedef* chassis);
static void chassis_ctrl_info_get(void);
static void chassis_wheel_cal(void);

extern Chassis_Info_Typedef chassis_info;
float joint_data[6] = {1.1f,1.2f,1.3f,1.4f,3.2f,1.6f};
uint8_t receive_data[51];
extern float joint_data_receive[12]; //to be used

static float Chassis_Direction_PID_Param[PID_PARAMETER_NUM] = {0.02f, 0.007f, 0.03f, 0.2f, 2.f, 1.f, 2.f};
PID_Info_TypeDef Chassis_Direction_PID;
/**
* @brief Function implementing the StartDetectTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Detect_Task */
void Detect_Task(void)
{
    /* USER CODE BEGIN Detect_Task */
    TickType_t Detect_Task_SysTick = 0;
    PID_Init(&Chassis_Direction_PID,PID_POSITION,Chassis_Direction_PID_Param);
    /* Infinite loop */
    for(;;)
    {
        //original code
        Remote_Message_Moniter(&remote_ctrl);
		MiniPC_Receive_Info(receive_data, 12);
		MiniPC_Transmit_Info(joint_data, 6);

        chassis_set_mode(&chassis_info);
        chassis_ctrl_info_get();
        chassis_wheel_cal();

		Detect_Task_SysTick = osKernelSysTick();//no use for now
        //USART_Vofa_Justfloat_Transmit(remote_ctrl.rc.ch[3], remote_ctrl.rc.ch[4], remote_ctrl.rc.ch[5]);
        osDelay(1);
    }
    /* USER CODE END Detect_Task */
}

static void chassis_set_mode(Chassis_Info_Typedef* chassis)
{
    if(chassis == NULL)
        return;

    uint16_t s1 = remote_ctrl.rc.s[1];
    uint16_t s0 = remote_ctrl.rc.s[0];
    uint16_t sw0 = remote_ctrl.rc.sw[0];
    uint16_t sw1 = remote_ctrl.rc.sw[1];

    if (switch_is_up(sw0)) {
        chassis->last_mode = chassis->mode;
        chassis->mode = CHASSIS_DISABLE;
    } else if (switch_is_up(s1)) {
        chassis->last_mode = chassis->mode;
        chassis->last_lift_mode = chassis->lift_mode;
        chassis->mode = CHASSIS_LIFT;
        switch (s0) {
            case RC_SW_DOWN: chassis->lift_mode = LIFT_STAGE_1; break;
            case RC_SW_MID:  chassis->lift_mode = LIFT_STAGE_2; break;
            case RC_SW_UP:   chassis->lift_mode = LIFT_STAGE_3; break;
            default: break;
        }
    } else if (switch_is_down(s1)) {
        chassis->last_mode = chassis->mode;
        chassis->last_lift_mode = chassis->lift_mode;
        chassis->mode = CHASSIS_LIFT;
        switch (s0) {
            case RC_SW_DOWN: chassis->lift_mode = LIFT_STAGE_4; break;
            case RC_SW_MID:  chassis->lift_mode = LIFT_STAGE_5; break;
            case RC_SW_UP:   chassis->lift_mode = LIFT_STAGE_6; break;
            default: break;
        }
    }
}

static void chassis_ctrl_info_get(void)
{
    chassis_info.target_vx = (float)remote_ctrl.rc.ch[1] * RC_TO_VX;
    chassis_info.target_vy = (float)remote_ctrl.rc.ch[0] * RC_TO_VY;
    if (remote_ctrl.rc.ch[2] <= 3 && remote_ctrl.rc.ch[2] >= -3) {
        remote_ctrl.rc.ch[2] = 0;
    }
    chassis_info.target_vw = (float)remote_ctrl.rc.ch[2] * RC_TO_VW * 0.8f;

    Single_Angle_PID_Calculate(&Chassis_Direction_PID, chassis_info.target_direction, INS_Info.Yaw_Angle);
    chassis_info.target_vw += Chassis_Direction_PID.Output;

    chassis_info.target_direction += remote_ctrl.rc.ch[2] * RC_TO_VW * 0.058f; //integrate to get direction
    chassis_info.target_direction = F_Loop_Constrain(chassis_info.target_direction, -180.0f, 180.0f);
}

static void chassis_wheel_cal(void)
{
    const float vx = chassis_info.target_vx;
    const float vy = chassis_info.target_vy;
    const float vw = chassis_info.target_vw;

    Chassis_Motor[LF].Data.Target_Velocity =  (vx - vy - vw*ROTATE_RATIO)*WHEEL_RPM_RATIO;
    Chassis_Motor[LB].Data.Target_Velocity =  (vx + vy - vw*ROTATE_RATIO)*WHEEL_RPM_RATIO;
    Chassis_Motor[RB].Data.Target_Velocity = -(vx - vy + vw*ROTATE_RATIO)*WHEEL_RPM_RATIO;
    Chassis_Motor[RF].Data.Target_Velocity = -(vx + vy + vw*ROTATE_RATIO)*WHEEL_RPM_RATIO;
}
  