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

/* USER CODE BEGIN Header_Detect_Task */
static void chassis_set_mode(Chassis_Info_Typedef* chassis);
static void chassis_ctrl_info_get(void);
static void chassis_wheel_cal(void);

extern Chassis_Info_Typedef chassis_info;
float joint_data[6] = {1.1f,1.2f,1.3f,1.4f,3.2f,1.6f};
uint8_t receive_data[51];
extern float joint_data_receive[12]; //to be used
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

		Detect_Task_SysTick = osKernelSysTick();

        USART_Vofa_Justfloat_Transmit(chassis_info.vx,chassis_info.vy,chassis_info.vw);

        osDelay(1);
    }
    /* USER CODE END Detect_Task */
}

static void chassis_set_mode(Chassis_Info_Typedef* chassis)
{
    if(chassis == NULL)
        return;

    /* Handle main chassis mode based on switch 1 */
    if (switch_is_up(remote_ctrl.rc.s[1]))
    {
        if(switch_is_mid(remote_ctrl.rc.s[0]))
        {
            chassis->last_mode = chassis->mode;
            chassis->mode = CHASSIS_ONLY;
        }
        else if(switch_is_down(remote_ctrl.rc.s[0]) || switch_is_up(remote_ctrl.rc.s[0]))
        {
            chassis->last_mode = chassis->mode;
            chassis->mode = CHASSIS_DISABLE;
        }
        else
        {
            chassis->mode = chassis->last_mode;
        }
    }
    /* Handle auto lift mode based on switch 1 mid */
    else if (switch_is_mid(remote_ctrl.rc.s[1]))
    {
        chassis->last_mode = chassis->mode;
        chassis->mode = CHASSIS_LIFT;

        if(switch_is_down(remote_ctrl.rc.s[0]))
            chassis->lift_mode = AUTO_LIFT_STAGE_1;

        else if(switch_is_mid(remote_ctrl.rc.s[0]))
            chassis->lift_mode = AUTO_LIFT_STAGE_2;

        else if(switch_is_up(remote_ctrl.rc.s[0]))
            chassis->lift_mode = AUTO_LIFT_STAGE_3;
    }
    else if (switch_is_down(remote_ctrl.rc.s[1]))
    {
        chassis->mode = chassis->last_mode;
    }
}

static void chassis_ctrl_info_get(void)
{
    chassis_info.vx = (float)remote_ctrl.rc.ch[3] * RC_TO_VX;
    chassis_info.vy = (float)remote_ctrl.rc.ch[2] * RC_TO_VY;
    chassis_info.vw = (float)remote_ctrl.rc.ch[0] * RC_TO_VW;
}

static void chassis_wheel_cal(void)
{
    const float vx = chassis_info.vx;
    const float vy = chassis_info.vy;
    const float vw = chassis_info.vw;

    M3508_Motor[0].Data.Target_Velocity =  (vx - vy - vw*ROTATE_RATIO)*WHEEL_RPM_RATIO;
    M3508_Motor[1].Data.Target_Velocity =  (vx + vy - vw*ROTATE_RATIO)*WHEEL_RPM_RATIO;
    M3508_Motor[2].Data.Target_Velocity = -(vx - vy + vw*ROTATE_RATIO)*WHEEL_RPM_RATIO;
    M3508_Motor[3].Data.Target_Velocity = -(vx + vy + vw*ROTATE_RATIO)*WHEEL_RPM_RATIO;
}
