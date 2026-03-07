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
/**
  * @note turn on:  800
	*       turn off: 4150
	*/

/* USER CODE BEGIN Header_Detect_Task */
static void Chassis_Set_Mode(Chassis_Info_Typedef* chassis);
Chassis_Info_Typedef Chassis_Info;
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
		Detect_Task_SysTick = osKernelSysTick();
        if (Detect_Task_SysTick % 1000 == 0)
        {
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_14);
            USART_Vofa_Justfloat_Transmit((float)Chassis_Info.mode,0,0);
        }

        Remote_Message_Moniter(&remote_ctrl);
		MiniPC_Receive_Info(receive_data, 12);
		MiniPC_Transmit_Info(joint_data, 6);
        Chassis_Set_Mode(&Chassis_Info);
        osDelay(1);
    }
    /* USER CODE END Detect_Task */
}

static void Chassis_Set_Mode(Chassis_Info_Typedef* chassis)
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
}
