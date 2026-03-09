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
#include "Chassis_Config.h"
#include "INS_Task.h"

static void Control_Init(void);
static float SmootherStep(float NowTime,float UseTime);

static void chassis_lifting_handler(void);
static void chassis_disabled_handler(void);
static void chassis_only_handler(void);

static void M3508_cal(bool acticated);
static void DM6006_cal(void);

bool lifting_mode_changed(void);
static void DM6006_set_feedforward_and_pos(void);

Chassis_Info_Typedef chassis_info;

static float M3508_PID_Param[PID_PARAMETER_NUM] = {M3508_KP, M3508_KI, M3508_KD, M3508_Alpha, M3508_Deadband, M3508_LimitIntegral, M3508_LimitOutput};

PID_Info_TypeDef M3508_PID[4];

TickType_t Control_Task_SysTick = 0; //to lower the frequence
TickType_t Timer_When_Lift_Stage_Changed = 0;
void Control_Task(void)
{
    /* USER CODE BEGIN Control_Task */
	Control_Init();
    osDelay(1800);
    /* Infinite loop */
	for(;;)
    {
		Control_Task_SysTick = osKernelSysTick();

        switch (chassis_info.mode)
        {
            case CHASSIS_DISABLE:
                chassis_disabled_handler();
                break;
            case CHASSIS_ONLY:
                chassis_only_handler();
                break;
            case CHASSIS_LIFT:
                chassis_lifting_handler();
                break;
            default: break;
        }

	    Timer_When_Lift_Stage_Changed++;
	    //USART_Vofa_Justfloat_Transmit(DM6006_Motor[LF].Data.Target_Position, DM6006_Motor[LF].Data.Temp_Target_Position, DM6006_Motor[LF].Data.Position);
		osDelay(1);
    }
}
  /* USER CODE END Control_Task */

static void Control_Init(void)
{
    PID_Init(&M3508_PID[LF],PID_POSITION,M3508_PID_Param);
    PID_Init(&M3508_PID[LB],PID_POSITION,M3508_PID_Param);
    PID_Init(&M3508_PID[RB],PID_POSITION,M3508_PID_Param);
    PID_Init(&M3508_PID[RF],PID_POSITION,M3508_PID_Param);

    chassis_info.activated_flag = false;
    chassis_info.mode = CHASSIS_DISABLE;
    chassis_info.last_mode = CHASSIS_DISABLE;
    chassis_info.lift_mode = LIFT_STAGE_1;
    chassis_info.last_lift_mode = LIFT_STAGE_1;
    chassis_info.target_direction = 0;

    DM6006_Motor[LF].Data.Feedforward = 0;
    DM6006_Motor[LB].Data.Feedforward = 0;
    DM6006_Motor[RB].Data.Feedforward = 0;
    DM6006_Motor[RF].Data.Feedforward = 0;

    DM6006_Motor[LF].Data.Target_Position = DM6006_USUAL_POS;
    DM6006_Motor[LB].Data.Target_Position = DM6006_USUAL_POS;
    DM6006_Motor[RB].Data.Target_Position = DM6006_USUAL_POS;
    DM6006_Motor[RF].Data.Target_Position = DM6006_USUAL_POS;
}

static float SmootherStep(float NowTime,float UseTime)
{
    if (NowTime > UseTime){
        return 1.f;
    }
	float Time = (NowTime/UseTime);
    float Time2 = Time * Time;
    float Time3 = Time2 * Time;
    return (6.0f * Time3 * Time2) - (15.0f * Time2 * Time2) + (10.0f * Time3);
}

static void chassis_lifting_handler(void)
{
    if (Control_Task_SysTick % 500 == 0)
    {
        HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
        HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_14);
    }
    chassis_info.activated_flag = true;
    if (lifting_mode_changed()) {
        Timer_When_Lift_Stage_Changed = 0;
        DM6006_set_feedforward_and_pos();
        DM6006_Motor[LF].Data.Start_Position = DM6006_Motor[LF].Data.Position;
        DM6006_Motor[LB].Data.Start_Position = DM6006_Motor[LB].Data.Position;
        DM6006_Motor[RB].Data.Start_Position = DM6006_Motor[RB].Data.Position;
        DM6006_Motor[RF].Data.Start_Position = DM6006_Motor[RF].Data.Position;

        DM6006_Motor[LF].Data.Error_Position = DM6006_Motor[LF].Data.Target_Position - DM6006_Motor[LF].Data.Start_Position;
        DM6006_Motor[LB].Data.Error_Position = DM6006_Motor[LB].Data.Target_Position - DM6006_Motor[LB].Data.Start_Position;
        DM6006_Motor[RB].Data.Error_Position = DM6006_Motor[RB].Data.Target_Position - DM6006_Motor[RB].Data.Start_Position;
        DM6006_Motor[RF].Data.Error_Position = DM6006_Motor[RF].Data.Target_Position - DM6006_Motor[RF].Data.Start_Position;
    }
    M3508_cal(chassis_info.activated_flag);
    DM6006_cal();
}

static void chassis_disabled_handler(void)
{
    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,GPIO_PIN_SET);
    chassis_info.activated_flag = false;
    chassis_info.target_vx = 0.0f;
    chassis_info.target_vy = 0.0f;
    chassis_info.target_vw = 0.0f;
    M3508_Motor[LF].Data.Final_Output = 0u;
    M3508_Motor[LB].Data.Final_Output = 0u;
    M3508_Motor[RB].Data.Final_Output = 0u;
    M3508_Motor[RF].Data.Final_Output = 0u;
}

static void chassis_only_handler(void)
{
    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,GPIO_PIN_RESET);
    chassis_info.activated_flag = true;
    DM6006_Motor[LF].Data.Temp_Target_Position = DM6006_USUAL_POS;
    DM6006_Motor[LB].Data.Temp_Target_Position = DM6006_USUAL_POS;
    DM6006_Motor[RB].Data.Temp_Target_Position = DM6006_USUAL_POS;
    DM6006_Motor[RF].Data.Temp_Target_Position = DM6006_USUAL_POS;
    M3508_cal(chassis_info.activated_flag);
}

static void M3508_cal(const bool acticated)
{
    if (acticated) {
        PID_Calculate(&M3508_PID[LF], M3508_Motor[LF].Data.Target_Velocity, M3508_Motor[LF].Data.Velocity);
        PID_Calculate(&M3508_PID[LB], M3508_Motor[LB].Data.Target_Velocity, M3508_Motor[LB].Data.Velocity);
        PID_Calculate(&M3508_PID[RB], M3508_Motor[RB].Data.Target_Velocity, M3508_Motor[RB].Data.Velocity);
        PID_Calculate(&M3508_PID[RF], M3508_Motor[RF].Data.Target_Velocity, M3508_Motor[RF].Data.Velocity);

        //just for now
        M3508_Motor[LF].Data.Final_Output = M3508_PID[LF].Output;
        M3508_Motor[LB].Data.Final_Output = M3508_PID[LB].Output;
        M3508_Motor[RB].Data.Final_Output = M3508_PID[RB].Output;
        M3508_Motor[RF].Data.Final_Output = M3508_PID[RF].Output;
    }
}

static void DM6006_cal(void)
{
    DM6006_Motor[LF].Data.Temp_Target_Position = DM6006_Motor[LF].Data.Start_Position + DM6006_Motor[LF].Data.Error_Position * SmootherStep(Timer_When_Lift_Stage_Changed, LIFTING_TIME);
    DM6006_Motor[LB].Data.Temp_Target_Position = DM6006_Motor[LB].Data.Start_Position + DM6006_Motor[LB].Data.Error_Position * SmootherStep(Timer_When_Lift_Stage_Changed, LIFTING_TIME);
    DM6006_Motor[RB].Data.Temp_Target_Position = DM6006_Motor[RB].Data.Start_Position + DM6006_Motor[RB].Data.Error_Position * SmootherStep(Timer_When_Lift_Stage_Changed, LIFTING_TIME);
    DM6006_Motor[RF].Data.Temp_Target_Position = DM6006_Motor[RF].Data.Start_Position + DM6006_Motor[RF].Data.Error_Position * SmootherStep(Timer_When_Lift_Stage_Changed, LIFTING_TIME);
}

bool lifting_mode_changed(void){
    return chassis_info.last_lift_mode != chassis_info.lift_mode || chassis_info.last_mode != chassis_info.mode;
}

static void DM6006_set_feedforward_and_pos(void)
{
    switch (chassis_info.lift_mode) {
        case LIFT_STAGE_1:
            DM6006_Motor[LF].Data.Target_Position = DM6006_LF_1st_ACTIVATED_POS;
            DM6006_Motor[LB].Data.Target_Position = DM6006_LB_1st_ACTIVATED_POS;
            DM6006_Motor[RB].Data.Target_Position = DM6006_RB_1st_ACTIVATED_POS;
            DM6006_Motor[RF].Data.Target_Position = DM6006_RF_1st_ACTIVATED_POS;

            DM6006_Motor[LF].Data.Feedforward = DM6006_FEEDFORWARD_FOR_LF_RB;
            DM6006_Motor[LB].Data.Feedforward = DM6006_FEEDFORWARD_FOR_LB_RF;
            DM6006_Motor[RB].Data.Feedforward = DM6006_FEEDFORWARD_FOR_LF_RB;
            DM6006_Motor[RF].Data.Feedforward = DM6006_FEEDFORWARD_FOR_LB_RF;
            break;
        case LIFT_STAGE_2:
            DM6006_Motor[LF].Data.Target_Position = DM6006_USUAL_POS;
            DM6006_Motor[LB].Data.Target_Position = DM6006_LB_1st_ACTIVATED_POS;
            DM6006_Motor[RB].Data.Target_Position = DM6006_RB_1st_ACTIVATED_POS;
            DM6006_Motor[RF].Data.Target_Position = DM6006_USUAL_POS;

            DM6006_Motor[LF].Data.Feedforward = 0.f;
            DM6006_Motor[LB].Data.Feedforward = DM6006_FEEDFORWARD_FOR_LB_RF;
            DM6006_Motor[RB].Data.Feedforward = DM6006_FEEDFORWARD_FOR_LF_RB;
            DM6006_Motor[RF].Data.Feedforward = 0.f;
            break;
        case LIFT_STAGE_3:
            DM6006_Motor[LF].Data.Target_Position = DM6006_USUAL_POS;
            DM6006_Motor[LB].Data.Target_Position = DM6006_USUAL_POS;
            DM6006_Motor[RB].Data.Target_Position = DM6006_USUAL_POS;
            DM6006_Motor[RF].Data.Target_Position = DM6006_USUAL_POS;

            DM6006_Motor[LF].Data.Feedforward = 0.f;
            DM6006_Motor[LB].Data.Feedforward = 0.f;
            DM6006_Motor[RB].Data.Feedforward = 0.f;
            DM6006_Motor[RF].Data.Feedforward = 0.f;
            break;
        default: break;
    }
}
