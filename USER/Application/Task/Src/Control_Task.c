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

static void Chassis_Motor_cal(bool acticated);
static void Elevator_Motor_cal(void);

static bool lifting_mode_changed(void);
static void Elevator_set_feedforward_and_pos(void);
static void chassis_set_leds(GPIO_PinState state);

Chassis_Info_Typedef chassis_info;

static float Chassis_PID_Param[PID_PARAMETER_NUM] = {CHASSIS_KP, CHASSIS_KI, CHASSIS_KD, CHASSIS_Alpha, CHASSIS_Deadband, CHASSIS_LimitIntegral, CHASSIS_LimitOutput};

PID_Info_TypeDef Chassis_PID[4];

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

        switch (chassis_info.mode) {
            case CHASSIS_DISABLE:
                chassis_disabled_handler();
                break;
            case CHASSIS_LIFT:
                chassis_lifting_handler();
                break;
            default: break;
        }

	    Timer_When_Lift_Stage_Changed++;
	    //USART_Vofa_Justfloat_Transmit(Elevator_Motor[LF].Data.Target_Position, Elevator_Motor[LF].Data.Temp_Target_Position, Elevator_Motor[LF].Data.Position);
		osDelay(1);
    }
}
  /* USER CODE END Control_Task */

static void Control_Init(void)
{
    PID_Init(&Chassis_PID[LF],PID_POSITION,Chassis_PID_Param);
    PID_Init(&Chassis_PID[LB],PID_POSITION,Chassis_PID_Param);
    PID_Init(&Chassis_PID[RB],PID_POSITION,Chassis_PID_Param);
    PID_Init(&Chassis_PID[RF],PID_POSITION,Chassis_PID_Param);

    chassis_info.activated_flag = false;
    chassis_info.mode = CHASSIS_DISABLE;
    chassis_info.last_mode = CHASSIS_DISABLE;
    chassis_info.lift_mode = LIFT_STAGE_1;
    chassis_info.last_lift_mode = LIFT_STAGE_1;
    chassis_info.target_direction = 0;

    Elevator_Motor[LF].Data.Feedforward = 0;
    Elevator_Motor[LB].Data.Feedforward = 0;
    Elevator_Motor[RB].Data.Feedforward = 0;
    Elevator_Motor[RF].Data.Feedforward = 0;

    Elevator_Motor[LF].Data.Target_Position = ELEVATOR_USUAL_POS;
    Elevator_Motor[LB].Data.Target_Position = ELEVATOR_USUAL_POS;
    Elevator_Motor[RB].Data.Target_Position = ELEVATOR_USUAL_POS;
    Elevator_Motor[RF].Data.Target_Position = ELEVATOR_USUAL_POS;
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
        Elevator_set_feedforward_and_pos();
        Elevator_Motor[LF].Data.Start_Position = Elevator_Motor[LF].Data.Position;
        Elevator_Motor[LB].Data.Start_Position = Elevator_Motor[LB].Data.Position;
        Elevator_Motor[RB].Data.Start_Position = Elevator_Motor[RB].Data.Position;
        Elevator_Motor[RF].Data.Start_Position = Elevator_Motor[RF].Data.Position;

        Elevator_Motor[LF].Data.Error_Position = Elevator_Motor[LF].Data.Target_Position - Elevator_Motor[LF].Data.Start_Position;
        Elevator_Motor[LB].Data.Error_Position = Elevator_Motor[LB].Data.Target_Position - Elevator_Motor[LB].Data.Start_Position;
        Elevator_Motor[RB].Data.Error_Position = Elevator_Motor[RB].Data.Target_Position - Elevator_Motor[RB].Data.Start_Position;
        Elevator_Motor[RF].Data.Error_Position = Elevator_Motor[RF].Data.Target_Position - Elevator_Motor[RF].Data.Start_Position;
    }
    Chassis_Motor_cal(chassis_info.activated_flag);
    Elevator_Motor_cal();
}

static void chassis_disabled_handler(void)
{
    chassis_set_leds(GPIO_PIN_SET);
    chassis_info.activated_flag = false;
    chassis_info.target_vx = 0.0f;
    chassis_info.target_vy = 0.0f;
    chassis_info.target_vw = 0.0f;
    Chassis_Motor[LF].Data.Final_Output = 0u;
    Chassis_Motor[LB].Data.Final_Output = 0u;
    Chassis_Motor[RB].Data.Final_Output = 0u;
    Chassis_Motor[RF].Data.Final_Output = 0u;
}

static void Chassis_Motor_cal(const bool acticated)
{
    if (acticated) {
        PID_Calculate(&Chassis_PID[LF], Chassis_Motor[LF].Data.Target_Velocity, Chassis_Motor[LF].Data.Velocity);
        PID_Calculate(&Chassis_PID[LB], Chassis_Motor[LB].Data.Target_Velocity, Chassis_Motor[LB].Data.Velocity);
        PID_Calculate(&Chassis_PID[RB], Chassis_Motor[RB].Data.Target_Velocity, Chassis_Motor[RB].Data.Velocity);
        PID_Calculate(&Chassis_PID[RF], Chassis_Motor[RF].Data.Target_Velocity, Chassis_Motor[RF].Data.Velocity);

        //just for now
        Chassis_Motor[LF].Data.Final_Output = Chassis_PID[LF].Output;
        Chassis_Motor[LB].Data.Final_Output = Chassis_PID[LB].Output;
        Chassis_Motor[RB].Data.Final_Output = Chassis_PID[RB].Output;
        Chassis_Motor[RF].Data.Final_Output = Chassis_PID[RF].Output;
    }
}

static void Elevator_Motor_cal(void)
{
    Elevator_Motor[LF].Data.Temp_Target_Position = Elevator_Motor[LF].Data.Start_Position + Elevator_Motor[LF].Data.Error_Position * 
    SmootherStep(Timer_When_Lift_Stage_Changed, LIFTING_TIME);
    Elevator_Motor[LB].Data.Temp_Target_Position = Elevator_Motor[LB].Data.Start_Position + Elevator_Motor[LB].Data.Error_Position *
     SmootherStep(Timer_When_Lift_Stage_Changed, LIFTING_TIME);
    Elevator_Motor[RB].Data.Temp_Target_Position = Elevator_Motor[RB].Data.Start_Position + Elevator_Motor[RB].Data.Error_Position *
     SmootherStep(Timer_When_Lift_Stage_Changed, LIFTING_TIME);
    Elevator_Motor[RF].Data.Temp_Target_Position = Elevator_Motor[RF].Data.Start_Position + Elevator_Motor[RF].Data.Error_Position *
     SmootherStep(Timer_When_Lift_Stage_Changed, LIFTING_TIME);
}

static bool lifting_mode_changed(void){
    return chassis_info.last_lift_mode != chassis_info.lift_mode || chassis_info.last_mode != chassis_info.mode;
}

static void Elevator_set_feedforward_and_pos(void)
{
    switch (chassis_info.lift_mode) {
        case LIFT_STAGE_1:
            Elevator_Motor[LF].Data.Target_Position = ELEVATOR_LF_1st_ACTIVATED_POS;
            Elevator_Motor[LB].Data.Target_Position = ELEVATOR_LB_1st_ACTIVATED_POS;
            Elevator_Motor[RB].Data.Target_Position = ELEVATOR_RB_1st_ACTIVATED_POS;
            Elevator_Motor[RF].Data.Target_Position = ELEVATOR_RF_1st_ACTIVATED_POS;

            Elevator_Motor[LF].Data.Feedforward = ELEVATOR_FEEDFORWARD_FOR_LF_RB;
            Elevator_Motor[LB].Data.Feedforward = ELEVATOR_FEEDFORWARD_FOR_LB_RF;
            Elevator_Motor[RB].Data.Feedforward = ELEVATOR_FEEDFORWARD_FOR_LF_RB;
            Elevator_Motor[RF].Data.Feedforward = ELEVATOR_FEEDFORWARD_FOR_LB_RF;
            break;
        case LIFT_STAGE_2:
            Elevator_Motor[LF].Data.Target_Position = ELEVATOR_USUAL_POS;
            Elevator_Motor[LB].Data.Target_Position = ELEVATOR_LB_1st_ACTIVATED_POS;
            Elevator_Motor[RB].Data.Target_Position = ELEVATOR_RB_1st_ACTIVATED_POS;
            Elevator_Motor[RF].Data.Target_Position = ELEVATOR_USUAL_POS;

            Elevator_Motor[LF].Data.Feedforward = 0.f;
            Elevator_Motor[LB].Data.Feedforward = ELEVATOR_FEEDFORWARD_FOR_LB_RF;
            Elevator_Motor[RB].Data.Feedforward = ELEVATOR_FEEDFORWARD_FOR_LF_RB;
            Elevator_Motor[RF].Data.Feedforward = 0.f;
            break;
        case LIFT_STAGE_3:
            Elevator_Motor[LF].Data.Target_Position = ELEVATOR_USUAL_POS;
            Elevator_Motor[LB].Data.Target_Position = ELEVATOR_USUAL_POS;
            Elevator_Motor[RB].Data.Target_Position = ELEVATOR_USUAL_POS;
            Elevator_Motor[RF].Data.Target_Position = ELEVATOR_USUAL_POS;

            Elevator_Motor[LF].Data.Feedforward = 0.f;
            Elevator_Motor[LB].Data.Feedforward = 0.f;
            Elevator_Motor[RB].Data.Feedforward = 0.f;
            Elevator_Motor[RF].Data.Feedforward = 0.f;
            break;
        case LIFT_STAGE_4:
            Elevator_Motor[LF].Data.Target_Position = ELEVATOR_LF_2nd_ACTIVATED_POS;
            Elevator_Motor[LB].Data.Target_Position = ELEVATOR_LB_2nd_ACTIVATED_POS;
            Elevator_Motor[RB].Data.Target_Position = ELEVATOR_RB_2nd_ACTIVATED_POS;
            Elevator_Motor[RF].Data.Target_Position = ELEVATOR_RF_2nd_ACTIVATED_POS;

            Elevator_Motor[LF].Data.Feedforward = ELEVATOR_FEEDFORWARD_FOR_LF_RB;
            Elevator_Motor[LB].Data.Feedforward = ELEVATOR_FEEDFORWARD_FOR_LB_RF;
            Elevator_Motor[RB].Data.Feedforward = ELEVATOR_FEEDFORWARD_FOR_LF_RB;
            Elevator_Motor[RF].Data.Feedforward = ELEVATOR_FEEDFORWARD_FOR_LB_RF;
            break;
        case LIFT_STAGE_5:
            Elevator_Motor[LF].Data.Target_Position = ELEVATOR_USUAL_POS;
            Elevator_Motor[LB].Data.Target_Position = ELEVATOR_LB_2nd_ACTIVATED_POS;
            Elevator_Motor[RB].Data.Target_Position = ELEVATOR_RB_2nd_ACTIVATED_POS;
            Elevator_Motor[RF].Data.Target_Position = ELEVATOR_USUAL_POS;

            Elevator_Motor[LF].Data.Feedforward = 0.f;
            Elevator_Motor[LB].Data.Feedforward = ELEVATOR_FEEDFORWARD_FOR_LB_RF;
            Elevator_Motor[RB].Data.Feedforward = ELEVATOR_FEEDFORWARD_FOR_LF_RB;
            Elevator_Motor[RF].Data.Feedforward = 0.f;
            break;
        case LIFT_STAGE_6:
            Elevator_Motor[LF].Data.Target_Position = ELEVATOR_USUAL_POS;
            Elevator_Motor[LB].Data.Target_Position = ELEVATOR_USUAL_POS;
            Elevator_Motor[RB].Data.Target_Position = ELEVATOR_USUAL_POS;
            Elevator_Motor[RF].Data.Target_Position = ELEVATOR_USUAL_POS;

            Elevator_Motor[LF].Data.Feedforward = 0.f;
            Elevator_Motor[LB].Data.Feedforward = 0.f;
            Elevator_Motor[RB].Data.Feedforward = 0.f;
            Elevator_Motor[RF].Data.Feedforward = 0.f;
            break;
        default:
            break;
    }
}

static void chassis_set_leds(GPIO_PinState state)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, state);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, state);
}
