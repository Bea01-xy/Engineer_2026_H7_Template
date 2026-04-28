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
#include "Power_Limit_communication.h"
#include "cmsis_os.h"
#include "bsp_uart.h"
#include "Remote_Control.h"
#include "PID.h"
#include "Motor.h"
#include "arm_math.h"
#include "Chassis_Config.h"
#include "INS_Task.h"
#include "CAN_Task.h"
#include "Robotic_Arm_Config.h"
#include "PowerLimiter.h"
#include "GravityCompensation.h"
#include "Referee_System.h"

static void Control_Init(void);
static float SmootherStep(float NowTime,float UseTime);

static void chassis_lifting_handler(void);
static void chassis_auto_lifting_handler(void);
static void chassis_disabled_handler(void);

static void Chassis_Motor_cal(bool acticated);
static void Elevator_Motor_cal(void);
static void Robotic_Arm_Motor_cal(void);

static bool lifting_mode_changed(void);
static void Elevator_set_feedforward_and_pos(void);
static void Elevator_set_start_error_pos(void);
static void Robotic_Arm_set_start_error_pos(void);
static void Robotic_Arm_set_feedfoward(void);
static void chassis_set_leds(GPIO_PinState state);

Chassis_Info_Typedef chassis_info;
extern Referee_System_Info_TypeDef Referee_System_Info;

// 功率限制器相关变量
static Limiter_t chassis_power_limiter[4];
static Limiter_t* chassis_limiter_list[4] = {
    &chassis_power_limiter[0],
    &chassis_power_limiter[1],
    &chassis_power_limiter[2],
    &chassis_power_limiter[3]
};
static LimiterScheduler_t chassis_scheduler;

// 置信度边界参数（需要根据实际测试调整）
#define CHASSIS_E_UPPER  3000.0f   // 转速差总和上界
#define CHASSIS_E_LOWER  500.0f    // 转速差总和下界

static float Chassis_PID_Param[PID_PARAMETER_NUM] = {CHASSIS_KP, CHASSIS_KI, CHASSIS_KD, CHASSIS_Alpha, CHASSIS_Deadband, CHASSIS_LimitIntegral, CHASSIS_LimitOutput};

static float Robotic_Arm_FF_PID_Param_A[PID_PARAMETER_NUM] = {
    ROBOTIC_ARM_FF_A_KP, ROBOTIC_ARM_FF_A_KI, ROBOTIC_ARM_FF_A_KD,
    ROBOTIC_ARM_FF_A_Alpha, ROBOTIC_ARM_FF_A_Deadband,
    ROBOTIC_ARM_FF_A_LimitIntegral, ROBOTIC_ARM_FF_A_LimitOutput
};
static float Robotic_Arm_FF_PID_Param_B[PID_PARAMETER_NUM] = {
    ROBOTIC_ARM_FF_B_KP, ROBOTIC_ARM_FF_B_KI, ROBOTIC_ARM_FF_B_KD,
    ROBOTIC_ARM_FF_B_Alpha, ROBOTIC_ARM_FF_B_Deadband,
    ROBOTIC_ARM_FF_B_LimitIntegral, ROBOTIC_ARM_FF_B_LimitOutput
};

PID_Info_TypeDef Chassis_PID[4];
PID_Info_TypeDef Robotic_Arm_FF_PID[ROBOTIC_ARM_DOF];

TickType_t Control_Task_SysTick = 0;
TickType_t Timer_When_Lift_Stage_Changed = 0;
TickType_t Timer_When_Mode_Changed = 0;
void Control_Task(void)
{
    /* USER CODE BEGIN Control_Task */
	Control_Init();
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
            case CHASSIS_AUTO_LIFT:
                chassis_auto_lifting_handler();
                break;
            default: break;
        }

	    Timer_When_Lift_Stage_Changed++;
        Timer_When_Mode_Changed++;

        float power_data[9] = {
            Chassis_Motor[LF].Data.Current, Chassis_Motor[LB].Data.Current,
            Chassis_Motor[RB].Data.Current, Chassis_Motor[RF].Data.Current,
            Chassis_Motor[LF].Data.Velocity, Chassis_Motor[LB].Data.Velocity,
            Chassis_Motor[RB].Data.Velocity, Chassis_Motor[RF].Data.Velocity,
            PowerMeter_Get_Power(),
        };
        //USART_Vofa_SendFloat(power_data, 9);
        //USART_Vofa_SendFloat(gGravityComp.feedforward_torque, 6);
		osDelayUntil(&Control_Task_SysTick, 1);
    }
}

static void Control_Init(void)
{
    PID_Init(&Chassis_PID[LF],PID_POSITION,Chassis_PID_Param);
    PID_Init(&Chassis_PID[LB],PID_POSITION,Chassis_PID_Param);
    PID_Init(&Chassis_PID[RB],PID_POSITION,Chassis_PID_Param);
    PID_Init(&Chassis_PID[RF],PID_POSITION,Chassis_PID_Param);

    PID_Init(&Robotic_Arm_FF_PID[J1], PID_POSITION, Robotic_Arm_FF_PID_Param_A);
    PID_Init(&Robotic_Arm_FF_PID[J2], PID_POSITION, Robotic_Arm_FF_PID_Param_A);
    PID_Init(&Robotic_Arm_FF_PID[J3], PID_POSITION, Robotic_Arm_FF_PID_Param_A);
    PID_Init(&Robotic_Arm_FF_PID[J4], PID_POSITION, Robotic_Arm_FF_PID_Param_B);
    PID_Init(&Robotic_Arm_FF_PID[J5], PID_POSITION, Robotic_Arm_FF_PID_Param_B);
    PID_Init(&Robotic_Arm_FF_PID[J6], PID_POSITION, Robotic_Arm_FF_PID_Param_B);
    for (uint8_t i = 0; i < ROBOTIC_ARM_DOF; i++) {
        Robotic_Arm_Motor[i].Data.Feedforward = 0.f;
    }

    GravityComp_Init();

    // 初始化功率限制器（4个3508电机）
    for(int i = 0; i < 4; i++) {
        powerInitialiseLimiter(&chassis_power_limiter[i], MODEL_3508);
    }

    // 初始化功率限制器调度器
    powerInitialiseLimiterScheduler(&chassis_scheduler, 4, chassis_limiter_list, CHASSIS_E_UPPER, CHASSIS_E_LOWER);

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

    Robotic_Arm_Motor[J1].Data.Temp_Target_Position = J1_INITIAL_POS;
    Robotic_Arm_Motor[J2].Data.Temp_Target_Position = J2_INITIAL_POS;
    Robotic_Arm_Motor[J3].Data.Temp_Target_Position = -J3_INITIAL_POS;
    Robotic_Arm_Motor[J4].Data.Temp_Target_Position = -J4_INITIAL_POS;
    Robotic_Arm_Motor[J5].Data.Temp_Target_Position = J5_INITIAL_POS;
    Robotic_Arm_Motor[J6].Data.Temp_Target_Position = -J6_INITIAL_POS;

    chassis_info.countering_1 = true;
    chassis_info.countering_2 = false;
    Referee_System_Info.power_heat_data.buffer_energy = 60u;
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
        Elevator_set_start_error_pos();
    }
    if (mode_changed_to_normal()){
        Timer_When_Mode_Changed = 0;
        Robotic_Arm_set_start_error_pos();
    }
    Chassis_Motor_cal(chassis_info.activated_flag);
    Elevator_Motor_cal();
    Robotic_Arm_Motor_cal();
    Robotic_Arm_set_feedfoward();
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

    /* 关节失能时清空前馈 PID, 防止积分饱和与残留前馈输出 */
    for (uint8_t i = 0; i < ROBOTIC_ARM_DOF; i++) {
        Robotic_Arm_FF_PID[i].PID_Calc_Clear(&Robotic_Arm_FF_PID[i]);
        Robotic_Arm_Motor[i].Data.Feedforward = 0.f;
    }
}

static void chassis_auto_lifting_handler(void)
{
    Chassis_Motor[LF].Data.Target_Velocity =  CHASSIS_AUTO_LIFT_TARGET_VELOCITY;
    Chassis_Motor[LB].Data.Target_Velocity =  CHASSIS_AUTO_LIFT_TARGET_VELOCITY;
    Chassis_Motor[RB].Data.Target_Velocity = -CHASSIS_AUTO_LIFT_TARGET_VELOCITY;
    Chassis_Motor[RF].Data.Target_Velocity = -CHASSIS_AUTO_LIFT_TARGET_VELOCITY;

    if (abs(Chassis_Motor[LF].Data.Velocity) < CHASSIS_AUTO_LIFT_STALL_VELOCITY_TH &&
        abs(Chassis_Motor[LB].Data.Velocity) < CHASSIS_AUTO_LIFT_STALL_VELOCITY_TH &&
        abs(Chassis_Motor[RB].Data.Velocity) < CHASSIS_AUTO_LIFT_STALL_VELOCITY_TH &&
        abs(Chassis_Motor[RF].Data.Velocity) < CHASSIS_AUTO_LIFT_STALL_VELOCITY_TH &&
        chassis_info.countering_1 == true)
    {
        chassis_info.lift_counter_1++;
    }
    else if (abs(Chassis_Motor[RF].Data.Velocity) < CHASSIS_AUTO_LIFT_STALL_VELOCITY_TH &&
             abs(Chassis_Motor[LF].Data.Velocity) < CHASSIS_AUTO_LIFT_STALL_VELOCITY_TH &&
             chassis_info.countering_2 == true)
    {
        chassis_info.lift_counter_2++;
    }

    if (chassis_info.lift_counter_1 >= CHASSIS_AUTO_LIFT_STAGE1_COUNTER_TH && chassis_info.countering_1 == true)
    {
        chassis_info.lift_mode = LIFT_STAGE_2;
        chassis_info.lift_counter_1 = 0;
        chassis_info.countering_1 = false;
        chassis_info.countering_2 = true;
    }
    else if (chassis_info.lift_counter_2 >= CHASSIS_AUTO_LIFT_STAGE2_COUNTER_TH && chassis_info.countering_2 == true)
    {
        chassis_info.lift_mode = LIFT_STAGE_3;
        chassis_info.lift_counter_2 = 0;
        chassis_info.countering_1 = true;
        chassis_info.countering_2 = false;
    }
    chassis_lifting_handler();
}

#define POWER_CONTROL 1
static void Chassis_Motor_cal(const bool acticated)
{
    if (acticated) {
        // ========== Step 1: PID计算 ==========
        PID_Calculate(&Chassis_PID[LF], Chassis_Motor[LF].Data.Target_Velocity, Chassis_Motor[LF].Data.Velocity);
        PID_Calculate(&Chassis_PID[LB], Chassis_Motor[LB].Data.Target_Velocity, Chassis_Motor[LB].Data.Velocity);
        PID_Calculate(&Chassis_PID[RB], Chassis_Motor[RB].Data.Target_Velocity, Chassis_Motor[RB].Data.Velocity);
        PID_Calculate(&Chassis_PID[RF], Chassis_Motor[RF].Data.Target_Velocity, Chassis_Motor[RF].Data.Velocity);

        // ========== Step 2: 计算原始输出（PID + 前馈） ==========
        int16_t raw_output[4];
        raw_output[LF] = Chassis_PID[LF].Output + Chassis_Motor[LF].Data.Target_Velocity * CHASSIS_FF_SPEED_COEF;
        raw_output[LB] = Chassis_PID[LB].Output + Chassis_Motor[LB].Data.Target_Velocity * CHASSIS_FF_SPEED_COEF;
        raw_output[RB] = Chassis_PID[RB].Output + Chassis_Motor[RB].Data.Target_Velocity * CHASSIS_FF_SPEED_COEF;
        raw_output[RF] = Chassis_PID[RF].Output + Chassis_Motor[RF].Data.Target_Velocity * CHASSIS_FF_SPEED_COEF;

        // ========== Step 3: 功率控制 ==========

        // 3.1 更新每个电机的功率限制器数据
        for(int i = 0; i < 4; i++) {
            fp32 speed_error = Chassis_Motor[i].Data.Target_Velocity - Chassis_Motor[i].Data.Velocity;
            powerLimiterUpdate(&chassis_power_limiter[i],
                              Chassis_Motor[i].Data.Velocity,
                              raw_output[i],
                              speed_error);
        }

        // 3.2 调度器统一计算功率分配
        int16_t power_limit = 120;  // 单位：W
        if (Referee_System_Info.power_heat_data.buffer_energy <= 40u) {
            power_limit = 90;
        }
        powerSchedulerUpdate(&chassis_scheduler, power_limit);

#if POWER_CONTROL
        // 3.3 获取限幅后的输出报文
        Chassis_Motor[LF].Data.Final_Output = powerGetLimiterUpdate(&chassis_power_limiter[0], raw_output[LF]);
        Chassis_Motor[LB].Data.Final_Output = powerGetLimiterUpdate(&chassis_power_limiter[1], raw_output[LB]);
        Chassis_Motor[RB].Data.Final_Output = powerGetLimiterUpdate(&chassis_power_limiter[2], raw_output[RB]);
        Chassis_Motor[RF].Data.Final_Output = powerGetLimiterUpdate(&chassis_power_limiter[3], raw_output[RF]);
#else
        Chassis_Motor[LF].Data.Final_Output = raw_output[LF];
        Chassis_Motor[LB].Data.Final_Output = raw_output[LB];
        Chassis_Motor[RB].Data.Final_Output = raw_output[RB];
        Chassis_Motor[RF].Data.Final_Output = raw_output[RF];
#endif
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

static void Robotic_Arm_Motor_cal(void)
{
    if (Timer_When_Mode_Changed > ROBOTIC_ARM_MOVING_TIME) {
        Robotic_Arm_Motor[J1].Data.Temp_Target_Position = Robotic_Arm_Motor[J1].Data.Target_Position;
        Robotic_Arm_Motor[J2].Data.Temp_Target_Position = Robotic_Arm_Motor[J2].Data.Target_Position;
        Robotic_Arm_Motor[J3].Data.Temp_Target_Position = Robotic_Arm_Motor[J3].Data.Target_Position;
        Robotic_Arm_Motor[J4].Data.Temp_Target_Position = Robotic_Arm_Motor[J4].Data.Target_Position;
        Robotic_Arm_Motor[J5].Data.Temp_Target_Position = Robotic_Arm_Motor[J5].Data.Target_Position;
        Robotic_Arm_Motor[J6].Data.Temp_Target_Position = Robotic_Arm_Motor[J6].Data.Target_Position;
    }
    else{
        Robotic_Arm_Motor[J1].Data.Temp_Target_Position = Robotic_Arm_Motor[J1].Data.Start_Position + Robotic_Arm_Motor[J1].Data.Error_Position *
        SmootherStep(Timer_When_Lift_Stage_Changed, ROBOTIC_ARM_MOVING_TIME);
        Robotic_Arm_Motor[J2].Data.Temp_Target_Position = Robotic_Arm_Motor[J2].Data.Start_Position + Robotic_Arm_Motor[J2].Data.Error_Position *
        SmootherStep(Timer_When_Lift_Stage_Changed, ROBOTIC_ARM_MOVING_TIME);
        Robotic_Arm_Motor[J3].Data.Temp_Target_Position = Robotic_Arm_Motor[J3].Data.Start_Position + Robotic_Arm_Motor[J3].Data.Error_Position *
        SmootherStep(Timer_When_Lift_Stage_Changed, ROBOTIC_ARM_MOVING_TIME);
        Robotic_Arm_Motor[J4].Data.Temp_Target_Position = Robotic_Arm_Motor[J4].Data.Start_Position + Robotic_Arm_Motor[J4].Data.Error_Position *
        SmootherStep(Timer_When_Lift_Stage_Changed, ROBOTIC_ARM_MOVING_TIME);
        Robotic_Arm_Motor[J5].Data.Temp_Target_Position = Robotic_Arm_Motor[J5].Data.Start_Position + Robotic_Arm_Motor[J5].Data.Error_Position *
        SmootherStep(Timer_When_Lift_Stage_Changed, ROBOTIC_ARM_MOVING_TIME);
        Robotic_Arm_Motor[J6].Data.Temp_Target_Position = Robotic_Arm_Motor[J6].Data.Start_Position + Robotic_Arm_Motor[J6].Data.Error_Position *
        SmootherStep(Timer_When_Lift_Stage_Changed, ROBOTIC_ARM_MOVING_TIME);
    }
}

static bool lifting_mode_changed(void){
    return chassis_info.last_lift_mode != chassis_info.lift_mode || chassis_info.last_mode != chassis_info.mode;
}

bool mode_changed(void){
    return chassis_info.last_mode != chassis_info.mode;
}

bool mode_changed_to_normal(void){
    return (chassis_info.last_mode == CHASSIS_DISABLE) && (chassis_info.mode != CHASSIS_DISABLE);
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

static void Elevator_set_start_error_pos(void)
{
    Elevator_Motor[LF].Data.Start_Position = Elevator_Motor[LF].Data.Position;
    Elevator_Motor[LB].Data.Start_Position = Elevator_Motor[LB].Data.Position;
    Elevator_Motor[RB].Data.Start_Position = Elevator_Motor[RB].Data.Position;
    Elevator_Motor[RF].Data.Start_Position = Elevator_Motor[RF].Data.Position;

    Elevator_Motor[LF].Data.Error_Position = Elevator_Motor[LF].Data.Target_Position - Elevator_Motor[LF].Data.Start_Position;
    Elevator_Motor[LB].Data.Error_Position = Elevator_Motor[LB].Data.Target_Position - Elevator_Motor[LB].Data.Start_Position;
    Elevator_Motor[RB].Data.Error_Position = Elevator_Motor[RB].Data.Target_Position - Elevator_Motor[RB].Data.Start_Position;
    Elevator_Motor[RF].Data.Error_Position = Elevator_Motor[RF].Data.Target_Position - Elevator_Motor[RF].Data.Start_Position;
}
static void Robotic_Arm_set_start_error_pos(void)
{
    Robotic_Arm_Motor[J1].Data.Start_Position = Robotic_Arm_Motor[J1].Data.Position;
    Robotic_Arm_Motor[J2].Data.Start_Position = Robotic_Arm_Motor[J2].Data.Position;
    Robotic_Arm_Motor[J3].Data.Start_Position = Robotic_Arm_Motor[J3].Data.Position;
    Robotic_Arm_Motor[J4].Data.Start_Position = Robotic_Arm_Motor[J4].Data.Position;
    Robotic_Arm_Motor[J5].Data.Start_Position = Robotic_Arm_Motor[J5].Data.Position;
    Robotic_Arm_Motor[J6].Data.Start_Position = Robotic_Arm_Motor[J6].Data.Position;

    Robotic_Arm_Motor[J1].Data.Error_Position = Robotic_Arm_Motor[J1].Data.Target_Position - Robotic_Arm_Motor[J1].Data.Start_Position;
    Robotic_Arm_Motor[J2].Data.Error_Position = Robotic_Arm_Motor[J2].Data.Target_Position - Robotic_Arm_Motor[J2].Data.Start_Position;
    Robotic_Arm_Motor[J3].Data.Error_Position = Robotic_Arm_Motor[J3].Data.Target_Position - Robotic_Arm_Motor[J3].Data.Start_Position;
    Robotic_Arm_Motor[J4].Data.Error_Position = Robotic_Arm_Motor[J4].Data.Target_Position - Robotic_Arm_Motor[J4].Data.Start_Position;
    Robotic_Arm_Motor[J5].Data.Error_Position = Robotic_Arm_Motor[J5].Data.Target_Position - Robotic_Arm_Motor[J5].Data.Start_Position;
    Robotic_Arm_Motor[J6].Data.Error_Position = Robotic_Arm_Motor[J6].Data.Target_Position - Robotic_Arm_Motor[J6].Data.Start_Position;
}

/**
 * @brief 通过位置环 PID 计算各关节 MIT 模式下的前馈力矩, 用于消除稳态误差
 * @note  - 输入误差: Temp_Target_Position - Position (rad)
 *        - 输出: PID->Output (N·m), 直接写入 Motor.Data.Feedforward
 *        - J3/J4/J6 电机正转方向与关节正转方向相反, 前馈值需取反
 */
#define GravityCompensation 1
static void Robotic_Arm_set_feedfoward(void)
{
    PID_Calculate(&Robotic_Arm_FF_PID[J1], Robotic_Arm_Motor[J1].Data.Temp_Target_Position, Robotic_Arm_Motor[J1].Data.Position);
    PID_Calculate(&Robotic_Arm_FF_PID[J2], Robotic_Arm_Motor[J2].Data.Temp_Target_Position, Robotic_Arm_Motor[J2].Data.Position);
    PID_Calculate(&Robotic_Arm_FF_PID[J3], Robotic_Arm_Motor[J3].Data.Temp_Target_Position, Robotic_Arm_Motor[J3].Data.Position);
    PID_Calculate(&Robotic_Arm_FF_PID[J4], Robotic_Arm_Motor[J4].Data.Temp_Target_Position, Robotic_Arm_Motor[J4].Data.Position);
    PID_Calculate(&Robotic_Arm_FF_PID[J5], Robotic_Arm_Motor[J5].Data.Temp_Target_Position, Robotic_Arm_Motor[J5].Data.Position);
    PID_Calculate(&Robotic_Arm_FF_PID[J6], Robotic_Arm_Motor[J6].Data.Temp_Target_Position, Robotic_Arm_Motor[J6].Data.Position);

#if !GravityCompensation
    Robotic_Arm_Motor[J1].Data.Feedforward =  Robotic_Arm_FF_PID[J1].Output;
    Robotic_Arm_Motor[J2].Data.Feedforward =  Robotic_Arm_FF_PID[J2].Output;
    Robotic_Arm_Motor[J3].Data.Feedforward =  Robotic_Arm_FF_PID[J3].Output;  /* J3 电机方向与关节相反 */
    Robotic_Arm_Motor[J4].Data.Feedforward =  Robotic_Arm_FF_PID[J4].Output;  /* J4 电机方向与关节相反 */
    Robotic_Arm_Motor[J5].Data.Feedforward =  Robotic_Arm_FF_PID[J5].Output;
    Robotic_Arm_Motor[J6].Data.Feedforward =  Robotic_Arm_FF_PID[J6].Output;  /* J6 电机方向与关节相反 */

#else
    float theta[6] = {Robotic_Arm_Motor[J1].Data.Position, Robotic_Arm_Motor[J2].Data.Position, -Robotic_Arm_Motor[J3].Data.Position, -Robotic_Arm_Motor[J4].Data.Position, Robotic_Arm_Motor[J5].Data.Position, -Robotic_Arm_Motor[J6].Data.Position};
    GravityComp_UpdateFeedforward(theta);
    Robotic_Arm_Motor[J1].Data.Feedforward =      gGravityComp.feedforward_torque[J1] + Robotic_Arm_FF_PID[J1].Output;
    Robotic_Arm_Motor[J2].Data.Feedforward =  0.4*gGravityComp.feedforward_torque[J2] + Robotic_Arm_FF_PID[J2].Output;
    Robotic_Arm_Motor[J3].Data.Feedforward =     -gGravityComp.feedforward_torque[J3] + Robotic_Arm_FF_PID[J3].Output;
    Robotic_Arm_Motor[J4].Data.Feedforward =     -gGravityComp.feedforward_torque[J4] + Robotic_Arm_FF_PID[J4].Output;
    Robotic_Arm_Motor[J5].Data.Feedforward =      gGravityComp.feedforward_torque[J5] + Robotic_Arm_FF_PID[J5].Output;
    Robotic_Arm_Motor[J6].Data.Feedforward =     -gGravityComp.feedforward_torque[J6] + Robotic_Arm_FF_PID[J6].Output;
#endif
}
/* USER CODE END Control_Task */
