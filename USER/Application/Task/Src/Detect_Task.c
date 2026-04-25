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
#include "Robotic_Arm_Config.h"
#include <stdint.h>
#include "Referee_System.h"
/* USER CODE BEGIN Header_Detect_Task */
static void chassis_set_mode(Chassis_Info_Typedef* chassis);
static void chassis_ctrl_info_get(void);
static void chassis_wheel_cal(void);
static void MiniPC_Transmit_Robotic_Arm_Info(void);
static void arm_ctrl_info_get(void);

extern Chassis_Info_Typedef chassis_info;

static float Chassis_Direction_PID_Param[PID_PARAMETER_NUM] = {
    CHASSIS_DIRECTION_KP,
    CHASSIS_DIRECTION_KI,
    CHASSIS_DIRECTION_KD,
    CHASSIS_DIRECTION_Alpha,
    CHASSIS_DIRECTION_Deadband,
    CHASSIS_DIRECTION_LimitIntegral,
    CHASSIS_DIRECTION_LimitOutput
};
PID_Info_TypeDef Chassis_Direction_PID;

extern Referee_System_Info_TypeDef Referee_System_Info;

#define KEYBOARD_CTL 0

Hand_State_e hand_state = HAND_OPEN;
TickType_t Detect_Task_SysTick = 0;
/**
* @brief Function implementing the StartDetectTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Detect_Task */
void Detect_Task(void)
{
    /* USER CODE BEGIN Detect_Task */
    PID_Init(&Chassis_Direction_PID,PID_POSITION,Chassis_Direction_PID_Param);
    /* Infinite loop */
    for(;;)
    {
        Detect_Task_SysTick = osKernelSysTick();
        Remote_Message_Moniter(&remote_ctrl);
        MiniPC_Receive_Info();  /* 接收小电脑发送的6个float关节数据 */
        MiniPC_Transmit_Robotic_Arm_Info();

        chassis_set_mode(&chassis_info);
        chassis_ctrl_info_get();
        chassis_wheel_cal();

        arm_ctrl_info_get();

        /* ========== VOFA 高速发送裁判系统关键信息（1kHz，用于验证数据接收）========== */
        /* 构建裁判系统调试数据包 */
        float referee_debug_data[9] = {
            /* 数据包1: 机器人状态 (0x0201) */
            (float)Referee_System_Info.robot_status.current_HP,           /* [0] 当前血量 */
            (float)Referee_System_Info.robot_status.maximum_HP,           /* [1] 最大血量 */
            (float)Referee_System_Info.robot_status.chassis_power_limit,  /* [2] 底盘功率限制(W) */

            /* 数据包2: 功率热量数据 (0x0202) */
            (float)Referee_System_Info.power_heat_data.buffer_energy,             /* [3] 缓冲能量(J) */
            (float)Referee_System_Info.power_heat_data.shooter_17mm_1_barrel_heat, /* [4] 17mm发射机构热量 */
            (float)Referee_System_Info.power_heat_data.shooter_42mm_barrel_heat,   /* [5] 42mm发射机构热量 */

            /* 数据包3: 增益数据 (0x0204) - V1.3.0 协议已修正 */
            (float)Referee_System_Info.buff.cooling_buff,      /* [6] 热量冷却增益(直接值) */
            (float)Referee_System_Info.buff.attack_buff,       /* [7] 攻击增益(百分比) */
            (float)Referee_System_Info.buff.remaining_energy     /* [8] 能量状态位 */
        };
        //USART_Vofa_SendFloat(referee_debug_data, 9);

        /* ========== 键盘按键状态调试（测试MiniPC接收）========== */
        float key_debug_data[5] = {
            (float)chassis_info.target_vx,       /* [0] 底盘X轴速度 */
            (float)chassis_info.target_vy,       /* [1] 底盘Y轴速度 */
            (float)chassis_info.target_vw,       /* [2] 底盘转向速度 */
            (float)MiniPC_Data.key_1,       /* [3] 底盘方向 */
            (float)MiniPC_Data.key_2,       /* [4] 底盘方向 */
        };
        USART_Vofa_SendFloat(key_debug_data, 5);
        /* ========================================================= */
        osDelayUntil(&Detect_Task_SysTick, 1);
    }
    /* USER CODE END Detect_Task */
}

static void chassis_set_mode(Chassis_Info_Typedef* chassis)
{
    if(chassis == NULL)
        return;
#if KEYBOARD_CTL
    if(MiniPC_Data.key_r) HAL_NVIC_SystemReset();
#else
    // if(remote_ctrl.rc_lost)
    // {
    //     chassis->last_mode = chassis->mode;
    //     chassis->mode = CHASSIS_DISABLE;
    //     chassis->last_lift_mode = chassis->lift_mode;
    //     chassis->lift_mode = LIFT_STAGE_1;
    //     return;
    // }
    if(remote_ctrl.rc.ch[5] <= -630) HAL_NVIC_SystemReset();

    const uint16_t s1 = remote_ctrl.rc.s[1];
    const uint16_t s0 = remote_ctrl.rc.s[0];
    const uint16_t sw0 = remote_ctrl.rc.sw[0];
    const uint16_t sw1 = remote_ctrl.rc.sw[1];

    if (switch_is_up(sw1)) {
        hand_state = HAND_OPEN;
        M2006_Gripper_Motor.Data.Target_Angle = GRIPPER_OPEN_POS;
    } else {
        hand_state = HAND_CLOSE;
        M2006_Gripper_Motor.Data.Target_Angle = GRIPPER_CLOSE_POS;
    }

    if (switch_is_up(sw0)) {
        chassis->last_mode = chassis->mode;
        chassis->mode = CHASSIS_DISABLE;
    } else if (remote_ctrl.rc.ch[4] >= 630) {
        chassis->last_mode = chassis->mode;
        chassis->last_lift_mode = chassis->lift_mode;
        chassis->mode = CHASSIS_LIFT;
        chassis->lift_mode = LIFT_STAGE_4;
    }
    else if (switch_is_up(s1)) {
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
        chassis->mode = CHASSIS_AUTO_LIFT;
        if(chassis->last_mode == CHASSIS_LIFT) {
            chassis->lift_mode = LIFT_STAGE_1;
        }
    }
#endif
}

static void chassis_ctrl_info_get(void)
{
#if KEYBOARD_CTL
    chassis_info.target_vx = (float)MiniPC_Data.key_w * 3 - (float)MiniPC_Data.key_s * 3;
    chassis_info.target_vy = (float)MiniPC_Data.key_a * 3 - (float)MiniPC_Data.key_d * 3;
    chassis_info.target_vw = (float)MiniPC_Data.mouse_x * 0.1f;
    Single_Angle_PID_Calculate(&Chassis_Direction_PID, chassis_info.target_direction, INS_Info.Yaw_Angle);
    chassis_info.target_vw += Chassis_Direction_PID.Output;

    chassis_info.target_direction += (float)MiniPC_Data.mouse_x * 0.1f * 0.058f;
    chassis_info.target_direction = F_Loop_Constrain(chassis_info.target_direction, -180.0f, 180.0f);
#else
    RC_CH_APPLY_DEADBAND(remote_ctrl.rc.ch[0]);
    RC_CH_APPLY_DEADBAND(remote_ctrl.rc.ch[2]);
    RC_CH_APPLY_DEADBAND(remote_ctrl.rc.ch[3]);

    chassis_info.target_vx = (float)remote_ctrl.rc.ch[3] * RC_TO_VX;
    chassis_info.target_vy = (float)remote_ctrl.rc.ch[2] * RC_TO_VY;
    chassis_info.target_vw = (float)remote_ctrl.rc.ch[0] * RC_TO_VW * 0.8f;
    Single_Angle_PID_Calculate(&Chassis_Direction_PID, chassis_info.target_direction, INS_Info.Yaw_Angle);
    chassis_info.target_vw += Chassis_Direction_PID.Output;

    chassis_info.target_direction += remote_ctrl.rc.ch[0] * RC_TO_VW * 0.058f; //integrate to get direction
    chassis_info.target_direction = F_Loop_Constrain(chassis_info.target_direction, -180.0f, 180.0f);
#endif
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

static void MiniPC_Transmit_Robotic_Arm_Info(void)
{
    float J1_Pos = Robotic_Arm_Motor[J1].Data.Position;
    float J2_Pos = Robotic_Arm_Motor[J2].Data.Position;
    float J3_Pos = Robotic_Arm_Motor[J3].Data.Position;
    float J4_Pos = Robotic_Arm_Motor[J4].Data.Position;
    float J5_Pos = Robotic_Arm_Motor[J5].Data.Position;
    float J6_Pos = -Robotic_Arm_Motor[J6].Data.Position;

    float J1_Vel = Robotic_Arm_Motor[J1].Data.Velocity;
    float J2_Vel = Robotic_Arm_Motor[J2].Data.Velocity;
    float J3_Vel = Robotic_Arm_Motor[J3].Data.Velocity;
    float J4_Vel = Robotic_Arm_Motor[J4].Data.Velocity;
    float J5_Vel = Robotic_Arm_Motor[J5].Data.Velocity;
    float J6_Vel = -Robotic_Arm_Motor[J6].Data.Velocity;

    float Robotic_Arm_Info[12] = {J1_Pos, J2_Pos, J3_Pos, J4_Pos, J5_Pos, J6_Pos, J1_Vel, J2_Vel, J3_Vel, J4_Vel, J5_Vel, J6_Vel};
    MiniPC_Transmit_Info(Robotic_Arm_Info, 12);
}

static void arm_ctrl_info_get(void)
{
    Robotic_Arm_Motor[J1].Data.Target_Position = MiniPC_Data.joint_data[J1];
    Robotic_Arm_Motor[J2].Data.Target_Position = MiniPC_Data.joint_data[J2];
    Robotic_Arm_Motor[J3].Data.Target_Position = MiniPC_Data.joint_data[J3];
    Robotic_Arm_Motor[J4].Data.Target_Position = MiniPC_Data.joint_data[J4];
    Robotic_Arm_Motor[J5].Data.Target_Position = MiniPC_Data.joint_data[J5];
    Robotic_Arm_Motor[J6].Data.Target_Position = -MiniPC_Data.joint_data[J6];
}

/**
  * @brief  获取裁判系统数据示例（从全局 Referee_System_Info 读取）
  * @note   裁判系统数据由 UART 中断自动解析更新，此处直接读取即可
  * @retval None
  */
__attribute__((unused)) static void referee_data_usage_example(void)
{
    /* 获取机器人状态 (0x0201) */
    uint8_t robot_id = Referee_System_Info.robot_status.robot_id;
    uint16_t current_hp = Referee_System_Info.robot_status.current_HP;
    uint16_t max_hp = Referee_System_Info.robot_status.maximum_HP;
    uint16_t chassis_power_limit = Referee_System_Info.robot_status.chassis_power_limit;

    /* 获取功率热量数据 (0x0202) - 用于功率控制 */
    uint16_t buffer_energy = Referee_System_Info.power_heat_data.buffer_energy;
    uint16_t shooter_heat = Referee_System_Info.power_heat_data.shooter_17mm_1_barrel_heat;

    /* 获取增益数据 (0x0204) - V1.3.0 协议已修正解析 */
    uint16_t cooling_buff = Referee_System_Info.buff.cooling_buff;  /* 热量冷却增益 (直接值) */
    uint16_t attack_buff = Referee_System_Info.buff.attack_buff;      /* 攻击增益 (百分比) */
    uint8_t remaining_energy = Referee_System_Info.buff.remaining_energy; /* 能量状态位 */

    /* 获取弹丸剩余量 (0x0208) - V1.3.0 协议已新增 fortress 字段 */
    uint16_t allowance_17mm = Referee_System_Info.projectile_allowance.projectile_allowance_17mm;
    uint16_t fortress_allowance = Referee_System_Info.projectile_allowance.projectile_allowance_fortress; /* NEW V1.3.0 */

    /* 获取比赛状态 (0x0001) */
    uint8_t game_progress = Referee_System_Info.game_status.game_progress;
    uint16_t stage_remain_time = Referee_System_Info.game_status.stage_remain_time;

    /* 抑制未使用变量警告 */
    (void)robot_id; (void)current_hp; (void)max_hp; (void)chassis_power_limit;
    (void)buffer_energy; (void)shooter_heat;
    (void)cooling_buff; (void)attack_buff; (void)remaining_energy;
    (void)allowance_17mm; (void)fortress_allowance;
    (void)game_progress; (void)stage_remain_time;

    /* 示例：血量过低预警（可通过VOFA或LED提示）
     * if (current_hp < max_hp * 0.2f) {
     *     // 低血量警告
     * }
     */

    /* 示例：功率限制计算
     * float power_limit_ratio = chassis_power_limit / 150.0f; // 归一化到最大值
     */

    /* 示例：发送裁判系统数据到VOFA调试（可选）
     * float debug_data[3] = {(float)current_hp, (float)buffer_energy, (float)shooter_heat};
     * USART_Vofa_SendFloat(debug_data, 3);
     */
}

/**
  * @brief  扩展裁判系统VOFA调试函数 - 可自定义发送更多数据
  * @note   调用此函数发送扩展的裁判系统信息到VOFA
  *         使用 JustFloat 协议，帧尾自动添加 (0x00 0x00 0x80 0x7F)
  * @retval None
  */
__attribute__((unused)) static void referee_data_vofa_debug_extended(void)
{
    /* 方案1: 发送比赛状态 + 机器人状态 (5个float) */
    float debug_packet_1[5] = {
        (float)Referee_System_Info.game_status.game_progress,     /* 比赛阶段 0-5 */
        (float)Referee_System_Info.game_status.stage_remain_time, /* 剩余时间(秒) */
        (float)Referee_System_Info.robot_status.robot_id,         /* 机器人ID */
        (float)Referee_System_Info.robot_status.current_HP,       /* 当前血量 */
        (float)Referee_System_Info.robot_status.maximum_HP        /* 最大血量 */
    };
    USART_Vofa_SendFloat_Block(debug_packet_1, 5);

    /* 方案2: 发送功率系统详细数据 (6个float) */
    float debug_packet_2[6] = {
        (float)Referee_System_Info.robot_status.chassis_power_limit,           /* 功率限制 */
        (float)Referee_System_Info.power_heat_data.buffer_energy,              /* 缓冲能量 */
        (float)Referee_System_Info.power_heat_data.shooter_17mm_1_barrel_heat, /* 17mm热量1 */
        (float)Referee_System_Info.power_heat_data.shooter_17mm_2_barrel_heat, /* 17mm热量2 */
        (float)Referee_System_Info.power_heat_data.shooter_42mm_barrel_heat,   /* 42mm热量 */
        (float)Referee_System_Info.buff.cooling_buff                            /* 冷却增益 */
    };
    USART_Vofa_SendFloat_Block(debug_packet_2, 6);

    /* 方案3: 发送弹丸信息 - V1.3.0新增堡垒储备弹量 (4个float) */
    float debug_packet_3[4] = {
        (float)Referee_System_Info.projectile_allowance.projectile_allowance_17mm,     /* 17mm允许发弹量 */
        (float)Referee_System_Info.projectile_allowance.projectile_allowance_42mm,     /* 42mm允许发弹量 */
        (float)Referee_System_Info.projectile_allowance.remaining_gold_coin,             /* 剩余金币 */
        (float)Referee_System_Info.projectile_allowance.projectile_allowance_fortress   /* 堡垒储备弹量 - NEW V1.3.0 */
    };
    USART_Vofa_SendFloat_Block(debug_packet_3, 4);
}

