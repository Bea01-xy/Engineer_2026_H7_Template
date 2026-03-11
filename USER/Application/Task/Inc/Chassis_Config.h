/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Chassis_Config.h
  * @brief          : Configuration parameters for the robot chassis
  * @author         : Wang Xinyuan
  * @date           : 2026/03/07
  * @version        : v1.0
  ******************************************************************************
  * @attention      : To be perfected
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CHASSIS_CONFIG_H
#define CHASSIS_CONFIG_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdbool.h"
#include "math.h"

/* Exported constants --------------------------------------------------------*/

/** * @brief Chassis wheel order definition*/
#define LF 0u
#define LB 1u
#define RB 2u
#define RF 3u

/** * @brief Remote control channel max value*/
#define MAX_RC_CH_VALUE 660.0f

#define MAX_CHASSIS_VX_SPEED 3.0f
#define MAX_CHASSIS_VY_SPEED 3.0f
#define MAX_CHASSIS_VW_SPEED 3.0f

#define RC_TO_VX  (MAX_CHASSIS_VX_SPEED/MAX_RC_CH_VALUE)
#define RC_TO_VY  (MAX_CHASSIS_VY_SPEED/-MAX_RC_CH_VALUE)
#define RC_TO_VW  (MAX_CHASSIS_VW_SPEED/-MAX_RC_CH_VALUE)    //MAX_CHASSIS_VR_SPEED / RC_MAX_VALUE

#define ROTATE_RATIO 0.6f
#define WHEEL_RPM_RATIO 2000.0f
#define CHASSIS_FF_SPEED_COEF 0.005f
#define CHASSIS_FF_ACCEL_COEF 120.0f

#define CHASSIS_OUTPUT_LIMIT 15000.0f
#define CHASSIS_I_OUT_LIMIT 300.0f
#define CHASSIS_KP 15.0f
#define CHASSIS_KI 0.1f
#define CHASSIS_KD 0.04f
#define CHASSIS_Alpha 0.5f
#define CHASSIS_Deadband 2.0f
#define CHASSIS_LimitIntegral 2000.0f
#define CHASSIS_LimitOutput 15000.0f
/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP                ((uint16_t)1)
#define RC_SW_MID               ((uint16_t)3)
#define RC_SW_DOWN              ((uint16_t)2)
#define switch_is_down(s)       (s == RC_SW_DOWN)
#define switch_is_mid(s)        (s == RC_SW_MID)
#define switch_is_up(s)         (s == RC_SW_UP)
/* Exported macros -----------------------------------------------------------*/

/**
  * @brief Limit value macro
  */
#ifndef VAL_LIMIT
#define VAL_LIMIT(x, min, max) do { \
    if ((x) > (max)) { (x) = (max); } \
    else if ((x) < (min)) { (x) = (min); } \
} while (0U)
#endif

/* Exported types ------------------------------------------------------------*/
typedef enum
{
    CHASSIS_ONLY,
    CHASSIS_DISABLE,
    CHASSIS_LIFT,
} Chassis_Mode_e;

typedef enum
{
    LIFT_STAGE_1,
    LIFT_STAGE_2,
    LIFT_STAGE_3,
    LIFT_STAGE_4,
    LIFT_STAGE_5,
    LIFT_STAGE_6,
} Chassis_LIFT_Mode_e;

typedef struct
{
    Chassis_Mode_e mode;
    Chassis_Mode_e last_mode;
    Chassis_LIFT_Mode_e lift_mode;
    Chassis_LIFT_Mode_e last_lift_mode;

    float target_vx;
    float target_vy;
    float target_vw;

    float target_direction;

    bool activated_flag;
} Chassis_Info_Typedef;

/* @brief Elevator (DM6006) parameters*/
#define MIT_NO_USE 0u
#define LIFTING_TIME 800u //ms
#define ELEVATOR_KP 2.f
#define ELEVATOR_FEEDFORWARD_FOR_LB_RF 0.97f
#define ELEVATOR_FEEDFORWARD_FOR_LF_RB -0.97f

#define ELEVATOR_USUAL_POS 0.f

#define ELEVATOR_LF_1st_ACTIVATED_POS -9.44f
#define ELEVATOR_LB_1st_ACTIVATED_POS 9.37f
#define ELEVATOR_RB_1st_ACTIVATED_POS -9.40f
#define ELEVATOR_RF_1st_ACTIVATED_POS 9.40f

#define ELEVATOR_LF_2nd_ACTIVATED_POS (ELEVATOR_LF_1st_ACTIVATED_POS*0.75f)
#define ELEVATOR_LB_2nd_ACTIVATED_POS (ELEVATOR_LB_1st_ACTIVATED_POS*0.75f)
#define ELEVATOR_RB_2nd_ACTIVATED_POS (ELEVATOR_RB_1st_ACTIVATED_POS*0.75f)
#define ELEVATOR_RF_2nd_ACTIVATED_POS (ELEVATOR_RF_1st_ACTIVATED_POS*0.75f)
#endif /* CHASSIS_CONFIG_H */
