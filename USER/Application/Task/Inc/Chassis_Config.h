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
#define M3508_FF_SPEED_COEF 0.005f
#define M3508_FF_ACCEL_COEF 120.0f

#define M3508_OUTPUT_LIMIT 15000.0f
#define M3508_I_OUT_LIMIT 300.0f
#define M3508_KP 15.0f
#define M3508_KI 0.1f
#define M3508_KD 0.04f
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

    AUTO_LIFT_STAGE_1,
    AUTO_LIFT_STAGE_2,
    AUTO_LIFT_STAGE_3,
} Chassis_Mode_e;

typedef struct
{
    Chassis_Mode_e mode;
    Chassis_Mode_e last_mode;
    Chassis_Mode_e lift_mode;

    float target_vx;
    float target_vy;
    float target_vw;

    float target_direction;

    bool activated_flag;
} Chassis_Info_Typedef;

/* Exported functions prototypes ---------------------------------------------*/

#endif /* CHASSIS_CONFIG_H */
