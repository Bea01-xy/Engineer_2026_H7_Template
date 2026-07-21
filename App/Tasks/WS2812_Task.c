/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : WS2812_Task.c
  * @brief          : WS2812 LED task — 正常时绿灯常亮, 机械臂电机离线时红灯呼吸
  * @author         : Ported from CtrBoard-H7_WS2812
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "WS2812.h"
#include "Motor_DM.h"       /* Robotic_Arm_Motor[].Data.offline */

/**
  * @brief  WS2812 task: 监测机械臂 DM 电机在线状态
  *         全部在线 → 绿灯常亮
  *         任一离线 → 红灯呼吸 (三角波, 周期 ~3.8s)
  * @param  argument: Not used
  * @retval None
  */
void WS2812_Task(void)
{
    uint16_t i;
    uint16_t step = 0;

    WS2812_Init();

    for (;;)
    {
        /* 检测是否有机械臂电机离线 */
        bool any_offline = false;
        for (i = 0; i < 6; i++)
        {
            if (Robotic_Arm_Motor[i].Data.offline)
            {
                any_offline = true;
                break;
            }
        }

        uint8_t r, g, b;

        if (any_offline)
        {
            /* 红灯呼吸: 三角波 0→255→0 */
            uint8_t bright;
            if (step < 128)
                bright = (uint8_t)(step * 2);
            else
                bright = (uint8_t)((255 - step) * 2);

            r = bright;
            g = 0;
            b = 0;

            step += 8;
            if (step >= 256) step = 0;
        }
        else
        {
            /* 全部在线: 绿灯常亮 */
            r = 0;
            g = 255;
            b = 0;
            step = 0;   /* 复位呼吸相位, 下次离线从亮起开始 */
        }

        for (i = 0; i < WS2812_LED_NUM; i++) {
            WS2812_SetLED(i, r, g, b);
        }
        WS2812_Update();
        osDelay(15);
    }
}
