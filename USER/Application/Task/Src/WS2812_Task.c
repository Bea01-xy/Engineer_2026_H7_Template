/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : WS2812_Task.c
  * @brief          : WS2812 LED demo task — breathing + flowing water loop
  * @author         : Ported from CtrBoard-H7_WS2812
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "WS2812.h"

/**
  * @brief  WS2812 task: initialise, then run breathing effect in a loop.
  *         All delays use osDelay — other tasks are not blocked.
  * @param  argument: Not used
  * @retval None
  *
  * @note   3 颗灯珠全体同色同步呼吸, 依次切换:
  *         红 → 暖橙 → 绿 → 蓝 → 青 → 紫, 循环
  */
void WS2812_Task(void)
{
    uint16_t i, step;
    uint8_t r, g, b;

    WS2812_Init();

    for (;;)
    {
        for (step = 0; step < 256; step++)
        {
            uint8_t bright;

            /* 三角波: 0 → 255 → 0 */
            if (step < 128)
                bright = (uint8_t)(step * 2);
            else
                bright = (uint8_t)((255 - step) * 2);

            /* 每 ~43 拍换一色 */
            if (step < 43)               { r = bright; g = 0;       b = 0;      }  /* 红 */
            else if (step < 85)           { r = bright; g = bright/3; b = 0;     }  /* 暖橙 */
            else if (step < 128)          { r = 0;      g = bright; b = 0;      }  /* 绿 */
            else if (step < 170)          { r = 0;      g = 0;       b = bright; }  /* 蓝 */
            else if (step < 213)          { r = 0;      g = bright/3; b = bright;}  /* 青 */
            else                          { r = bright; g = 0;       b = bright; }  /* 紫 */

            for (i = 0; i < WS2812_LED_NUM; i++) {
                WS2812_SetLED(i, r, g, b);
            }
            WS2812_Update();
            osDelay(15);
        }
    }
}
