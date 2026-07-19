/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : LCD_Task.c
  * @brief          : LCD display — servo position only
  * @note           : Shows current servo position in large text.
  *                   Servo control is in Servo_Task.c (sine wave).
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "lcd.h"
#include "Servo_Task.h"     /* 共享状态 servo_display */

/* ========================= 任务入口 ========================= */

void LCD_Task(void const *argument)
{
    (void)argument;
    LCD_Init();
    LCD_Fill(0, 0, LCD_W, LCD_H, BLACK);

    /* 标题 — 只画一次 */
    LCD_ShowString(50, 30, (const uint8_t *)"SERVO POS", CYAN, BLACK, 24, 0);

    uint16_t prev_pos = 0xFFFF;  /* 保证第一次一定刷新 */

    for (;;)
    {
        uint16_t pos = servo_display.resp_position;

        if (pos != prev_pos) {
            /* 用大号字体显示位置值 */
            LCD_Fill(30, 80, 250, 140, BLACK);           /* 清数字区域 */
            LCD_ShowIntNum(40, 90, pos, 4, GREEN, BLACK, 48);  /* 大字号显示 */

            /* 换算成角度显示 (-180° ~ +180°, 0~4095) */
            int16_t angle_deg = ((int32_t)pos * 360 / 4096) - 180;
            LCD_ShowIntNum(50, 155, angle_deg, 4, WHITE, BLACK, 24);
            LCD_ShowString(130, 155, (const uint8_t *)"deg", WHITE, BLACK, 24, 0);

            prev_pos = pos;
        }

        osDelay(50);
    }
}
