/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : LCD_Task.c
  * @brief          : LCD demo task — displays static boot screen
  * @author         : Ported from CtrBoard-H7_LCD
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "lcd.h"

/**
  * @brief  LCD task: initialise display, show boot screen, then idle.
  * @param  argument: Not used
  * @retval None
  */
void LCD_Task(void)
{
    LCD_Init();

    /* 开机画面 */
    LCD_Fill(0, 0, LCD_W, LCD_H, BLACK);
    LCD_ShowString(20, 100, (const uint8_t *)"Hello World", BLUE, BLACK, 24, 0);

    /* 保持画面, 空闲等待 */
    for (;;)
    {
        osDelay(1000);
    }
}
