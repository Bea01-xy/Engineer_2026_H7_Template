/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : MiniPC.h
  * @brief          : MiniPC interfaces functions 
  * @author         : GrassFan Wang
  * @date           : 2025/02/10
  * @version        : v1.0
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DEVICE_MINIPC_H
#define DEVICE_MINIPC_H


/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdbool.h" 


extern uint8_t MiniPC_Transmit_Info(float* Buf, uint16_t Len);

extern void MiniPC_Receive_Info(uint8_t* Buff, uint32_t Len);
#endif
