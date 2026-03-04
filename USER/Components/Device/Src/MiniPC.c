/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : MiniPC.c
  * @brief          : MiniPC interfaces functions 
  * @author         : GarssFan Wang
  * @date           : 2025/01/22
  * @version        : v1.0
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "Minipc.h"
#include "usbd_cdc_if.h"


uint8_t joint_data_transmit[27] = {0};

uint8_t MiniPC_Transmit_Info(float* Buf, uint16_t FloatLen){
    //校验位置零覆盖上一次校验位
    joint_data_transmit[FloatLen*4+1] = 0;
    joint_data_transmit[0] = 0xAA;
    for (uint8_t i = 0; i < FloatLen; i++)
    {
        float_to_bytes_union(Buf[i], &joint_data_transmit[1 + i * 4]);
    }
    for (uint8_t i = 1; i < FloatLen*4+1; i++)
    {
        joint_data_transmit[FloatLen*4+1] += joint_data_transmit[i];
    }
    joint_data_transmit[FloatLen*4+2] = 0x55;
    return CDC_Transmit_HS(joint_data_transmit, FloatLen*4 + 3);

}

//usbd_cdc_if.c -> CDC_Receive_HS
void MiniPC_Receive_Info(uint8_t* Buff){

    CDC_Receive_HS(Buff, 0);

}