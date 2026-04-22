/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_cdc_if.c
  * @version        : v1.0_Cube
  * @brief          : Usb device for Virtual Com Port.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_if.h"

/* USER CODE BEGIN INCLUDE */
#include "Robotic_Arm_Config.h"
/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief Usb device library.
  * @{
  */

/** @addtogroup USBD_CDC_IF
  * @{
  */

/** @defgroup USBD_CDC_IF_Private_TypesDefinitions USBD_CDC_IF_Private_TypesDefinitions
  * @brief Private types.
  * @{
  */

/* USER CODE BEGIN PRIVATE_TYPES */

/* USER CODE END PRIVATE_TYPES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Defines USBD_CDC_IF_Private_Defines
  * @brief Private defines.
  * @{
  */

/* USER CODE BEGIN PRIVATE_DEFINES */
/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Macros USBD_CDC_IF_Private_Macros
  * @brief Private macros.
  * @{
  */

/* USER CODE BEGIN PRIVATE_MACRO */

/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Variables USBD_CDC_IF_Private_Variables
  * @brief Private variables.
  * @{
  */
/* MiniPC 数据全局实例 - 结构体定义在 usbd_cdc_if.h 中 */
MiniPC_DataTypeDef MiniPC_Data = {
    .joint_data = {J1_INITIAL_POS, J2_INITIAL_POS, J3_INITIAL_POS, J4_INITIAL_POS, J5_INITIAL_POS, J6_INITIAL_POS},
    .mouse_x = 0, .mouse_y = 0, .mouse_z = 0,
    .mouse_left = 0, .mouse_right = 0, .mouse_mid = 0,
    .key_w = 0, .key_s = 0, .key_a = 0, .key_d = 0,
    .key_shift = 0, .key_ctrl = 0, .key_q = 0, .key_e = 0,
    .key_r = 0, .key_f = 0, .key_g = 0, .key_z = 0,
    .key_x = 0, .key_c = 0, .key_v = 0, .key_b = 0
};
/* Create buffer for reception and transmission           */
/* It's up to user to redefine and/or remove those define */
/** Received data over USB are stored in this buffer      */
uint8_t UserRxBufferHS[APP_RX_DATA_SIZE];

/** Data to send over USB CDC are stored in this buffer   */
uint8_t UserTxBufferHS[APP_TX_DATA_SIZE];

/* USER CODE BEGIN PRIVATE_VARIABLES */
/* USB接收缓冲区和状态标志 */
#define USB_RX_BUFFER_SIZE  64
static uint8_t USB_RxBuffer[USB_RX_BUFFER_SIZE];
static volatile uint32_t USB_RxLength = 0;
static volatile uint8_t USB_RxReady = 0;  /* 数据包接收完成标志 */
/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_Variables USBD_CDC_IF_Exported_Variables
  * @brief Public variables.
  * @{
  */

extern USBD_HandleTypeDef hUsbDeviceHS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_FunctionPrototypes USBD_CDC_IF_Private_FunctionPrototypes
  * @brief Private functions declaration.
  * @{
  */

static int8_t CDC_Init_HS(void);
static int8_t CDC_DeInit_HS(void);
static int8_t CDC_Control_HS(uint8_t cmd, uint8_t* pbuf, uint16_t length);
int8_t CDC_Receive_HS(uint8_t* pbuf, uint32_t *Len);
static int8_t CDC_TransmitCplt_HS(uint8_t *pbuf, uint32_t *Len, uint8_t epnum);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */

/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
  * @}
  */

USBD_CDC_ItfTypeDef USBD_Interface_fops_HS =
{
  CDC_Init_HS,
  CDC_DeInit_HS,
  CDC_Control_HS,
  CDC_Receive_HS,
  CDC_TransmitCplt_HS
};

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initializes the CDC media low layer over the USB HS IP
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Init_HS(void)
{
  /* USER CODE BEGIN 8 */
  /* Set Application Buffers */
  USBD_CDC_SetTxBuffer(&hUsbDeviceHS, UserTxBufferHS, 0);
  USBD_CDC_SetRxBuffer(&hUsbDeviceHS, UserRxBufferHS);
  return (USBD_OK);
  /* USER CODE END 8 */
}

/**
  * @brief  DeInitializes the CDC media low layer
  * @param  None
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_DeInit_HS(void)
{
  /* USER CODE BEGIN 9 */
  return (USBD_OK);
  /* USER CODE END 9 */
}

/**
  * @brief  Manage the CDC class requests
  * @param  cmd: Command code
  * @param  pbuf: Buffer containing command data (request parameters)
  * @param  length: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Control_HS(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
  /* USER CODE BEGIN 10 */
  switch(cmd)
  {
  case CDC_SEND_ENCAPSULATED_COMMAND:

    break;

  case CDC_GET_ENCAPSULATED_RESPONSE:

    break;

  case CDC_SET_COMM_FEATURE:

    break;

  case CDC_GET_COMM_FEATURE:

    break;

  case CDC_CLEAR_COMM_FEATURE:

    break;

  /*******************************************************************************/
  /* Line Coding Structure                                                       */
  /*-----------------------------------------------------------------------------*/
  /* Offset | Field       | Size | Value  | Description                          */
  /* 0      | dwDTERate   |   4  | Number |Data terminal rate, in bits per second*/
  /* 4      | bCharFormat |   1  | Number | Stop bits                            */
  /*                                        0 - 1 Stop bit                       */
  /*                                        1 - 1.5 Stop bits                    */
  /*                                        2 - 2 Stop bits                      */
  /* 5      | bParityType |  1   | Number | Parity                               */
  /*                                        0 - None                             */
  /*                                        1 - Odd                              */
  /*                                        2 - Even                             */
  /*                                        3 - Mark                             */
  /*                                        4 - Space                            */
  /* 6      | bDataBits  |   1   | Number Data bits (5, 6, 7, 8 or 16).          */
  /*******************************************************************************/
  case CDC_SET_LINE_CODING:

    break;

  case CDC_GET_LINE_CODING:

    break;

  case CDC_SET_CONTROL_LINE_STATE:

    break;

  case CDC_SEND_BREAK:

    break;

  default:
    break;
  }

  return (USBD_OK);
  /* USER CODE END 10 */
}
typedef union {
  float f;
  uint8_t bytes[4];
} FloatConverter;

float bytes_to_float_union(const uint8_t *buffer) {
  FloatConverter converter;

  for(int i = 0; i < 4; i++) {
    converter.bytes[i] = buffer[i];
  }

  return converter.f;
}

void float_to_bytes_union(float value, uint8_t *buffer) {
  FloatConverter converter;
  converter.f = value;

  for(int i = 0; i < 4; i++) {
    buffer[i] = converter.bytes[i];
  }

}

/**
  * @brief Data received over USB OUT endpoint are sent over CDC interface
  *         through this function.
  *
  *         @note
  *         This function will issue a NAK packet on any OUT packet received on
  *         USB endpoint until exiting this function. If you exit this function
  *         before transfer is complete on CDC interface (ie. using DMA controller)
  *         it will result in receiving more data while previous ones are still
  *         not sent.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAILL
  */
int8_t CDC_Receive_HS(uint8_t* Buf, uint32_t *Len)
{
  /* USER CODE BEGIN 11 */
  /* 将接收到的数据复制到本地缓冲区 */
  if (*Len > 0 && *Len <= USB_RX_BUFFER_SIZE)
  {
    USB_RxLength = *Len;
    USB_RxReady = 1;  /* 标记新数据已接收 */
  }

  /* 继续接收下一个数据包 - 这才是回调函数的主要职责 */
  USBD_CDC_SetRxBuffer(&hUsbDeviceHS, UserRxBufferHS);
  USBD_CDC_ReceivePacket(&hUsbDeviceHS);

  return (USBD_OK);
  /* USER CODE END 11 */
}

/**
  * @brief  Data to send over USB IN endpoint are sent over CDC interface
  *         through this function.
  * @param  Buf: Buffer of data to be sent
  * @param  Len: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
  */
uint8_t CDC_Transmit_HS(uint8_t* Buf, uint16_t Len)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 12 */
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceHS.pClassData;
  if (hcdc->TxState != 0){
    return USBD_BUSY;
  }
  USBD_CDC_SetTxBuffer(&hUsbDeviceHS, Buf, Len);
  result = USBD_CDC_TransmitPacket(&hUsbDeviceHS);
  /* USER CODE END 12 */
  return result;
}

/**
  * @brief  CDC_TransmitCplt_HS
  *         Data transmitted callback
  *
  *         @note
  *         This function is IN transfer complete callback used to inform user that
  *         the submitted Data is successfully sent over USB.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_TransmitCplt_HS(uint8_t *Buf, uint32_t *Len, uint8_t epnum)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 14 */
  UNUSED(Buf);
  UNUSED(Len);
  UNUSED(epnum);
  /* USER CODE END 14 */
  return result;
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */
uint8_t joint_data_transmit[27] = {0};
uint8_t MiniPC_Transmit_Info(float* Buf, uint16_t Len){
    //校验位置零覆盖上一次校验位
    joint_data_transmit[Len*4+1] = 0;
    joint_data_transmit[0] = 0xAA;
    for (uint8_t i = 0; i < Len; i++)
    {
        float_to_bytes_union(Buf[i], &joint_data_transmit[1 + i * 4]);
    }
    for (uint8_t i = 1; i < Len*4+1; i++)
    {
        joint_data_transmit[Len*4+1] += joint_data_transmit[i];
    }
    joint_data_transmit[Len*4+2] = 0x55;
    return CDC_Transmit_HS(joint_data_transmit, Len*4 + 3);
}

/**
  * @brief  解析从小电脑接收的USB虚拟串口数据
  * @note   数据包格式：帧头(0xAA) + 数据(49字节) + 校验和(1字节) + 帧尾(0x55)
  *         数据布局：6个float(24B) + 3个int16_t(6B) + 19个uint8_t(19B)
  *         总长度：52字节
  * @retval None - 解析结果存入全局变量 MiniPC_Data
  */
void MiniPC_Receive_Info(void)
{
    /* 数据部分大小：6*4 + 3*2 + 19*1 = 49字节 */
    const uint32_t data_len = 49;
    /* 完整数据包：帧头(1) + 数据(49) + 校验(1) + 帧尾(1) = 52字节 */
    const uint32_t packet_len = data_len + 3;

    /* 检查是否有新数据到达 */
    if (USB_RxReady == 0)
    {
        return;  /* 无新数据，直接返回 */
    }
    USB_RxReady = 0;  /* 清除就绪标志 */

    /* 数据包长度检查 */
    if (USB_RxLength < packet_len)
    {
        return;  /* 数据长度不足 */
    }

    /* 帧头帧尾检查 */
    if (UserRxBufferHS[0] != 0xAA ||
        UserRxBufferHS[packet_len - 1] != 0x55)
    {
        return;  /* 帧格式错误 */
    }

    /* 校验和计算：累加所有数据字节 */
    uint8_t verification = 0;
    for (uint32_t i = 1; i <= data_len; i++)
    {
        verification += UserRxBufferHS[i];
    }

    if (UserRxBufferHS[packet_len - 2] != verification)
    {
        return;  /* 校验失败 */
    }

    uint8_t* pbuf = &UserRxBufferHS[1];  /* 指向数据起始位置 */
    uint32_t idx = 0;

    /* 解析6个float - 关节数据 (偏移 0-23) */
    for (uint32_t i = 0; i < 6; i++)
    {
        MiniPC_Data.joint_data[i] = bytes_to_float_union(&pbuf[idx]);
        idx += 4;
    }

    /* 解析3个int16_t - 鼠标数据 (偏移 24-29) */
    MiniPC_Data.mouse_x = (int16_t)(pbuf[idx] | (pbuf[idx + 1] << 8));
    idx += 2;
    MiniPC_Data.mouse_y = (int16_t)(pbuf[idx] | (pbuf[idx + 1] << 8));
    idx += 2;
    MiniPC_Data.mouse_z = (int16_t)(pbuf[idx] | (pbuf[idx + 1] << 8));
    idx += 2;

    /* 解析19个uint8_t - 鼠标按键和键盘按键 (偏移 30-48) */
    MiniPC_Data.mouse_left  = pbuf[idx++];
    MiniPC_Data.mouse_right = pbuf[idx++];
    MiniPC_Data.mouse_mid   = pbuf[idx++];
    MiniPC_Data.key_w       = pbuf[idx++];
    MiniPC_Data.key_s       = pbuf[idx++];
    MiniPC_Data.key_a       = pbuf[idx++];
    MiniPC_Data.key_d       = pbuf[idx++];
    MiniPC_Data.key_shift   = pbuf[idx++];
    MiniPC_Data.key_ctrl    = pbuf[idx++];
    MiniPC_Data.key_q       = pbuf[idx++];
    MiniPC_Data.key_e       = pbuf[idx++];
    MiniPC_Data.key_r       = pbuf[idx++];
    MiniPC_Data.key_f       = pbuf[idx++];
    MiniPC_Data.key_g       = pbuf[idx++];
    MiniPC_Data.key_z       = pbuf[idx++];
    MiniPC_Data.key_x       = pbuf[idx++];
    MiniPC_Data.key_c       = pbuf[idx++];
    MiniPC_Data.key_v       = pbuf[idx++];
    MiniPC_Data.key_b       = pbuf[idx++];
}
/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */

/**
  * @}
  */
