/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : UI.h
  * @brief          : 选手端图传画面 UI 绘制 (RoboMaster 2026 协议 V1.3.0)
  *                   通过 USART1 向裁判系统发送图形绘制指令
  * @author         : sanyue (适配: Engineer_2026_H7_Template)
  * @date           : 2026/04/26
  * @version        : v1.0
  ******************************************************************************
  * @attention      : 矩形 + 两条横线 (0x0103) + 居中字符 (0x0110)
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef UI_H
#define UI_H

#include "stdint.h"
#include "stdbool.h"

/*********************************************************************************************************
*                                              ui绘制参数
*  屏幕坐标: (0,0) 为屏幕左下角, (1920,1080) 为屏幕右上角
*********************************************************************************************************/
#define SCREEN_RES_WIDTH            1920U
#define SCREEN_RES_HEIGHT           1080U

#define AIM_RECTANGLE_START_X       508
#define AIM_RECTANGLE_START_Y       262
#define AIM_RECTANGLE_WIDTH         800
#define AIM_RECTANGLE_HEIGHT        600
#define AIM_RECTANGLE_LINE_WIDTH    3

/* 横线 1：矩形垂直方向正中 */
#define AIM_LINE1_START_X           AIM_RECTANGLE_START_X
#define AIM_LINE1_START_Y           (AIM_RECTANGLE_START_Y + (AIM_RECTANGLE_HEIGHT / 2))
#define AIM_LINE1_END_X            (AIM_RECTANGLE_START_X + AIM_RECTANGLE_WIDTH)
#define AIM_LINE1_END_Y             AIM_LINE1_START_Y
#define AIM_LINE1_WIDTH             AIM_RECTANGLE_LINE_WIDTH

/* 横线 2：矩形靠上约 1/4 高度处 */
#define AIM_LINE2_START_X           AIM_RECTANGLE_START_X
#define AIM_LINE2_START_Y           (AIM_RECTANGLE_START_Y + (AIM_RECTANGLE_HEIGHT * 3 / 4))
#define AIM_LINE2_END_X             AIM_LINE1_END_X
#define AIM_LINE2_END_Y             AIM_LINE2_START_Y
#define AIM_LINE2_WIDTH             AIM_RECTANGLE_LINE_WIDTH

/* 居中文字「Engineer」：表 2-27，左下角锚点 */
#define AIM_TEXT_STRING             "Engineer_26"
#define AIM_TEXT_LEN                (sizeof(AIM_TEXT_STRING) - 1U)
#define AIM_TEXT_FONT_SIZE          28U
#define AIM_TEXT_LINE_WIDTH         3U
#define AIM_TEXT_START_X            (SCREEN_RES_WIDTH / 2U - 110U)
#define AIM_TEXT_START_Y            (SCREEN_RES_HEIGHT / 2U - 14U)

/*********************************************************************************************************
*                                              裁判系统协议常量
*********************************************************************************************************/
#define REFEREE_HEADER_SOF          0xA5U
#define REFEREE_LEN_FRAME_HEAD      5U
#define REFEREE_CMD_ID_INTERACT     0x0301U
#define UI_INTERACT_ID_DRAW_ONE     0x0101U
#define UI_INTERACT_ID_DRAW_TWO     0x0102U
#define UI_INTERACT_ID_DRAW_FIVE    0x0103U   /* 表 2-25：5 个 15B 图形 */
#define UI_INTERACT_ID_DRAW_CHAR    0x0110U

#define UI_LEN_REFEREE_DATA_FIVE    (6U + 15U * 5U)   /* 0x0103: 81 */
#define UI_LEN_REFEREE_DATA_CHAR    (6U + 15U + 30U)  /* 0x0110: 51 */

/*********************************************************************************************************
*                                              图形操作 / 类型 / 颜色枚举 (协议表 1-27)
*********************************************************************************************************/
typedef enum
{
    UI_NONE     = 0,
    UI_ADD      = 1,
    UI_MODIFY   = 2,
    UI_DELETE   = 3,
} Graphic_Operate_e;

typedef enum
{
    UI_LINE         = 0,
    UI_RECTANGLE    = 1,
    UI_CIRCLE       = 2,
    UI_OVAL         = 3,
    UI_ARC          = 4,
    UI_FLOAT        = 5,
    UI_INT          = 6,
    UI_CHAR         = 7,
} Graphic_Type_e;

typedef enum
{
    UI_LAYER_0 = 0, UI_LAYER_1, UI_LAYER_2, UI_LAYER_3, UI_LAYER_4,
    UI_LAYER_5,     UI_LAYER_6, UI_LAYER_7, UI_LAYER_8, UI_LAYER_9,
} Graphic_Layer_e;

typedef enum
{
    UI_SELF_COLOR   = 0,
    UI_YELLOW       = 1,
    UI_GREEN        = 2,
    UI_ORANGE       = 3,
    UI_FUCHSIA      = 4,
    UI_PINK         = 5,
    UI_CYAN_BLUE    = 6,
    UI_BLACK        = 7,
    UI_WHITE        = 8,
} Graphic_Color_e;

/*********************************************************************************************************
*                                              数据结构
*********************************************************************************************************/
#pragma pack(1)

typedef struct
{
    uint8_t  SOF;
    uint16_t data_length;
    uint8_t  seq;
    uint8_t  CRC8;
} frame_header_t;

typedef struct
{
    uint16_t data_cmd_id;
    uint16_t sender_id;
    uint16_t receiver_id;
} interaction_header_t;

/* 0x0103：5 + 2 + 81 + 2 = 90 */
typedef struct
{
    frame_header_t       frame_header;
    uint16_t             cmd_id;
    interaction_header_t interact_header;
    uint8_t              graphic[75];
    uint16_t             frame_tail_crc16;
} ext_graphic_five_data_t;

/* 0x0110：5 + 2 + 51 + 2 = 60 */
typedef struct
{
    frame_header_t       frame_header;
    uint16_t             cmd_id;
    interaction_header_t interact_header;
    uint8_t              char_config[15];
    uint8_t              char_data[30];
    uint16_t             frame_tail_crc16;
} ext_graphic_char_data_t;

#pragma pack()

extern void UI_Tick(void);

#endif //UI_H
