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
  * @attention      : 仅实现绘制单个矩形, 走 0x0301/0x0101 子内容
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef UI_H
#define UI_H

#include "stdint.h"
#include "stdbool.h"

/*********************************************************************************************************
*                                              ui绘制参数 (单个矩形)
*  屏幕坐标: (0,0) 为屏幕左下角, (1920,1080) 为屏幕右上角
*********************************************************************************************************/
#define AIM_RECTANGLE_START_X       508     /* 矩形左下角 x 坐标 */
#define AIM_RECTANGLE_START_Y       262     /* 矩形左下角 y 坐标 */
#define AIM_RECTANGLE_WIDTH         800     /* 矩形宽度 */
#define AIM_RECTANGLE_HEIGHT        600     /* 矩形高度 */
#define AIM_RECTANGLE_LINE_WIDTH    3       /* 矩形线宽 */

/*********************************************************************************************************
*                                              裁判系统协议常量
*********************************************************************************************************/
#define REFEREE_HEADER_SOF          0xA5U                   /* 帧头起始字节 */
#define REFEREE_LEN_FRAME_HEAD      5U                      /* 帧头长度 */
#define REFEREE_CMD_ID_INTERACT     0x0301U                 /* 机器人交互数据 cmd_id */
#define UI_INTERACT_ID_DRAW_ONE     0x0101U                 /* 选手端绘制一个图形 子内容 ID */

/* 表 1-1：data_length = cmd_id 之后 data 域长度（不含 cmd_id），0x0101 单图为 6+15=21 */
#define UI_LEN_REFEREE_DATA_AFTER_CMD_ID   21U

/*********************************************************************************************************
*                                              图形操作 / 类型 / 颜色枚举 (协议表 1-27)
*********************************************************************************************************/
typedef enum
{
    UI_NONE     = 0,    /* 空操作 */
    UI_ADD      = 1,    /* 增加 */
    UI_MODIFY   = 2,    /* 修改 */
    UI_DELETE   = 3,    /* 删除 */
} Graphic_Operate_e;

typedef enum
{
    UI_LINE         = 0,    /* 直线 */
    UI_RECTANGLE    = 1,    /* 矩形 */
    UI_CIRCLE       = 2,    /* 正圆 */
    UI_OVAL         = 3,    /* 椭圆 */
    UI_ARC          = 4,    /* 圆弧 */
    UI_FLOAT        = 5,    /* 浮点数 */
    UI_INT          = 6,    /* 整型数 */
    UI_CHAR         = 7,    /* 字符 */
} Graphic_Type_e;

typedef enum
{
    UI_LAYER_0 = 0, UI_LAYER_1, UI_LAYER_2, UI_LAYER_3, UI_LAYER_4,
    UI_LAYER_5,     UI_LAYER_6, UI_LAYER_7, UI_LAYER_8, UI_LAYER_9,
} Graphic_Layer_e;

typedef enum
{
    UI_SELF_COLOR   = 0,    /* 红/蓝 (己方颜色) */
    UI_YELLOW       = 1,
    UI_GREEN        = 2,
    UI_ORANGE       = 3,
    UI_FUCHSIA      = 4,    /* 紫红色 */
    UI_PINK         = 5,
    UI_CYAN_BLUE    = 6,    /* 青色 */
    UI_BLACK        = 7,
    UI_WHITE        = 8,
} Graphic_Color_e;

/*********************************************************************************************************
*                                              数据结构 (协议表 1-3 / 1-25 / 1-27)
*********************************************************************************************************/
#pragma pack(1)

/* 帧头: 5 字节 */
typedef struct
{
    uint8_t  SOF;           /* 0xA5 */
    uint16_t data_length;   /* data 段长度 */
    uint8_t  seq;           /* 包序号 */
    uint8_t  CRC8;          /* 帧头 CRC8 */
} frame_header_t;

/* 数据段头 (机器人交互数据 0x0301 内): 6 字节 */
typedef struct
{
    uint16_t data_cmd_id;   /* 子内容 ID, 这里为 0x0101 */
    uint16_t sender_id;     /* 发送者 ID = 自身机器人 ID */
    uint16_t receiver_id;   /* 接收者 ID, 选手端 ID */
} interaction_header_t;

/* 整包: 帧头 + cmd_id + data(交互头6+图形15) + CRC16 = 5 + 2 + 21 + 2 = 30 字节 */
typedef struct
{
    frame_header_t       frame_header;
    uint16_t             cmd_id;
    interaction_header_t interact_header;
    uint8_t              graphic[15]; /* 表 2-23，按位打包在 UI.c 内手工编码 */
    uint16_t             frame_tail_crc16;
} ext_graphic_one_data_t;

#pragma pack()

/*********************************************************************************************************
*                                              对外接口
*  在 Detect_Task 等 1ms 周期任务循环里调用 UI_Tick();
*  内部已节流 (约每 100ms 发一帧), 开机前若干帧为 ADD, 之后为 MODIFY
*********************************************************************************************************/
extern void UI_Tick(void);

#endif //UI_H
