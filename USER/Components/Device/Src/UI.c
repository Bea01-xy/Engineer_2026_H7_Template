/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : UI.c
  * @brief          : 选手端图传画面 UI 绘制 (RoboMaster 2026 协议 V1.3.0)
  *                   矩形 + 两横线 (0x0103) + 两段 0x0110 字符（档位 + ENABLE/DISABLE）
  * @author         : sanyue (适配: Engineer_2026_H7_Template)
  * @date           : 2026/04/26
  * @version        : v1.0
  ******************************************************************************
  * @attention      : 后两个图形槽位填「空操作」以满足 0x0103 五槽位长度
  ******************************************************************************
  */
/* USER CODE END Header */

/*********************************************************************************************************
*                                              包含头文件
*********************************************************************************************************/
#include "UI.h"
#include "string.h"
#include "usart.h"
#include "stm32h7xx_hal_uart.h"
#include "CRC.h"
#include "Referee_System.h"
#include "Chassis_Config.h"

/*********************************************************************************************************
*                                              外部变量声明
*********************************************************************************************************/
extern Referee_System_Info_TypeDef Referee_System_Info;
extern Chassis_Info_Typedef chassis_info;

/*********************************************************************************************************
*                                              内部变量
*********************************************************************************************************/
__attribute__((section(".AXI_SRAM"))) static uint8_t ClientTxBuffer[sizeof(ext_graphic_five_data_t)];
static uint8_t ui_seq = 0;

/*********************************************************************************************************
*                                              内部函数声明
*********************************************************************************************************/
static uint16_t Get_Self_Client_ID(uint8_t robot_id);
static void     EncodeGraphic15(uint8_t g[15],
                                const uint8_t  name[3],
                                Graphic_Operate_e op,
                                Graphic_Type_e    type,
                                Graphic_Layer_e   layer,
                                Graphic_Color_e   color,
                                uint16_t width,
                                uint16_t start_x, uint16_t start_y,
                                uint16_t details_a,
                                uint16_t details_b,
                                uint16_t details_c,
                                uint16_t details_d, uint16_t details_e);
static void     UI_Pack_And_Send_Shapes(Graphic_Operate_e op);
static void     UI_Pack_And_Send_Char(Graphic_Operate_e op);
static void     UI_Pack_And_Send_ModeChar(Graphic_Operate_e op);

/*********************************************************************************************************
*                                              对外调用入口
*********************************************************************************************************/

/**
  * @brief  1ms 任务中调用；约 100ms 一发。
  *         前 5 帧 ADD 几何，再 5 帧 ADD 升降档文字，再 5 帧 ADD 模式 ENABLE/DISABLE 文字，
  *         之后按 几何 / 档字 / 模式字 轮流 MODIFY。
  */
void UI_Tick(void)
{
    static uint16_t tick_cnt = 0;
    static uint8_t  init_cnt = 0;
    static uint8_t  tx_sel    = 0;

    if (++tick_cnt < 100) {
        return;
    }
    tick_cnt = 0;

    if (Referee_System_Info.robot_status.robot_id == 0) {
        return;
    }

    if (init_cnt < 5) {
        UI_Pack_And_Send_Shapes(UI_ADD);
        init_cnt++;
    } else if (init_cnt < 10) {
        UI_Pack_And_Send_Char(UI_ADD);
        init_cnt++;
    } else if (init_cnt < 15) {
        UI_Pack_And_Send_ModeChar(UI_ADD);
        init_cnt++;
    } else {
        switch (tx_sel % 3U) {
        case 0U:
            UI_Pack_And_Send_Shapes(UI_MODIFY);
            break;
        case 1U:
            UI_Pack_And_Send_Char(UI_MODIFY);
            break;
        default:
            UI_Pack_And_Send_ModeChar(UI_MODIFY);
            break;
        }
        tx_sel++;
    }
}

/*********************************************************************************************************
*                                              ui绘制 / 发送
*********************************************************************************************************/

static uint16_t Get_Self_Client_ID(uint8_t robot_id)
{
    if (robot_id >= 1 && robot_id <= 6) {
        return (uint16_t)(0x0100 + robot_id);
    } else if (robot_id >= 101 && robot_id <= 106) {
        return (uint16_t)(0x0164 + (robot_id - 100));
    }
    return 0x0102;
}

static void EncodeGraphic15(uint8_t g[15],
                            const uint8_t  name[3],
                            Graphic_Operate_e op,
                            Graphic_Type_e    type,
                            Graphic_Layer_e   layer,
                            Graphic_Color_e   color,
                            uint16_t width,
                            uint16_t start_x, uint16_t start_y,
                            uint16_t details_a,
                            uint16_t details_b,
                            uint16_t details_c,
                            uint16_t details_d, uint16_t details_e)
{
    const uint32_t da = (uint32_t)(details_a & 0x1FFU);
    const uint32_t db = (uint32_t)(details_b & 0x1FFU);
    memcpy(g, name, 3);

    const uint32_t w0 = ((uint32_t)(op & 7U))
                        | (((uint32_t)(type & 7U)) << 3)
                        | (((uint32_t)((uint8_t)layer & 0xFU)) << 6)
                        | (((uint32_t)((uint8_t)color & 0xFU)) << 10)
                        | ((da & 0x1FFU) << 14)
                        | ((db & 0x1FFU) << 23);

    const uint32_t w1 = ((uint32_t)(width & 0x3FFU))
                        | (((uint32_t)(start_x & 0x7FFU)) << 10)
                        | (((uint32_t)(start_y & 0x7FFU)) << 21);

    const uint32_t w2 = ((uint32_t)(details_c & 0x3FFU))
                        | (((uint32_t)(details_d & 0x7FFU)) << 10)
                        | (((uint32_t)(details_e & 0x7FFU)) << 21);

    memcpy(&g[3], &w0, sizeof(w0));
    memcpy(&g[7], &w1, sizeof(w1));
    memcpy(&g[11], &w2, sizeof(w2));
}

/** 空槽位：操作类型为「空操作」，占满 0x0103 后两格 */
static void EncodeGraphic15_Nop(uint8_t g[15], const uint8_t name[3])
{
    EncodeGraphic15(g, name, UI_NONE, UI_LINE, UI_LAYER_0, UI_SELF_COLOR,
                    0, 0, 0, 0, 0, 0, 0, 0);
}

static void UI_Pack_And_Send_Shapes(Graphic_Operate_e op)
{
    ext_graphic_five_data_t pkt;

    if (HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX) {
        return;
    }

    pkt.frame_header.SOF         = REFEREE_HEADER_SOF;
    pkt.frame_header.data_length = UI_LEN_REFEREE_DATA_FIVE;
    pkt.frame_header.seq         = ui_seq++;
    pkt.frame_header.CRC8        = 0;

    pkt.cmd_id = REFEREE_CMD_ID_INTERACT;
    pkt.interact_header.data_cmd_id = UI_INTERACT_ID_DRAW_FIVE;
    pkt.interact_header.sender_id   = Referee_System_Info.robot_status.robot_id;
    pkt.interact_header.receiver_id = Get_Self_Client_ID(Referee_System_Info.robot_status.robot_id);

    {
        const uint8_t nm[3] = {'R', 'C', '1'};
        EncodeGraphic15(pkt.graphic, nm, op, UI_RECTANGLE, UI_LAYER_0, UI_SELF_COLOR,
                        RECTANGLE_LINE_WIDTH,
                        RECTANGLE_START_X, RECTANGLE_START_Y,
                        0, 0, 0,
                        (uint16_t)(RECTANGLE_START_X + RECTANGLE_WIDTH),
                        (uint16_t)(RECTANGLE_START_Y + RECTANGLE_HEIGHT));
    }
    {
        const uint8_t nm[3] = {'L', 'N', '1'};
        EncodeGraphic15(&pkt.graphic[15], nm, op, UI_LINE, UI_LAYER_0, UI_SELF_COLOR,
                        LINE1_WIDTH,
                        LINE1_START_X, LINE1_START_Y,
                        0, 0, 0,
                        LINE1_END_X, LINE1_END_Y);
    }
    {
        const uint8_t nm[3] = {'L', 'N', '2'};
        EncodeGraphic15(&pkt.graphic[30], nm, op, UI_LINE, UI_LAYER_0, UI_SELF_COLOR,
                        LINE2_WIDTH,
                        LINE2_START_X, LINE2_START_Y,
                        0, 0, 0,
                        LINE2_END_X, LINE2_END_Y);
    }
    {    
        const uint8_t nm[3] = {'L', 'N', '3'};
        EncodeGraphic15(&pkt.graphic[45], nm, op, UI_LINE, UI_LAYER_0, UI_SELF_COLOR,
                        LINE3_WIDTH,
                        LINE3_START_X, LINE3_START_Y,
                        0, 0, 0,
                        LINE3_END_X, LINE3_END_Y);
    }
    {
        const uint8_t nm[3] = {'L', 'N', '4'};
        EncodeGraphic15(&pkt.graphic[60], nm, op, UI_LINE, UI_LAYER_0, UI_SELF_COLOR,
                        LINE4_WIDTH,
                        LINE4_START_X, LINE4_START_Y,
                        0, 0, 0,
                        LINE4_END_X, LINE4_END_Y);
    }

    memcpy(ClientTxBuffer, &pkt, sizeof(pkt));
    Append_CRC8_Check_Sum(ClientTxBuffer, REFEREE_LEN_FRAME_HEAD);
    Append_CRC16_Check_Sum(ClientTxBuffer, sizeof(pkt));
    HAL_UART_Transmit_DMA(&huart1, ClientTxBuffer, sizeof(pkt));
}

static void UI_Pack_And_Send_Char(Graphic_Operate_e op)
{
    ext_graphic_char_data_t pkt;

    if (HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX) {
        return;
    }

    pkt.frame_header.SOF         = REFEREE_HEADER_SOF;
    pkt.frame_header.data_length = UI_LEN_REFEREE_DATA_CHAR;
    pkt.frame_header.seq         = ui_seq++;
    pkt.frame_header.CRC8        = 0;

    pkt.cmd_id = REFEREE_CMD_ID_INTERACT;
    pkt.interact_header.data_cmd_id = UI_INTERACT_ID_DRAW_CHAR;
    pkt.interact_header.sender_id   = Referee_System_Info.robot_status.robot_id;
    pkt.interact_header.receiver_id = Get_Self_Client_ID(Referee_System_Info.robot_status.robot_id);

    {
        const uint8_t nm[3] = {'E', 'G', 'R'};
        if (chassis_info.lift_mode == LIFT_STAGE_1) {
        EncodeGraphic15(pkt.char_config, nm, op, UI_CHAR, UI_LAYER_1, UI_YELLOW,
                        TEXT_LINE_WIDTH,
                        (uint16_t)TEXT_START_X, (uint16_t)TEXT_START_Y,
                        (uint16_t)TEXT_FONT_SIZE, (uint16_t)TEXT_LEN_UP,
                        0, 0, 0);
        }else if (chassis_info.lift_mode == LIFT_STAGE_5) {
            EncodeGraphic15(pkt.char_config, nm, op, UI_CHAR, UI_LAYER_1, UI_YELLOW,
                            TEXT_LINE_WIDTH,
                            (uint16_t)TEXT_START_X, (uint16_t)TEXT_START_Y,
                            (uint16_t)TEXT_FONT_SIZE, (uint16_t)TEXT_LEN_MID,
                            0, 0, 0);
        }else if (chassis_info.lift_mode == LIFT_STAGE_3) {
            EncodeGraphic15(pkt.char_config, nm, op, UI_CHAR, UI_LAYER_1, UI_YELLOW,
                            TEXT_LINE_WIDTH,
                            (uint16_t)TEXT_START_X, (uint16_t)TEXT_START_Y,
                            (uint16_t)TEXT_FONT_SIZE, (uint16_t)TEXT_LEN_DOWN,
                            0, 0, 0);
        }else if (chassis_info.lift_mode == LIFT_STAGE_4) {
            EncodeGraphic15(pkt.char_config, nm, op, UI_CHAR, UI_LAYER_1, UI_YELLOW,
                            TEXT_LINE_WIDTH,
                            (uint16_t)TEXT_START_X, (uint16_t)TEXT_START_Y,
                            (uint16_t)TEXT_FONT_SIZE, (uint16_t)TEXT_LEN_BRACE,
                            0, 0, 0);
        } else if (chassis_info.lift_mode == LIFT_STAGE_2) {
            EncodeGraphic15(pkt.char_config, nm, op, UI_CHAR, UI_LAYER_1, UI_YELLOW,
                            TEXT_LINE_WIDTH,
                            (uint16_t)TEXT_START_X, (uint16_t)TEXT_START_Y,
                            (uint16_t)TEXT_FONT_SIZE, (uint16_t)TEXT_LEN_CLIMB,
                            0, 0, 0);
        } else {
            EncodeGraphic15(pkt.char_config, nm, op, UI_CHAR, UI_LAYER_1, UI_YELLOW,
                            TEXT_LINE_WIDTH,
                            (uint16_t)TEXT_START_X, (uint16_t)TEXT_START_Y,
                            (uint16_t)TEXT_FONT_SIZE, (uint16_t)TEXT_LEN_UP,
                            0, 0, 0);
        }
    }
    memset(pkt.char_data, 0, sizeof(pkt.char_data));
    if (chassis_info.lift_mode == LIFT_STAGE_1) {
        memcpy(pkt.char_data, TEXT_STRING_UP, TEXT_LEN_UP);
    } else if (chassis_info.lift_mode == LIFT_STAGE_5) {
        memcpy(pkt.char_data, TEXT_STRING_MID, TEXT_LEN_MID);
    } else if (chassis_info.lift_mode == LIFT_STAGE_3) {
        memcpy(pkt.char_data, TEXT_STRING_DOWN, TEXT_LEN_DOWN);
    } else if (chassis_info.lift_mode == LIFT_STAGE_4) {
        memcpy(pkt.char_data, TEXT_STRING_BRACE, TEXT_LEN_BRACE);
    } else if (chassis_info.lift_mode == LIFT_STAGE_2) {
        memcpy(pkt.char_data, TEXT_STRING_CLIMB, TEXT_LEN_CLIMB);
    } else {
        memcpy(pkt.char_data, TEXT_STRING_UP, TEXT_LEN_UP);
    }

    memcpy(ClientTxBuffer, &pkt, sizeof(pkt));
    Append_CRC8_Check_Sum(ClientTxBuffer, REFEREE_LEN_FRAME_HEAD);
    Append_CRC16_Check_Sum(ClientTxBuffer, sizeof(pkt));
    HAL_UART_Transmit_DMA(&huart1, ClientTxBuffer, sizeof(pkt));
}

/** 第二段字符：TEXT_MODE_ENABLE / DISABLE（宏在 UI.h），图名 MO1 与 EGR 区分 */
static void UI_Pack_And_Send_ModeChar(Graphic_Operate_e op)
{
    ext_graphic_char_data_t pkt;
    const char *mode_str;
    uint16_t    mode_len;

    if (HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX) {
        return;
    }

    if (chassis_info.mode != CHASSIS_DISABLE) {
        mode_str = TEXT_MODE_ENABLE;
        mode_len = (uint16_t)TEXT_MODE_ENABLE_LEN;
    } else {
        mode_str = TEXT_MODE_DISABLE;
        mode_len = (uint16_t)TEXT_MODE_DISABLE_LEN;
    }

    pkt.frame_header.SOF         = REFEREE_HEADER_SOF;
    pkt.frame_header.data_length = UI_LEN_REFEREE_DATA_CHAR;
    pkt.frame_header.seq         = ui_seq++;
    pkt.frame_header.CRC8        = 0;

    pkt.cmd_id = REFEREE_CMD_ID_INTERACT;
    pkt.interact_header.data_cmd_id = UI_INTERACT_ID_DRAW_CHAR;
    pkt.interact_header.sender_id   = Referee_System_Info.robot_status.robot_id;
    pkt.interact_header.receiver_id = Get_Self_Client_ID(Referee_System_Info.robot_status.robot_id);

    {
        const uint8_t nm[3] = {'M', 'O', '1'};
        EncodeGraphic15(pkt.char_config, nm, op, UI_CHAR, UI_LAYER_2, UI_YELLOW,
                        TEXT_MODE_LINE_WIDTH,
                        (uint16_t)TEXT_MODE_START_X, (uint16_t)TEXT_MODE_START_Y,
                        (uint16_t)TEXT_MODE_FONT_SIZE, mode_len,
                        0, 0, 0);
    }
    memset(pkt.char_data, 0, sizeof(pkt.char_data));
    memcpy(pkt.char_data, mode_str, mode_len);

    memcpy(ClientTxBuffer, &pkt, sizeof(pkt));
    Append_CRC8_Check_Sum(ClientTxBuffer, REFEREE_LEN_FRAME_HEAD);
    Append_CRC16_Check_Sum(ClientTxBuffer, sizeof(pkt));
    HAL_UART_Transmit_DMA(&huart1, ClientTxBuffer, sizeof(pkt));
}
