/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : UI.c
  * @brief          : 选手端图传画面 UI 绘制 (RoboMaster 2026 协议 V1.3.0)
  *                   矩形 + 两条横线 (0x0103) + 居中字符串 (0x0110)
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

/*********************************************************************************************************
*                                              外部变量声明
*********************************************************************************************************/
extern Referee_System_Info_TypeDef Referee_System_Info;

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

/*********************************************************************************************************
*                                              对外调用入口
*********************************************************************************************************/

/**
  * @brief  1ms 任务中调用；约 100ms 一发。前 5 帧 ADD 几何，再 5 帧 ADD 字符，之后交替 MODIFY。
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
    } else {
        if ((tx_sel & 1U) != 0U) {
            UI_Pack_And_Send_Char(UI_MODIFY);
        } else {
            UI_Pack_And_Send_Shapes(UI_MODIFY);
        }
        tx_sel ^= 1U;
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
                        AIM_RECTANGLE_LINE_WIDTH,
                        AIM_RECTANGLE_START_X, AIM_RECTANGLE_START_Y,
                        0, 0, 0,
                        (uint16_t)(AIM_RECTANGLE_START_X + AIM_RECTANGLE_WIDTH),
                        (uint16_t)(AIM_RECTANGLE_START_Y + AIM_RECTANGLE_HEIGHT));
    }
    {
        const uint8_t nm[3] = {'L', 'N', '1'};
        EncodeGraphic15(&pkt.graphic[15], nm, op, UI_LINE, UI_LAYER_0, UI_SELF_COLOR,
                        AIM_LINE1_WIDTH,
                        AIM_LINE1_START_X, AIM_LINE1_START_Y,
                        0, 0, 0,
                        AIM_LINE1_END_X, AIM_LINE1_END_Y);
    }
    {
        const uint8_t nm[3] = {'L', 'N', '2'};
        EncodeGraphic15(&pkt.graphic[30], nm, op, UI_LINE, UI_LAYER_0, UI_SELF_COLOR,
                        AIM_LINE2_WIDTH,
                        AIM_LINE2_START_X, AIM_LINE2_START_Y,
                        0, 0, 0,
                        AIM_LINE2_END_X, AIM_LINE2_END_Y);
    }
    /* 0x0103 固定 5 个图形槽；仅用到前 3 个，后 2 个用「空操作」占位，图名随意唯一即可 (PD = padding) */
    {
        const uint8_t nm[3] = {'P', 'D', '1'};
        EncodeGraphic15_Nop(&pkt.graphic[45], nm);
    }
    {
        const uint8_t nm[3] = {'P', 'D', '2'};
        EncodeGraphic15_Nop(&pkt.graphic[60], nm);
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
        EncodeGraphic15(pkt.char_config, nm, op, UI_CHAR, UI_LAYER_1, UI_PINK,
                        AIM_TEXT_LINE_WIDTH,
                        (uint16_t)AIM_TEXT_START_X, (uint16_t)AIM_TEXT_START_Y,
                        (uint16_t)AIM_TEXT_FONT_SIZE, (uint16_t)AIM_TEXT_LEN,
                        0, 0, 0);
    }
    memset(pkt.char_data, 0, sizeof(pkt.char_data));
    memcpy(pkt.char_data, AIM_TEXT_STRING, AIM_TEXT_LEN);

    memcpy(ClientTxBuffer, &pkt, sizeof(pkt));
    Append_CRC8_Check_Sum(ClientTxBuffer, REFEREE_LEN_FRAME_HEAD);
    Append_CRC16_Check_Sum(ClientTxBuffer, sizeof(pkt));
    HAL_UART_Transmit_DMA(&huart1, ClientTxBuffer, sizeof(pkt));
}
