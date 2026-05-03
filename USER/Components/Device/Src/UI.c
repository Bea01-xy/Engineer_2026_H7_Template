/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : UI.c
  * @brief          : 选手端图传画面 UI 绘制 (RoboMaster 2026 协议 V1.3.0)
  *                   通过 USART1 向裁判系统发送一个矩形, 走 0x0301/0x0101 链路
  * @author         : sanyue (适配: Engineer_2026_H7_Template)
  * @date           : 2026/04/26
  * @version        : v1.0
  ******************************************************************************
  * @attention      : 仅保留绘制单个矩形的最简功能
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
/* DMA 发送缓存 (放到 AXI_SRAM, 与项目其它 DMA 缓存保持一致) */
__attribute__((section(".AXI_SRAM"))) static uint8_t ClientTxBuffer[sizeof(ext_graphic_one_data_t)];
static uint8_t ui_seq = 0;  /* 包序号 */

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
                                uint16_t details_c,
                                uint16_t details_d, uint16_t details_e);
static void     UI_Pack_And_Send(Graphic_Operate_e op);

/*********************************************************************************************************
*                                              对外调用入口
*********************************************************************************************************/

/**
  * @brief  挂在 1ms 周期任务里 (如 Detect_Task), 内部自动节流 + 自动首发
  *         约每 100ms 发一帧, 开机前 5 帧用 UI_ADD 创建图形, 之后用 UI_MODIFY 刷新
  */
void UI_Tick(void)
{
    static uint16_t tick_cnt = 0;
    static uint8_t  init_cnt = 0;

    /* 100ms 节流 (假定每 1ms 调用一次) */
    if (++tick_cnt < 100) {
        return;
    }
    tick_cnt = 0;

    /* 未收到 0x0201 机器人状态前 robot_id 为 0，此时发 UI 会落到默认客户端 ID，应等裁判机上报 */
    if (Referee_System_Info.robot_status.robot_id == 0) {
        return;
    }

    if (init_cnt < 5) {
        UI_Pack_And_Send(UI_ADD);
        init_cnt++;
    } else {
        UI_Pack_And_Send(UI_MODIFY);
    }
}

/*********************************************************************************************************
*                                              ui绘制 / 发送函数
*********************************************************************************************************/

/**
  * @brief 根据本机器人 ID 推算对应选手端 ID (协议附录二)
  *        红方: 1~6 -> 0x0101~0x0106
  *        蓝方: 101~106 -> 0x0165~0x016A
  */
static uint16_t Get_Self_Client_ID(uint8_t robot_id)
{
    if (robot_id >= 1 && robot_id <= 6) {
        return (uint16_t)(0x0100 + robot_id);
    } else if (robot_id >= 101 && robot_id <= 106) {
        return (uint16_t)(0x0164 + (robot_id - 100));
    }
    /* 未知 ID 缺省按 红方工程 (engineer = 2 -> 0x0102) */
    return 0x0102;
}

/**
  * @brief 表 2-23：15 字节图形数据，按小端 uint32 三词手工打包（与协议 interaction_figure_t 一致）
  */
static void EncodeGraphic15(uint8_t g[15],
                              const uint8_t  name[3],
                              Graphic_Operate_e op,
                              Graphic_Type_e    type,
                              Graphic_Layer_e   layer,
                              Graphic_Color_e   color,
                              uint16_t width,
                              uint16_t start_x, uint16_t start_y,
                              uint16_t details_c,
                              uint16_t details_d, uint16_t details_e)
{
    const uint32_t da = 0U;
    const uint32_t db = 0U;
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

/**
  * @brief 组装一帧 0x0301/0x0101 数据并通过 USART1 DMA 发送
  *        op = UI_ADD 用于首次创建, UI_MODIFY 用于后续刷新
  */
static void UI_Pack_And_Send(Graphic_Operate_e op)
{
    ext_graphic_one_data_t pkt;

    /* 上一帧 USART1 TX DMA 未完成则跳过，避免 HAL_BUSY / 冲缓存 */
    if (HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX) {
        return;
    }

    /* 1) 帧头 */
    pkt.frame_header.SOF         = REFEREE_HEADER_SOF;
    pkt.frame_header.data_length = UI_LEN_REFEREE_DATA_AFTER_CMD_ID;
    pkt.frame_header.seq         = ui_seq++;
    pkt.frame_header.CRC8        = 0;     /* 占位, 后面统一计算 */

    /* 2) cmd_id = 0x0301 */
    pkt.cmd_id = REFEREE_CMD_ID_INTERACT;

    /* 3) 数据段头 */
    pkt.interact_header.data_cmd_id = UI_INTERACT_ID_DRAW_ONE;
    pkt.interact_header.sender_id   = Referee_System_Info.robot_status.robot_id;
    pkt.interact_header.receiver_id = Get_Self_Client_ID(Referee_System_Info.robot_status.robot_id);

    /* 4) 图形内容: 矩形 ("RC1")，对角顶点见表 2-23「矩形」行 */
    const uint8_t name[3] = {'R', 'C', '1'};
    EncodeGraphic15(pkt.graphic, name, op, UI_RECTANGLE, UI_LAYER_0, UI_SELF_COLOR,
                    AIM_RECTANGLE_LINE_WIDTH,
                    AIM_RECTANGLE_START_X,
                    AIM_RECTANGLE_START_Y,
                    0,
                    (uint16_t)(AIM_RECTANGLE_START_X + AIM_RECTANGLE_WIDTH),
                    (uint16_t)(AIM_RECTANGLE_START_Y + AIM_RECTANGLE_HEIGHT));

    /* 5) 拷贝到发送缓冲区 */
    memcpy(ClientTxBuffer, &pkt, sizeof(pkt));

    /* 6) 帧头 CRC8 */
    Append_CRC8_Check_Sum(ClientTxBuffer, REFEREE_LEN_FRAME_HEAD);

    /* 7) 整包 CRC16 (放在最后两字节) */
    Append_CRC16_Check_Sum(ClientTxBuffer, sizeof(pkt));

    /* 8) USART1 DMA 发送 */
    HAL_UART_Transmit_DMA(&huart1, ClientTxBuffer, sizeof(pkt));
}
