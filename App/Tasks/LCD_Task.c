/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : LCD_Task.c
  * @brief          : LCD display — servo debug & auto-test
  * @note           : Displays real-time servo status on screen and runs
  *                   an automatic phase sequence to help diagnose why the
  *                   servo isn't moving.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "lcd.h"
#include "Servo.h"
#include <string.h>

/* ========================= 调试状态 ========================= */
typedef struct {
    uint8_t  id;               /* 应答舵机 ID */
    uint8_t  status;           /* 最后一次收到的状态字节 */
    uint16_t position;         /* 最后一次读到的位置 */
    uint32_t last_resp_tick;   /* 最后一次收到应答的 SysTick */
    uint32_t last_cmd_tick;    /* 最后一次发指令的 SysTick */
    bool     has_response;     /* 是否收到过有效应答 */
    uint8_t  phase;            /* 当前测试阶段 (0~7) */
    uint32_t phase_start;      /* 当前阶段进入时的 SysTick */
} ServoDebugInfo;

static ServoDebugInfo sv = {0};

/* ========================= 阶段定义 ========================= */
#define PHASE_WAIT_MS   1500    /* 每个阶段持续时间 (ms) */
#define PHASE_PING_MS   1000    /* PING 等应答超时 (ms) */

static const char *phase_desc[] = {
    "Init mode+torque",         /* 0 */
    ">> PING servo...",         /* 1 — 先发 PING 测连通 */
    "SetPos ->  0(2048)",       /* 2 */
    "SetPos ->+90(3072)",       /* 3 */
    "SetPos ->-90(1024)",       /* 4 */
    "ReadPosition",             /* 5 */
    "SetPos ->  0(2048)",       /* 6 */
    "Done, idle",               /* 7 */
};

/* ========================= 回调 ========================= */

static void Servo_DbgCallback(const Servo_ResponsePacket *resp)
{
    sv.id = resp->id;
    sv.status = resp->status;
    sv.last_resp_tick = osKernelSysTick();
    sv.has_response = true;

    /* READ 应答的参数就是位置值 (2 字节, 大端) */
    if (resp->param_len == 2) {
        sv.position = (uint16_t)(resp->params[0] << 8) | resp->params[1];
    }
}

/* ========================= 显示工具 ========================= */

/* 显示 1 字节十六进制 (如 "1A") */
static void ShowHex8(uint16_t x, uint16_t y, uint8_t val, uint16_t color, uint8_t size)
{
    char s[3];
    const char hex[] = "0123456789ABCDEF";
    s[0] = hex[(val >> 4) & 0x0F];
    s[1] = hex[val  & 0x0F];
    s[2] = '\0';
    LCD_ShowString(x, y, (const uint8_t *)s, color, BLACK, size, 0);
}

/* 显示一行十六进制帧 (最多显示 n 字节) */
static void ShowFrameHex(uint16_t x, uint16_t y, const uint8_t *buf, uint8_t len, uint8_t n, uint16_t color, uint8_t size)
{
    char line[32];
    uint8_t pos = 0;
    uint8_t show = (len < n) ? len : n;
    for (uint8_t i = 0; i < show && pos < sizeof(line) - 3; i++) {
        const char hex[] = "0123456789ABCDEF";
        line[pos++] = hex[(buf[i] >> 4) & 0x0F];
        line[pos++] = hex[buf[i] & 0x0F];
        line[pos++] = ' ';
    }
    if (pos > 0) pos--;  /* 去掉最后一个空格 */
    line[pos] = '\0';
    LCD_ShowString(x, y, (const uint8_t *)line, color, BLACK, size, 0);
}

/* 将 status 字节解码为错误字符串 */
static const char *StatusToStr(uint8_t st)
{
    if (st == 0)          return "ok";
    if (st & (1<<4))      return "STALL";
    if (st & (1<<3))      return "OVERCUR";
    if (st & (1<<2))      return "OVERTEMP";
    if (st & (1<<1))      return "OVERVOLT";
    if (st & (1<<0))      return "UNDERVOLT";
    return "ERR";
}

/* 诊断建议 */
static const char *GetDiagnosis(uint32_t tx_cnt, uint32_t tx_ok, uint32_t rx_cnt, bool has_resp)
{
    if (tx_cnt == 0)          return "No cmd sent?";
    if (tx_ok == 0)           return "DMA TX stuck!";
    if (!has_resp)            return "No reply -> chk wiring/ID/pwr";
    if (rx_cnt < tx_ok)       return "Intermit reply";
    return "Servo OK";
}

/* ========================= 主任务 ========================= */

void LCD_Task(void)
{
    LCD_Init();
    LCD_Fill(0, 0, LCD_W, LCD_H, BLACK);

    /* 注册应答回调 */
    Servo_SetResponseCallback(Servo_DbgCallback);

    /* ---- 开机画面 (2 秒) ---- */
    LCD_ShowString(30, 80,  (const uint8_t *)"SERVO DEBUG", CYAN, BLACK, 24, 0);
    LCD_ShowString(30, 110, (const uint8_t *)"v1.1", CYAN, BLACK, 16, 0);
    osDelay(1000);
    LCD_Fill(0, 0, LCD_W, LCD_H, BLACK);

    /* ---- 舵机初始化 ---- */
    Servo_SetMode(1, 0x01);          /* 舵机模式 */
    osDelay(20);
    Servo_SetTorque(1, 1);           /* 开扭矩 */
    osDelay(20);

    /* ---- 进入自动测试循环 ---- */
    sv.phase_start = osKernelSysTick();
    sv.phase = 0;
    sv.last_cmd_tick = osKernelSysTick();

    for (;;)
    {
        uint32_t now = osKernelSysTick();
        uint32_t elapsed = now - sv.phase_start;
        uint8_t total = sizeof(phase_desc) / sizeof(phase_desc[0]);

        /* ======== 阶段逻辑 ======== */
        if (elapsed >= PHASE_WAIT_MS)
        {
            /* 阶段 1 (PING) 等待时间短一些, 且如果收到过应答就立刻跳走 */
            if (sv.phase == 1 && sv.has_response) {
                sv.phase = 2;
                sv.phase_start = now;
            } else {
                sv.phase++;
                sv.phase_start = now;
            }

            if (sv.phase >= total - 1) sv.phase = total - 1;  /* 停在 Done */

            sv.last_cmd_tick = now;

            switch (sv.phase) {
            case 1:  Servo_Ping(1);                    break;   /* PING 测试连通 */
            case 2:  Servo_SetPosition(1, 2048, 1000); break;   /* 0° */
            case 3:  Servo_SetPosition(1, 3072, 1000); break;   /* +90° */
            case 4:  Servo_SetPosition(1, 1024, 1000); break;   /* -90° */
            case 5:  Servo_ReadPosition(1);             break;   /* 读位置 */
            case 6:  Servo_SetPosition(1, 2048, 1000); break;   /* 回 0° */
            default: break;
            }
        }

        /* ======== 显示刷新 ======== */
        uint32_t tx_cnt   = Servo_GetTxCount();
        uint32_t tx_ok    = Servo_GetTxOkCount();
        uint32_t rx_cnt   = Servo_GetRxCount();
        uint8_t  frame_len = 0;
        const uint8_t *frame = Servo_GetLastFrame(&frame_len);

        uint16_t y = 2;
        uint8_t  fs = 16;    /* 字体大小 */
        uint8_t  lh = 19;    /* 行距 (含间距) */

        /* 标题行 */
        LCD_ShowString(5, y, (const uint8_t *)"== SERVO DEBUG ==", CYAN, BLACK, fs, 0);
        y += lh;

        /* 行1: 发送/完成/接收 计数 */
        LCD_ShowString(5,  y, (const uint8_t *)"TX:", CYAN, BLACK, fs, 0);
        LCD_ShowIntNum(35, y, (tx_cnt > 9999) ? 9999 : (uint16_t)tx_cnt, 4, YELLOW, BLACK, fs);
        LCD_ShowString(80, y, (const uint8_t *)"OK:", CYAN, BLACK, fs, 0);
        LCD_ShowIntNum(110, y, (tx_ok > 9999) ? 9999 : (uint16_t)tx_ok, 4, GREEN, BLACK, fs);
        LCD_ShowString(155, y, (const uint8_t *)"RX:", CYAN, BLACK, fs, 0);
        LCD_ShowIntNum(185, y, (rx_cnt > 9999) ? 9999 : (uint16_t)rx_cnt, 4,
                       rx_cnt ? GREEN : RED, BLACK, fs);
        y += lh;

        /* 行2: Busy + ID */
        LCD_ShowString(5,  y, (const uint8_t *)"Busy:", CYAN, BLACK, fs, 0);
        LCD_ShowIntNum(55, y, Servo_IsBusy() ? 1 : 0, 1,
                       Servo_IsBusy() ? YELLOW : GREEN, BLACK, fs);
        LCD_ShowString(90, y, (const uint8_t *)"ID:", CYAN, BLACK, fs, 0);
        LCD_ShowIntNum(120, y, sv.id, 3, WHITE, BLACK, fs);
        y += lh;

        /* 行3: 状态字节 */
        LCD_ShowString(5,  y, (const uint8_t *)"Status:0x", CYAN, BLACK, fs, 0);
        ShowHex8(85, y, sv.status, WHITE, fs);
        LCD_ShowString(120, y, (const uint8_t *)"(", CYAN, BLACK, fs, 0);
        LCD_ShowString(130, y, (const uint8_t *)StatusToStr(sv.status),
                       sv.status ? RED : GREEN, BLACK, fs, 0);
        LCD_ShowString(185, y, (const uint8_t *)")", CYAN, BLACK, fs, 0);
        y += lh;

        /* 行4: 当前位置 */
        LCD_ShowString(5,  y, (const uint8_t *)"Position:", CYAN, BLACK, fs, 0);
        LCD_ShowIntNum(85, y, sv.position, 4, WHITE, BLACK, fs);
        y += lh;

        /* 行5: 响应状态 */
        LCD_ShowString(5, y, (const uint8_t *)"Response:", CYAN, BLACK, fs, 0);
        if (sv.has_response) {
            uint32_t age = now - sv.last_resp_tick;
            LCD_ShowString(85, y, (const uint8_t *)"YES", GREEN, BLACK, fs, 0);
            LCD_ShowString(130, y, (const uint8_t *)"(ago ", CYAN, BLACK, fs, 0);
            LCD_ShowIntNum(170, y, (age > 9999) ? 9999 : (uint16_t)age, 4, WHITE, BLACK, fs);
            LCD_ShowString(215, y, (const uint8_t *)"ms)", CYAN, BLACK, fs, 0);
        } else {
            LCD_ShowString(85, y, (const uint8_t *)"NO (no reply)", RED, BLACK, fs, 0);
        }
        y += lh;

        /* 行6: 测试阶段 */
        LCD_ShowString(5,  y, (const uint8_t *)"Phase:", CYAN, BLACK, fs, 0);
        LCD_ShowIntNum(65, y, sv.phase, 1, YELLOW, BLACK, fs);
        LCD_ShowString(85, y, (const uint8_t *)"/", CYAN, BLACK, fs, 0);
        LCD_ShowIntNum(95, y, total - 1, 1, YELLOW, BLACK, fs);
        y += lh - 2;

        /* 阶段描述 + 倒计时 */
        if (sv.phase < total) {
            LCD_ShowString(5, y, (const uint8_t *)phase_desc[sv.phase], WHITE, BLACK, fs, 0);
        }
        y += lh;

        uint16_t remain = (elapsed < PHASE_WAIT_MS)
                          ? (uint16_t)(PHASE_WAIT_MS - elapsed)
                          : 0;
        LCD_ShowString(5,  y, (const uint8_t *)"Next cmd in:", CYAN, BLACK, fs, 0);
        LCD_ShowIntNum(100, y, remain, 4, WHITE, BLACK, fs);
        LCD_ShowString(145, y, (const uint8_t *)"ms", CYAN, BLACK, fs, 0);
        y += lh;

        /* 行7: 诊断建议 (根据数据自动判断) */
        {
            const char *diag = GetDiagnosis(tx_cnt, tx_ok, rx_cnt, sv.has_response);
            uint16_t diag_color = YELLOW;
            if (strcmp(diag, "Servo OK") == 0)          diag_color = GREEN;
            else if (strcmp(diag, "No reply -> chk wiring/ID/pwr") == 0) diag_color = RED;
            LCD_ShowString(5, y, (const uint8_t *)">>", CYAN, BLACK, fs, 0);
            LCD_ShowString(25, y, (const uint8_t *)diag, diag_color, BLACK, fs, 0);
        }
        y += lh;

        /* 行8: 最后发送的帧 (hex, 最多前 8 字节) */
        if (frame_len > 0) {
            LCD_ShowString(5, y, (const uint8_t *)"Frame:", CYAN, BLACK, fs, 0);
            ShowFrameHex(65, y, frame, frame_len, 8, WHITE, fs);
        }

        osDelay(50);    /* ~20fps */
    }
}
