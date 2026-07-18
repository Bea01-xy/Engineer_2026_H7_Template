/**
  ******************************************************************************
  * @file           : Buzzer.c
  * @brief          : Passive buzzer driver via TIM12_CH2 (PB15)
  * @author         : Ported from CtrBoard-H7_BUZZER
  ******************************************************************************
  *
  * 时钟推导 (从主项目 SystemClock_Config):
  *   HSE = 8 MHz
  *   PLL: M=6, N=160, P=1  →  SYSCLK = 8/6×160 = 213.33 MHz
  *   HCLK = SYSCLK/4       =  53.33 MHz
  *   APB1 = HCLK/2         =  26.67 MHz
  *   TIM12_CLK = APB1×2    =  53.33 MHz  (APB1 prescaler=2 ≠ 1 → 乘 2)
  *   PSC = 16-1            →  计数器时钟 = 53.33/16 = 3.333 MHz
  *
  *  因此 ARR = 3333333 / freq - 1, 与参考项目 (80MHz/24=3.333MHz) 完全相同,
  *  可直接复用参考项目的音高表。
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "Buzzer.h"
#include "main.h"            /* HAL, Error_Handler */
#include "cmsis_os.h"        /* osDelay */

/* Private variables ---------------------------------------------------------*/
static TIM_HandleTypeDef  hbuzzer_tim;
static uint8_t            buzzer_inited = 0;

/**
  * @brief  音高表 — 简谱 7 音 × 4 个八度
  * @note   arr[oct+1][deg-1], oct: -1=低音, 0=中音, 1=高音, 2=倍高
  *         值与参考项目完全一致 (计数器时钟同为 3.333 MHz)
  */
static const uint16_t note_arr[4][7] = {
    {25479, 22700, 20224, 19091, 17005, 15151, 13499},  /* oct -1: 低音 */
    {12739, 11349, 10112,  9545,  8502,  7575,  6749},  /* oct  0: 中音 */
    { 6369,  5674,  5056,  4772,  4251,  3787,  3374},  /* oct  1: 高音 */
    { 3184,  2837,  2528,  2386,  2125,  1893,  1687},  /* oct  2: 倍高 */
};

/* ======================== Demo 旋律 ======================== */
typedef struct {
    int8_t  oct;      /* 八度: -1/0/1/2 */
    uint8_t deg;      /* 简谱音级: 1-7, 0=休止 */
    uint16_t ms;      /* 时长 (ms) */
} buzzer_note_t;

/* Creep (Radiohead) — G 大调 BPM≈92 */
#define BPM 92
static const uint16_t Q = TEMPO(BPM);

static const buzzer_note_t demo_melody[] = {
    //When you were here before
    {  0, 0, Q/1 },
    {  0, 0, Q/1 },
    {  0, 2, Q/4 },
    {  0, 2, Q/4 },
    {  0, 1, Q/4 },
    { -1, 7, Q/4 },
    { -1, 7, Q/4 },
    {  0, 1, Q/4 },
    {  0, 1, Q/2 },
    {  0, 1, Q/1 },

};

/* ======================== 私有函数 ======================== */

/**
  * @brief  向 TIM12 写入频率对应的 ARR & CCR2
  * @param  arr  自动重装载值 (0 = 静音)
  */
static void buzzer_set_arr(uint16_t arr)
{
    if (arr == 0 || !buzzer_inited) {
        /* 静音: 占空比置 0, TIM 继续跑 */
        __HAL_TIM_SET_COMPARE(&hbuzzer_tim, TIM_CHANNEL_2, 0);
        return;
    }
    __HAL_TIM_SET_AUTORELOAD(&hbuzzer_tim, arr);
    __HAL_TIM_SET_COMPARE(&hbuzzer_tim, TIM_CHANNEL_2, arr / 2);
}

/* ======================== 公有 API ======================== */

/**
  * @brief  初始化蜂鸣器
  * @note   自包含, 不依赖 CubeMX 生成的 tim.c MSP 函数
  */
void Buzzer_Init(void)
{
    GPIO_InitTypeDef  gpio = {0};
    TIM_OC_InitTypeDef oc  = {0};

    if (buzzer_inited) return;

    /* ---- 1. 时钟使能 ---- */
    __HAL_RCC_TIM12_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* ---- 2. GPIO: PB15 → TIM12_CH2 (AF2) ---- */
    gpio.Pin       = GPIO_PIN_15;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF2_TIM12;
    HAL_GPIO_Init(GPIOB, &gpio);

    /* ---- 3. TIM12 基础配置 ---- */
    hbuzzer_tim.Instance               = TIM12;
    hbuzzer_tim.Init.Prescaler         = 16 - 1;           /* 3.333 MHz */
    hbuzzer_tim.Init.CounterMode       = TIM_COUNTERMODE_UP;
    hbuzzer_tim.Init.Period            = 2000 - 1;         /* 临时占位值 */
    hbuzzer_tim.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    hbuzzer_tim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&hbuzzer_tim) != HAL_OK) {
        Error_Handler();
    }

    /* ---- 4. PWM 通道配置 ---- */
    oc.OCMode     = TIM_OCMODE_PWM1;
    oc.Pulse      = 0;                  /* 初始静音 */
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&hbuzzer_tim, &oc, TIM_CHANNEL_2) != HAL_OK) {
        Error_Handler();
    }

    /* ---- 5. 启动 PWM 输出 ---- */
    if (HAL_TIM_PWM_Start(&hbuzzer_tim, TIM_CHANNEL_2) != HAL_OK) {
        Error_Handler();
    }

    buzzer_inited = 1;
}

/**
  * @brief  设定 PWM 频率
  * @param  freq_hz  频率 (Hz), 0 则停止输出
  */
void Buzzer_SetFreq(uint16_t freq_hz)
{
    if (!buzzer_inited) return;

    if (freq_hz == 0) {
        Buzzer_Stop();
        return;
    }

    /* 计数器时钟 = 3,333,333 Hz  →  ARR = 3333333 / freq - 1 */
    uint32_t arr = 3333333u / freq_hz;
    if (arr > 65535u) arr = 65535u;
    if (arr < 2u)     arr = 2u;
    buzzer_set_arr((uint16_t)arr);
}

/**
  * @brief  设定音量 (PWM 占空比)
  * @param  percent  0~100
  */
void Buzzer_SetVolume(uint8_t percent)
{
    if (!buzzer_inited) return;
    if (percent > 100) percent = 100;

    uint16_t arr = __HAL_TIM_GET_AUTORELOAD(&hbuzzer_tim);
    if (arr == 0) return;

    __HAL_TIM_SET_COMPARE(&hbuzzer_tim, TIM_CHANNEL_2, (uint32_t)arr * percent / 100);
}

/**
  * @brief  停止输出 (占空比置 0)
  */
void Buzzer_Stop(void)
{
    if (!buzzer_inited) return;
    __HAL_TIM_SET_COMPARE(&hbuzzer_tim, TIM_CHANNEL_2, 0);
}

/**
  * @brief  播放单个音符 (阻塞)
  * @param  arr          NOTE_xxx 宏值, NOTE_REST=0 为休止
  * @param  duration_ms  持续时间 (ms)
  */
void Buzzer_PlayNote(uint16_t arr, uint16_t duration_ms)
{
    if (!buzzer_inited) return;

    buzzer_set_arr(arr);
    osDelay(duration_ms);
}

/**
  * @brief  播放示例旋律 (阻塞)
  * @note   播放完毕后返回, 此后可正常使用 Buzzer 其他 API
  */
void Buzzer_Demo(void)
{
    uint32_t i, n;

    if (!buzzer_inited) {
        Buzzer_Init();
    }

    n = sizeof(demo_melody) / sizeof(demo_melody[0]);
    for (i = 0; i < n; i++) {
        int8_t  oct = demo_melody[i].oct;
        uint8_t deg = demo_melody[i].deg;
        uint16_t ms = demo_melody[i].ms;

        if (deg == 0 || oct < -1 || oct > 2) {
            /* 休止符 */
            Buzzer_Stop();
        } else {
            uint16_t arr = note_arr[oct + 1][deg - 1];
            buzzer_set_arr(arr);
        }
        osDelay(ms);
    }

    /* 结束后静音 */
    Buzzer_Stop();
}
