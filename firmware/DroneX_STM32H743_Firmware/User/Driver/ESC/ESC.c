/**
 * @file ESC.c
 * @brief DShot300 驱动：TIM2_CH3/CH4 + DMA，双通道同步输出，兼容 AM32 标准协议。
 *
 * 时序（TIM2 时钟 = 240 MHz，Prescaler = 0，ARR = 799）：
 *   位周期  = 800 ticks = 3.333 us
 *   Bit 1   = 600 ticks，75%  高电平
 *   Bit 0   = 300 ticks，37.5% 高电平
 *   复位帧  =   0 ticks，0%   输出保持低（DShot 帧间空闲）
 *
 * 帧格式（16 bits, MSB first）：
 *   [15:5]  11-bit 油门值  0=停转，48..2047=调速范围
 *   [4]     Telemetry 请求位
 *   [3:0]   CRC = nibble0 ^ nibble1 ^ nibble2
 *
 * 硬件资源：
 *   TIM2_CH3 -> PA2 (AF1), DMA1_Stream0, DMAMUX1 请求 TIM2_CH3
 *   TIM2_CH4 -> PA3 (AF1), DMA1_Stream1, DMAMUX1 请求 TIM2_CH4
 *
 * D-Cache 策略：
 *   缓冲区置于普通 SRAM（32 字节对齐），发送前调用 SCB_CleanDCache_by_Addr
 *   确保 CPU 写入已刷入物理 RAM，DMA 读取到正确数据。
 *
 * 双通道独立启动：
 *   此版本 HAL 使用 per-channel ChannelState[6]，CH3 与 CH4 状态相互独立，
 *   可连续调用两次 HAL_TIM_PWM_Start_DMA 而不会产生 HAL_BUSY 冲突。
 */

#include "Driver/ESC/ESC.h"
#include "stm32h7xx_hal.h"
#include <string.h>

/* ────────────────────── DShot300 时序常数 ───────────────────── */
#define DSHOT300_ARR     799U   /* ARR=800-1；位周期=800/240MHz=3.333us */
#define DSHOT300_BIT1    600U   /* 75%  高电平 -> Bit 1 */
#define DSHOT300_BIT0    300U   /* 37.5% 高电平 -> Bit 0 */
#define DSHOT_FRAME_BITS  16U
#define DSHOT_FRAME_LEN  (DSHOT_FRAME_BITS + 1U)  /* 16位帧 + 1位复位帧 */

/* DMA 缓冲字节数，向上取整到 32 字节（D-Cache 行大小）*/
#define DSHOT_BUF_BYTES  (((DSHOT_FRAME_LEN * sizeof(uint32_t)) + 31U) & ~31U)

/* ──────────────────── 内部 HAL 句柄 ─────────────────────────── */
static TIM_HandleTypeDef s_htim2;
static DMA_HandleTypeDef s_hdma_ch3;
static DMA_HandleTypeDef s_hdma_ch4;
static TIM_HandleTypeDef s_htim4_test;  /* PD15 -> TIM4_CH4 PWM 测试输出 */

/* ─── DShot DMA 缓冲 ─────────────────────────────────────────────
 * 必须放在 AXI SRAM（0x24000000）而非 DTCM（0x20000000），
 * 因为 DMA1/DMA2 总线主机无法访问 DTCM。
 * scatter file 已将 .DMA_BUF section 映射到 AXI SRAM。
 * 32 字节对齐保证 SCB_CleanDCache_by_Addr 按整 cache 行操作。
 * ──────────────────────────────────────────────────────────────── */
static uint32_t s_buf_ch3[DSHOT_FRAME_LEN] __attribute__((section(".DMA_BUF"), aligned(32)));
static uint32_t s_buf_ch4[DSHOT_FRAME_LEN] __attribute__((section(".DMA_BUF"), aligned(32)));

/* ──────── 待发送原始值（Set* 写入，ESC_Update 时编帧）─────────  */
static uint16_t s_pending_val[ESC_CH_COUNT];
static uint8_t  s_pending_tel[ESC_CH_COUNT];

/* ──────────────── 调试统计 & DMA 忙标志 ─────────────────────── */
static ESC_DebugInfo_t  s_dbg;
static volatile uint8_t s_dma_busy;

#define DMA_BUSY_CH3  0x01U
#define DMA_BUSY_CH4  0x02U

/* ─────────────────── 空闲态强制拉低（非反转 DShot） ───────────────────
 * 说明：
 *   HAL_TIM_PWM_Stop_DMA 会关闭通道输出；在无外部下拉/上拉时，引脚可能呈现不确定电平。
 *   为确保 DShot 空闲态为“低电平”，每次发送完成后将引脚临时切为 GPIO 推挽输出并拉低；
 *   下一次发送前再切回 AF1_TIM2。
 */
static inline void ESC_PinToGpioLow_PA2(void)
{
    /* ODR=0 */
    GPIOA->BSRR = (1U << (2U + 16U));
    /* 推挽 */
    GPIOA->OTYPER &= ~(1U << 2U);
    /* 无上下拉 */
    GPIOA->PUPDR &= ~(3U << (2U * 2U));
    /* 高速 */
    GPIOA->OSPEEDR = (GPIOA->OSPEEDR & ~(3U << (2U * 2U))) | (3U << (2U * 2U));
    /* 输出模式 MODER=01 */
    GPIOA->MODER   = (GPIOA->MODER   & ~(3U << (2U * 2U))) | (1U << (2U * 2U));
}

static inline void ESC_PinToGpioLow_PA3(void)
{
    GPIOA->BSRR = (1U << (3U + 16U));
    GPIOA->OTYPER &= ~(1U << 3U);
    GPIOA->PUPDR &= ~(3U << (3U * 2U));
    GPIOA->OSPEEDR = (GPIOA->OSPEEDR & ~(3U << (3U * 2U))) | (3U << (3U * 2U));
    GPIOA->MODER   = (GPIOA->MODER   & ~(3U << (3U * 2U))) | (1U << (3U * 2U));
}

static inline void ESC_PinsToAF_TIM2_CH3_CH4(void)
{
    /* AFRL: PA2/PA3 -> AF1 (TIM2) */
    GPIOA->AFR[0] = (GPIOA->AFR[0] & ~(0xFU << (2U * 4U))) | (0x1U << (2U * 4U));
    GPIOA->AFR[0] = (GPIOA->AFR[0] & ~(0xFU << (3U * 4U))) | (0x1U << (3U * 4U));

    /* 推挽 + 无上下拉 + 高速 */
    GPIOA->OTYPER &= ~((1U << 2U) | (1U << 3U));
    GPIOA->PUPDR  &= ~((3U << (2U * 2U)) | (3U << (3U * 2U)));
    GPIOA->OSPEEDR = (GPIOA->OSPEEDR & ~((3U << (2U * 2U)) | (3U << (3U * 2U))))
                   |  ((3U << (2U * 2U)) | (3U << (3U * 2U)));

    /* 复用模式 MODER=10 */
    GPIOA->MODER = (GPIOA->MODER & ~((3U << (2U * 2U)) | (3U << (3U * 2U))))
                 |  ((2U << (2U * 2U)) | (2U << (3U * 2U)));
}

/* ════════════════════════════════════════════════════════════════
 * 内部：编码一帧 DShot 到 DMA 缓冲
 *   帧格式：[15:5]=throttle  [4]=telemetry  [3:0]=CRC
 * ════════════════════════════════════════════════════════════════ */
static void encode_frame(uint32_t *buf, uint16_t throttle, bool telemetry)
{
    uint16_t packet = (uint16_t)((throttle << 1U) | (telemetry ? 1U : 0U));
    uint16_t crc    = (uint16_t)((packet ^ (packet >> 4U) ^ (packet >> 8U)) & 0x0FU);
    uint16_t frame  = (uint16_t)((packet << 4U) | crc);

    for (uint32_t i = 0U; i < DSHOT_FRAME_BITS; i++) {
        buf[i] = (frame & 0x8000U) ? DSHOT300_BIT1 : DSHOT300_BIT0;
        frame  = (uint16_t)(frame << 1U);
    }
    buf[DSHOT_FRAME_BITS] = 0U;  /* 复位帧：CCR=0 -> 输出保持低电平 */
}

/* ════════════════════════════════════════════════════════════════
 * HAL MSP 覆盖（覆盖 stm32h7xx_hal_tim.c 中的 __weak 定义）
 *   由 HAL_TIM_PWM_Init 自动调用，完成 GPIO / DMA / NVIC 配置。
 * ════════════════════════════════════════════════════════════════ */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM2) { return; }

    s_dbg.msp_init_count++;

    /* 1. 外设时钟使能 */
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* 2. GPIO：PA2=TIM2_CH3，PA3=TIM2_CH4，复用 AF1，推挽输出 */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin       = GPIO_PIN_2 | GPIO_PIN_3;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_PULLDOWN;
    gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &gpio);

    /* 3. DMA1_Stream0 -> TIM2_CH3（CCR3 为 32-bit，使用 WORD 对齐）*/
    s_hdma_ch3.Instance                 = DMA1_Stream0;
    s_hdma_ch3.Init.Request             = DMA_REQUEST_TIM2_CH3;
    s_hdma_ch3.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    s_hdma_ch3.Init.PeriphInc           = DMA_PINC_DISABLE;
    s_hdma_ch3.Init.MemInc              = DMA_MINC_ENABLE;
    s_hdma_ch3.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    s_hdma_ch3.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
    s_hdma_ch3.Init.Mode                = DMA_NORMAL;
    s_hdma_ch3.Init.Priority            = DMA_PRIORITY_MEDIUM;
    s_hdma_ch3.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&s_hdma_ch3) != HAL_OK) { return; }
    __HAL_LINKDMA(htim, hdma[TIM_DMA_ID_CC3], s_hdma_ch3);

    /* 4. DMA1_Stream1 -> TIM2_CH4 */
    s_hdma_ch4.Instance                 = DMA1_Stream1;
    s_hdma_ch4.Init.Request             = DMA_REQUEST_TIM2_CH4;
    s_hdma_ch4.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    s_hdma_ch4.Init.PeriphInc           = DMA_PINC_DISABLE;
    s_hdma_ch4.Init.MemInc              = DMA_MINC_ENABLE;
    s_hdma_ch4.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    s_hdma_ch4.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
    s_hdma_ch4.Init.Mode                = DMA_NORMAL;
    s_hdma_ch4.Init.Priority            = DMA_PRIORITY_MEDIUM;
    s_hdma_ch4.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&s_hdma_ch4) != HAL_OK) { return; }
    __HAL_LINKDMA(htim, hdma[TIM_DMA_ID_CC4], s_hdma_ch4);

    /* 5. NVIC：DMA 中断优先级 5，低于 PIT（1..3）
     *    确保 ESC 任务不被 DMA 回调抢占；回调在 ESC 任务返回后执行 */
    HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
    HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
}

/* ════════════════════════════════════════════════════════════════
 * IRQ 处理（覆盖 startup_stm32h743xx.s 中的弱定义）
 * ════════════════════════════════════════════════════════════════ */
void DMA1_Stream0_IRQHandler(void)
{
    s_dbg.dma_irq_ch3_count++;
    HAL_DMA_IRQHandler(&s_hdma_ch3);
}

void DMA1_Stream1_IRQHandler(void)
{
    s_dbg.dma_irq_ch4_count++;
    HAL_DMA_IRQHandler(&s_hdma_ch4);
}

/* ════════════════════════════════════════════════════════════════
 * HAL 回调：DMA 传输完成
 *
 * 调用链：
 *   HAL_DMA_IRQHandler
 *     -> TIM_DMADelayPulseCplt（HAL 内部）
 *        -> 将通道 ChannelState 置为 READY（DMA_NORMAL 模式）
 *        -> 本函数
 *
 * 本函数再调用 HAL_TIM_PWM_Stop_DMA：
 *   - 清除 TIM_DIER 中的 CCxDE（禁止 DMA 请求）
 *   - 禁用 CCx 输出（引脚回到低电平 = DShot 空闲态）
 *   - 双通道：先完成的通道停止后另一通道仍使能，__HAL_TIM_DISABLE 无效；
 *             最后完成的通道停止后所有通道禁用，计数器随之停止
 * ════════════════════════════════════════════════════════════════ */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM2) { return; }

    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
        s_dbg.pulse_done_ch3_count++;
        HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_3);
        ESC_PinToGpioLow_PA2();
        s_dma_busy &= (uint8_t)~DMA_BUSY_CH3;
    }
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
        s_dbg.pulse_done_ch4_count++;
        HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_4);
        ESC_PinToGpioLow_PA3();
        s_dma_busy &= (uint8_t)~DMA_BUSY_CH4;
    }
    s_dbg.dma_busy = s_dma_busy;
}

void HAL_TIM_PWM_PulseFinishedHalfCpltCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM2) { return; }
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) { s_dbg.pulse_half_ch3_count++; }
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) { s_dbg.pulse_half_ch4_count++; }
}

void HAL_TIM_ErrorCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) { s_dbg.tim_error_count++; }
}

/* ════════════════════════════════════════════════════════════════
 *                          公开 API
 * ════════════════════════════════════════════════════════════════ */

ESC_Status_t ESC_Init(void)
{
    memset(&s_dbg,        0, sizeof(s_dbg));
    memset(s_buf_ch3,     0, sizeof(s_buf_ch3));
    memset(s_buf_ch4,     0, sizeof(s_buf_ch4));
    memset(s_pending_val, 0, sizeof(s_pending_val));
    memset(s_pending_tel, 0, sizeof(s_pending_tel));
    s_dma_busy = 0U;

    /* TIM2：Prescaler=0 -> 计数频率=240MHz，ARR=799 -> 位周期=3.333us */
    s_htim2.Instance               = TIM2;
    s_htim2.Init.Prescaler         = 0U;
    s_htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    s_htim2.Init.Period            = DSHOT300_ARR;
    s_htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    s_htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&s_htim2) != HAL_OK) {
        return ESC_ERR_HAL;
    }

    /* PWM Mode1（CNT<CCR->高，CNT>=CCR->低），高极性，预装载禁用 */
    TIM_OC_InitTypeDef oc = {0};
    oc.OCMode     = TIM_OCMODE_PWM1;
    oc.Pulse      = 0U;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&s_htim2, &oc, TIM_CHANNEL_3) != HAL_OK ||
        HAL_TIM_PWM_ConfigChannel(&s_htim2, &oc, TIM_CHANNEL_4) != HAL_OK) {
        return ESC_ERR_HAL;
    }

    /* 确保上电后空闲态为低电平 */
    ESC_PinToGpioLow_PA2();
    ESC_PinToGpioLow_PA3();

    s_dbg.initialized = 1U;
    return ESC_OK;
}

/* 简单 PWM 输出：PD15 = TIM4_CH4，50Hz，占空比约 1.5ms（兼容标准 PWM 电调/舵机） */
void ESC_TestPwm_StartOnPD15(void)
{
    /* 1. 使能时钟 */
    __HAL_RCC_TIM4_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /* 2. GPIO：PD15 -> TIM4_CH4 (AF2) */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin       = GPIO_PIN_15;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOD, &gpio);

    /* 3. TIM4：假定定时器时钟约为 240MHz
     *    PSC=2399 -> 计数频率 100kHz（10us 一计数）
     *    ARR=1999 -> 周期 20000us = 50Hz
     *    CCR4=150 -> 高电平约 150 * 10us = 1.5ms
     */
    s_htim4_test.Instance               = TIM4;
    s_htim4_test.Init.Prescaler         = 2399U;
    s_htim4_test.Init.CounterMode       = TIM_COUNTERMODE_UP;
    s_htim4_test.Init.Period            = 1999U;
    s_htim4_test.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    s_htim4_test.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&s_htim4_test) != HAL_OK) {
        return;
    }

    TIM_OC_InitTypeDef oc = {0};
    oc.OCMode     = TIM_OCMODE_PWM1;
    oc.Pulse      = 150U;  /* 1.5ms 脉宽（中位） */
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&s_htim4_test, &oc, TIM_CHANNEL_4) != HAL_OK) {
        return;
    }

    (void)HAL_TIM_PWM_Start(&s_htim4_test, TIM_CHANNEL_4);
}

/* PD15 PWM 与 power 同步：power 0 -> 1ms 脉宽，power 1000 -> 2ms 脉宽（50Hz 周期 20ms） */
void ESC_TestPwm_SetPower(uint16_t power_0_1000)
{
    if (power_0_1000 > 1000U) power_0_1000 = 1000U;
    /* 100 ticks=1ms, 200 ticks=2ms */
    uint32_t pulse = 100U + (uint32_t)power_0_1000 * 100U / 1000U;
    __HAL_TIM_SET_COMPARE(&s_htim4_test, TIM_CHANNEL_4, (uint32_t)pulse);
}

ESC_Status_t ESC_SetThrottleRaw(ESC_Channel_t ch, uint16_t dshot_value, bool request_telemetry)
{
    if (ch >= ESC_CH_COUNT)  { return ESC_ERR_PARAM;    }
    if (!s_dbg.initialized)  { return ESC_ERR_NOT_INIT; }
    if (dshot_value > 2047U) { dshot_value = 2047U;     }

    s_pending_val[ch] = dshot_value;
    s_pending_tel[ch] = request_telemetry ? 1U : 0U;
    return ESC_OK;
}

ESC_Status_t ESC_SetThrottleBidirectional(ESC_Channel_t ch, int16_t throttle)
{
    uint16_t dshot_val;
    if (throttle <= 0) {
        dshot_val = 0U;
    } else {
        uint32_t t = (uint32_t)throttle;
        if (t > 1000U) { t = 1000U; }
        /* 1..1000 线性映射 -> DShot 48..2047 */
        dshot_val = (uint16_t)(48U + (t - 1U) * (2047U - 48U) / 999U);
    }
    return ESC_SetThrottleRaw(ch, dshot_val, false);
}

ESC_Status_t ESC_SetInputMapped(ESC_Channel_t ch, uint16_t input_value)
{
    uint16_t dshot_val;
    if (input_value == 0U) {
        dshot_val = 0U;
    } else {
        uint32_t v = input_value;
        if (v > 1000U) { v = 1000U; }
        dshot_val = (uint16_t)(48U + (v - 1U) * (2047U - 48U) / 999U);
    }
    return ESC_SetThrottleRaw(ch, dshot_val, false);
}

ESC_Status_t ESC_Update(void)
{
    if (!s_dbg.initialized) { return ESC_ERR_NOT_INIT; }
    s_dbg.update_req_count++;

    if (s_dma_busy) {
        s_dbg.update_busy_reject_count++;
        return ESC_ERR_BUSY;
    }

    /* 每次发送前恢复为 TIM2 复用输出（上一次发送完成后会被切回 GPIO 低电平） */
    ESC_PinsToAF_TIM2_CH3_CH4();

    /* 切到 AF 后通道尚未使能，引脚会浮空导致信号线电压线性上升；立即用 TIM 驱动为低（CCR=0 并开启输出），再启动 DMA */
    s_htim2.Instance->CCR3 = 0U;
    s_htim2.Instance->CCR4 = 0U;
    s_htim2.Instance->CCER |= (TIM_CCER_CC3E | TIM_CCER_CC4E);

    /* 编帧 */
    encode_frame(s_buf_ch3, s_pending_val[ESC_CH1], (bool)s_pending_tel[ESC_CH1]);
    encode_frame(s_buf_ch4, s_pending_val[ESC_CH2], (bool)s_pending_tel[ESC_CH2]);

    /* 刷 D-Cache -> SRAM，确保 DMA 读到最新内容（缓冲区已 32 字节对齐）*/
    SCB_CleanDCache_by_Addr((uint32_t *)s_buf_ch3, (int32_t)DSHOT_BUF_BYTES);
    SCB_CleanDCache_by_Addr((uint32_t *)s_buf_ch4, (int32_t)DSHOT_BUF_BYTES);

    s_dbg.last_raw_ch1 = s_pending_val[ESC_CH1];
    s_dbg.last_raw_ch2 = s_pending_val[ESC_CH2];

    /* 重置计数器，保证每帧从周期起点开始（上帧 Stop_DMA 后计数器已停止）*/
    __HAL_TIM_SET_COUNTER(&s_htim2, 0U);

    /* 启动 DMA — CH3 (PA2) */
    if (HAL_TIM_PWM_Start_DMA(&s_htim2, TIM_CHANNEL_3,
                               s_buf_ch3, DSHOT_FRAME_LEN) == HAL_OK) {
        s_dma_busy |= DMA_BUSY_CH3;
    } else {
        s_dbg.start_dma_ch3_fail_count++;
    }

    /* 启动 DMA — CH4 (PA3)：ChannelState[3] 与 ChannelState[2] 独立，无冲突 */
    if (HAL_TIM_PWM_Start_DMA(&s_htim2, TIM_CHANNEL_4,
                               s_buf_ch4, DSHOT_FRAME_LEN) == HAL_OK) {
        s_dma_busy |= DMA_BUSY_CH4;
    } else {
        s_dbg.start_dma_ch4_fail_count++;
    }

    s_dbg.dma_busy = s_dma_busy;
    s_dbg.update_ok_count++;
    return ESC_OK;
}

bool ESC_IsBusy(void)
{
    return (s_dma_busy != 0U);
}

void ESC_GetDebugInfo(ESC_DebugInfo_t *info)
{
    if (info != NULL) { *info = s_dbg; }
}

void ESC_RecoverIfStuck(uint32_t timeout_ms)
{
    static uint32_t s_stuck_since_ms = 0U;

    if (!s_dma_busy) {
        s_stuck_since_ms = 0U;
        return;
    }

    uint32_t now = HAL_GetTick();
    if (s_stuck_since_ms == 0U) {
        s_stuck_since_ms = now;
        return;
    }

    if ((now - s_stuck_since_ms) >= timeout_ms) {
        /* 强制停止：清 CCxDE、中止 DMA、禁用通道输出、关闭计数器 */
        HAL_TIM_PWM_Stop_DMA(&s_htim2, TIM_CHANNEL_3);
        HAL_TIM_PWM_Stop_DMA(&s_htim2, TIM_CHANNEL_4);
        s_dma_busy            = 0U;
        s_dbg.dma_busy        = 0U;
        s_dbg.stuck_recover_count++;
        s_stuck_since_ms      = 0U;
    }
}
