/**
 * @file ESC.c
 * @brief DShot300 驱动：TIM2_CH3/CH4 + DMA，双通道同步输出，兼容 AM32 标准协议。
 *
 * 时序（TIM2 时钟 = 240 MHz，Prescaler = 0，ARR = 799）：
 *   位周期  = 800 ticks = 3.333 us
 *   Bit 1   = 600 ticks，75%  高电平
 *   Bit 0   = 300 ticks，37.5% 高电平
 *   拖尾空闲 = 若干槽 CCR=0，每槽 1 位时（≈3.33µs）保持低，拉长帧间线空闲
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
 *
 * 双向 DShot 时序/引脚参考 Betaflight：
 *   platform/STM32/pwm_output_dshot_hal.c（pwmDshotSetDirectionInput、GPIO 上拉、ICFilter）
 *   platform/common/stm32/pwm_output_dshot_shared.c（pwmTelemetryDecode）
 */

#include "Driver/ESC/ESC.h"
#include "Driver/Timing/Timing.h"
#include "stm32h7xx_hal.h"
#include <string.h>

/* ────────────────────── DShot300 时序常数 ───────────────────── */
#define DSHOT300_ARR     799U   /* ARR=800-1；位周期=800/240MHz=3.333us */
#define DSHOT300_BIT1    600U   /* 75%  高电平 -> Bit 1 */
#define DSHOT300_BIT0    300U   /* 37.5% 高电平 -> Bit 0 */
#define DSHOT_FRAME_BITS         16U
#define DSHOT_TRAILING_LOW_SLOTS 3U   /* 16 位后连续全低槽数（≈3×3.33µs，与 BF 18 字档同级并略长） */
#define DSHOT_FRAME_LEN  (DSHOT_FRAME_BITS + DSHOT_TRAILING_LOW_SLOTS)
#define TIM2_TX_PSC       0U
#define TIM2_TX_ARR       DSHOT300_ARR
#define TIM2_RX_PSC       23U        /* 240MHz / (23+1) = 10MHz -> 0.1us/tick */
#define TIM2_RX_ARR       0xFFFFFFFFU
#define ESC_RX_MAX_EDGES  48U
/* 无有效边沿时用 DWT 微秒超时结束 RX；勿用 HAL_GetTick(ms) 否则回传失败会卡满 ~1ms，4kHz 下大量丢帧电调无法解锁 */
#define ESC_RX_TIMEOUT_US  300U

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

#define DMA_BUSY_CH3_TX  0x01U
#define DMA_BUSY_CH4_TX  0x02U
#define DMA_BUSY_CH3_RX  0x04U
#define DMA_BUSY_CH4_RX  0x08U

/* 双通道 TX：HAL_TIM_PWM_Stop_DMA 会 __HAL_TIM_DISABLE 整颗定时器，先完成的一路会掐死另一路 DMA */
#define ESC_TX_DMA_DONE_CH3  0x01U
#define ESC_TX_DMA_DONE_CH4  0x02U
static volatile uint8_t s_esc_tx_dma_done_bits;
static volatile uint8_t s_esc_tx_expect_mask; /* 本帧实际启动的 TX DMA 通道，用于与 done_bits 对齐后关 TIM */

typedef struct {
    uint32_t edge_dt[ESC_RX_MAX_EDGES];
    uint8_t edge_count;
    uint8_t active;
    uint8_t has_first_capture;
    uint32_t last_capture;
    uint32_t start_cycles; /* DWT，用于 RX 微秒超时 */
    uint32_t start_ms;
} ESC_RxState_t;

static ESC_RxState_t s_rx[ESC_CH_COUNT];
static ESC_Telemetry_t s_telem[ESC_CH_COUNT];
static uint8_t s_rx_start_pending;
static ESC_Command_t s_service_cmd[ESC_CH_COUNT];
static ESC_ServiceState_t s_service_state[ESC_CH_COUNT];
#if ESC_BIDIR_DSHOT
static uint32_t s_bidir_tel_frame;
#endif

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

/* 双路同时发 DShot：两路均 AF1。上拉仅用于双向（开漏回传）；纯发码时勿开上拉以免边沿变差、电调不识别 */
static inline void ESC_PinsToAF_TxBoth(void)
{
    /* AFRL: PA2/PA3 -> AF1 (TIM2) */
    GPIOA->AFR[0] = (GPIOA->AFR[0] & ~(0xFU << (2U * 4U))) | (0x1U << (2U * 4U));
    GPIOA->AFR[0] = (GPIOA->AFR[0] & ~(0xFU << (3U * 4U))) | (0x1U << (3U * 4U));

    GPIOA->OTYPER &= ~((1U << 2U) | (1U << 3U));
#if ESC_BIDIR_DSHOT
    GPIOA->PUPDR  = (GPIOA->PUPDR & ~((3U << (2U * 2U)) | (3U << (3U * 2U))))
                  | ((1U << (2U * 2U)) | (1U << (3U * 2U))); /* PUPDR=01 上拉 */
#else
    GPIOA->PUPDR &= ~((3U << (2U * 2U)) | (3U << (3U * 2U)));
#endif
    GPIOA->OSPEEDR = (GPIOA->OSPEEDR & ~((3U << (2U * 2U)) | (3U << (3U * 2U))))
                   |  ((3U << (2U * 2U)) | (3U << (3U * 2U)));

    /* 复用模式 MODER=10 */
    GPIOA->MODER = (GPIOA->MODER & ~((3U << (2U * 2U)) | (3U << (3U * 2U))))
                 |  ((2U << (2U * 2U)) | (2U << (3U * 2U)));
}

/*
 * 仅一路做 GCR 回传时：接收脚 AF+上拉；另一路保持 GPIO 推挽强低。
 * 若两路均 AF 而仅一路开 IC，未参与通道易呈高阻，与相邻线串扰会被另一路 ESC 当成油门（零油门误转）。
 */
static inline void ESC_PinsToAF_RxCh1_TxHoldCh2Low(void)
{
    /* PA2: TIM2_CH3 输入捕获 */
    GPIOA->AFR[0] = (GPIOA->AFR[0] & ~(0xFU << (2U * 4U))) | (0x1U << (2U * 4U));
    GPIOA->MODER   = (GPIOA->MODER & ~(3U << (2U * 2U))) | (2U << (2U * 2U));
    GPIOA->OTYPER &= ~(1U << 2U);
    GPIOA->PUPDR   = (GPIOA->PUPDR & ~(3U << (2U * 2U))) | (1U << (2U * 2U));
    GPIOA->OSPEEDR = (GPIOA->OSPEEDR & ~(3U << (2U * 2U))) | (3U << (2U * 2U));

    /* PA3: 强低，不参与本帧遥测 */
    GPIOA->BSRR    = (1U << (3U + 16U));
    GPIOA->OTYPER &= ~(1U << 3U);
    GPIOA->PUPDR &= ~(3U << (3U * 2U));
    GPIOA->OSPEEDR = (GPIOA->OSPEEDR & ~(3U << (3U * 2U))) | (3U << (3U * 2U));
    GPIOA->MODER   = (GPIOA->MODER & ~(3U << (3U * 2U))) | (1U << (3U * 2U));
}

static inline void ESC_PinsToAF_RxCh2_TxHoldCh1Low(void)
{
    /* PA2: 强低 */
    GPIOA->BSRR    = (1U << (2U + 16U));
    GPIOA->OTYPER &= ~(1U << 2U);
    GPIOA->PUPDR &= ~(3U << (2U * 2U));
    GPIOA->OSPEEDR = (GPIOA->OSPEEDR & ~(3U << (2U * 2U))) | (3U << (2U * 2U));
    GPIOA->MODER   = (GPIOA->MODER & ~(3U << (2U * 2U))) | (1U << (2U * 2U));

    /* PA3: TIM2_CH4 输入捕获 */
    GPIOA->AFR[0] = (GPIOA->AFR[0] & ~(0xFU << (3U * 4U))) | (0x1U << (3U * 4U));
    GPIOA->MODER   = (GPIOA->MODER & ~(3U << (3U * 2U))) | (2U << (3U * 2U));
    GPIOA->OTYPER &= ~(1U << 3U);
    GPIOA->PUPDR   = (GPIOA->PUPDR & ~(3U << (3U * 2U))) | (1U << (3U * 2U));
    GPIOA->OSPEEDR = (GPIOA->OSPEEDR & ~(3U << (3U * 2U))) | (3U << (3U * 2U));
}

static void ESC_Tim2ConfigTx(void)
{
    s_htim2.Instance->CR1 &= ~TIM_CR1_CEN;
    s_htim2.Instance->PSC  = TIM2_TX_PSC;
    s_htim2.Instance->ARR  = TIM2_TX_ARR;
    s_htim2.Instance->EGR  = TIM_EGR_UG;
}

static void ESC_Tim2ConfigRx(void)
{
    s_htim2.Instance->CR1 &= ~TIM_CR1_CEN;
    s_htim2.Instance->PSC  = TIM2_RX_PSC;
    s_htim2.Instance->ARR  = TIM2_RX_ARR;
    s_htim2.Instance->EGR  = TIM_EGR_UG;
    s_htim2.Instance->CNT  = 0U;
}

static uint8_t gcr5b_to_nibble(uint8_t code)
{
    switch (code) {
        case 0x19: return 0x0;
        case 0x1B: return 0x1;
        case 0x12: return 0x2;
        case 0x13: return 0x3;
        case 0x1D: return 0x4;
        case 0x15: return 0x5;
        case 0x16: return 0x6;
        case 0x17: return 0x7;
        case 0x1A: return 0x8;
        case 0x09: return 0x9;
        case 0x0A: return 0xA;
        case 0x0B: return 0xB;
        case 0x1E: return 0xC;
        case 0x0D: return 0xD;
        case 0x0E: return 0xE;
        case 0x0F: return 0xF;
        default:   return 0xFF;
    }
}

static uint8_t esc_crc4_calc(uint16_t data12)
{
    uint8_t n0 = (uint8_t)(data12 & 0x0FU);
    uint8_t n1 = (uint8_t)((data12 >> 4U) & 0x0FU);
    uint8_t n2 = (uint8_t)((data12 >> 8U) & 0x0FU);
    return (uint8_t)((n0 ^ n1 ^ n2) & 0x0FU);
}

static bool ESC_DecodeTelemetryFromEdges(const ESC_RxState_t *rx, ESC_Telemetry_t *out)
{
    uint8_t bits[40];
    uint8_t bit_count = 0U;
    uint8_t level = 1U;
    uint32_t min_dt = 0xFFFFFFFFU;

    /* Betaflight MIN_GCR_EDGES = 7 */
    if (rx->edge_count < 7U) {
        return false;
    }

    for (uint8_t i = 0U; i < rx->edge_count; i++) {
        uint32_t dt = rx->edge_dt[i];
        if (dt > 0U && dt < min_dt) {
            min_dt = dt;
        }
    }
    if (min_dt == 0xFFFFFFFFU || min_dt == 0U) {
        return false;
    }

    for (uint8_t i = 0U; i < rx->edge_count && bit_count < 40U; i++) {
        uint32_t dt = rx->edge_dt[i];
        uint32_t repeat = (dt + (min_dt / 2U)) / min_dt;
        if (repeat < 1U) { repeat = 1U; }
        if (repeat > 4U) { repeat = 4U; }
        for (uint32_t k = 0U; k < repeat && bit_count < 40U; k++) {
            bits[bit_count++] = level;
        }
        level ^= 1U;
    }

    if (bit_count < 20U) {
        return false;
    }

    uint16_t raw16 = 0U;
    for (uint8_t group = 0U; group < 4U; group++) {
        uint8_t code = 0U;
        for (uint8_t b = 0U; b < 5U; b++) {
            code = (uint8_t)((code << 1U) | bits[group * 5U + b]);
        }
        uint8_t nibble = gcr5b_to_nibble(code);
        if (nibble > 0x0F) {
            return false;
        }
        raw16 = (uint16_t)((raw16 << 4U) | nibble);
    }

    {
        uint16_t data12 = (uint16_t)(raw16 >> 4U);
        uint8_t crc_rx = (uint8_t)(raw16 & 0x0FU);
        uint8_t crc = esc_crc4_calc(data12);
        out->raw_word = raw16;
        out->data_12bit = data12;
        out->crc = crc_rx;
        out->crc_ok = (uint8_t)((crc_rx == crc) || (crc_rx == ((uint8_t)(~crc) & 0x0FU)));
        out->valid = out->crc_ok ? 1U : 0U;
        out->edge_count = rx->edge_count;
        out->timestamp_ms = HAL_GetTick();
        out->updated = 1U;
        return (out->valid != 0U);
    }
}

static void ESC_StopRxChannel(ESC_Channel_t ch)
{
    uint32_t tim_ch = (ch == ESC_CH1) ? TIM_CHANNEL_3 : TIM_CHANNEL_4;
    (void)HAL_TIM_IC_Stop_IT(&s_htim2, tim_ch);
    s_rx[ch].active = 0U;
    if (ch == ESC_CH1) {
        s_dma_busy &= (uint8_t)~DMA_BUSY_CH3_RX;
        ESC_PinToGpioLow_PA2();
    } else {
        s_dma_busy &= (uint8_t)~DMA_BUSY_CH4_RX;
        ESC_PinToGpioLow_PA3();
    }
    s_dbg.dma_busy = s_dma_busy;
}

static void ESC_FinishRxChannel(ESC_Channel_t ch)
{
    ESC_Telemetry_t telem = {0};
    (void)ESC_DecodeTelemetryFromEdges(&s_rx[ch], &telem);
    s_telem[ch] = telem;
    ESC_StopRxChannel(ch);
}

static void ESC_StartRxChannel(ESC_Channel_t ch)
{
    TIM_IC_InitTypeDef ic = {0};
    uint32_t tim_ch = (ch == ESC_CH1) ? TIM_CHANNEL_3 : TIM_CHANNEL_4;

    memset(&s_rx[ch], 0, sizeof(s_rx[ch]));
    s_rx[ch].active = 1U;
    s_rx[ch].start_cycles = Timing_GetCycles();
    s_rx[ch].start_ms = HAL_GetTick();

#if defined(STM32H743xx)
    /* Betaflight pwm_output_dshot_hal.c pwmDshotSetDirectionInput：H7 先 GPIO 推挽低速输出再配 IC，避免毛刺 */
    {
        GPIO_InitTypeDef gpio = {0};
        gpio.Pin   = (ch == ESC_CH1) ? GPIO_PIN_2 : GPIO_PIN_3;
        gpio.Mode  = GPIO_MODE_OUTPUT_PP;
        gpio.Pull  = GPIO_NOPULL;
        gpio.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(GPIOA, &gpio);
        HAL_GPIO_WritePin(GPIOA, gpio.Pin, GPIO_PIN_RESET);
    }
#endif

    ic.ICPolarity  = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
    ic.ICSelection = TIM_ICSELECTION_DIRECTTI;
    ic.ICPrescaler = TIM_ICPSC_DIV1;
    ic.ICFilter    = 2U; /* 与 BF LL_TIM_IC_StructInit + ICFilter=2 一致 */
    (void)HAL_TIM_IC_ConfigChannel(&s_htim2, &ic, tim_ch);
    /* 仅接收路 AF+上拉；另一路 GPIO 强低，减轻两线并行时的串扰 */
    if (ch == ESC_CH1) {
        ESC_PinsToAF_RxCh1_TxHoldCh2Low();
    } else {
        ESC_PinsToAF_RxCh2_TxHoldCh1Low();
    }
    (void)HAL_TIM_IC_Start_IT(&s_htim2, tim_ch);

    if (ch == ESC_CH1) {
        s_dma_busy |= DMA_BUSY_CH3_RX;
    } else {
        s_dma_busy |= DMA_BUSY_CH4_RX;
    }
    s_dbg.dma_busy = s_dma_busy;
}

static void ESC_StartPendingRxIfReady(void)
{
    if ((s_dma_busy & (DMA_BUSY_CH3_TX | DMA_BUSY_CH4_TX)) != 0U) {
        return;
    }
    if (s_rx_start_pending == 0U) {
        return;
    }

    ESC_Tim2ConfigRx();
    /* RX 引脚在 ESC_StartRxChannel 内配置：仅一路 AF+IC，另一路 GPIO 强低（减两线串扰） */

    if (s_rx_start_pending & DMA_BUSY_CH3_RX) {
        ESC_StartRxChannel(ESC_CH1);
    }
    if (s_rx_start_pending & DMA_BUSY_CH4_RX) {
        ESC_StartRxChannel(ESC_CH2);
    }
    s_rx_start_pending = 0U;
}

/**
 * 仅关闭单路 PWM+DMA，不执行 __HAL_TIM_DISABLE。
 * HAL_TIM_PWM_Stop_DMA() 会在任一路回调里关掉整颗 TIM，另一路 DMA 无法跑完。
 */
static void ESC_TimPwmStopDmaOneChKeepTimRunning(TIM_HandleTypeDef *htim, uint32_t Channel)
{
    switch (Channel) {
        case TIM_CHANNEL_3:
            __HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC3);
            (void)HAL_DMA_Abort_IT(htim->hdma[TIM_DMA_ID_CC3]);
            break;
        case TIM_CHANNEL_4:
            __HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC4);
            (void)HAL_DMA_Abort_IT(htim->hdma[TIM_DMA_ID_CC4]);
            break;
        default:
            return;
    }
    TIM_CCxChannelCmd(htim->Instance, Channel, TIM_CCx_DISABLE);
    if (IS_TIM_BREAK_INSTANCE(htim->Instance) != RESET) {
        __HAL_TIM_MOE_DISABLE(htim);
    }
}

static void ESC_Tim2DisableCounterIfTxDmaDoneMatchesExpect(void)
{
    if (s_esc_tx_expect_mask != 0U &&
        (s_esc_tx_dma_done_bits == s_esc_tx_expect_mask)) {
        __HAL_TIM_DISABLE(&s_htim2);
        s_esc_tx_dma_done_bits = 0U;
        s_esc_tx_expect_mask    = 0U;
    }
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
    for (uint32_t i = DSHOT_FRAME_BITS; i < DSHOT_FRAME_LEN; i++) {
        buf[i] = 0U;  /* 拖尾：CCR=0，线保持低 */
    }
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
#if ESC_BIDIR_DSHOT
    /* 双向：与 BF 一致上拉，便于电调开漏回传 */
    gpio.Pull      = GPIO_PULLUP;
#else
    /* 纯 DShot 发码：勿内部上拉，避免与推挽边沿叠加导致电调不识别 */
    gpio.Pull      = GPIO_NOPULL;
#endif
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
    HAL_NVIC_SetPriority(TIM2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
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

void TIM2_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&s_htim2);
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
 * 双通道 TX：每路完成时只关本路 DMA/CC，两路都完成后再 __HAL_TIM_DISABLE（见上）。
 * ════════════════════════════════════════════════════════════════ */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM2) { return; }

    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
        s_dbg.pulse_done_ch3_count++;
        ESC_TimPwmStopDmaOneChKeepTimRunning(htim, TIM_CHANNEL_3);
        s_dma_busy &= (uint8_t)~DMA_BUSY_CH3_TX;
        s_esc_tx_dma_done_bits |= ESC_TX_DMA_DONE_CH3;
        ESC_Tim2DisableCounterIfTxDmaDoneMatchesExpect();
        if (s_pending_tel[ESC_CH1]) {
            s_rx_start_pending |= DMA_BUSY_CH3_RX;
        } else {
            ESC_PinToGpioLow_PA2();
        }
    }
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
        s_dbg.pulse_done_ch4_count++;
        ESC_TimPwmStopDmaOneChKeepTimRunning(htim, TIM_CHANNEL_4);
        s_dma_busy &= (uint8_t)~DMA_BUSY_CH4_TX;
        s_esc_tx_dma_done_bits |= ESC_TX_DMA_DONE_CH4;
        ESC_Tim2DisableCounterIfTxDmaDoneMatchesExpect();
        if (s_pending_tel[ESC_CH2]) {
            s_rx_start_pending |= DMA_BUSY_CH4_RX;
        } else {
            ESC_PinToGpioLow_PA3();
        }
    }
    ESC_StartPendingRxIfReady();
    s_dbg.dma_busy = s_dma_busy;
}

void HAL_TIM_PWM_PulseFinishedHalfCpltCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM2) { return; }
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) { s_dbg.pulse_half_ch3_count++; }
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) { s_dbg.pulse_half_ch4_count++; }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    ESC_Channel_t ch;
    uint32_t cap;
    ESC_RxState_t *rx;
    uint32_t dt;

    if (htim->Instance != TIM2) { return; }
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
        ch = ESC_CH1;
        cap = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
    } else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
        ch = ESC_CH2;
        cap = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
    } else {
        return;
    }

    rx = &s_rx[ch];
    if (!rx->active) {
        return;
    }

    if (!rx->has_first_capture) {
        rx->has_first_capture = 1U;
        rx->last_capture = cap;
        return;
    }

    dt = cap - rx->last_capture;
    rx->last_capture = cap;

    if (rx->edge_count < ESC_RX_MAX_EDGES) {
        rx->edge_dt[rx->edge_count] = dt;
        rx->edge_count++;
    } else {
        ESC_FinishRxChannel(ch);
        return;
    }

    if (dt > 120U && rx->edge_count > 8U) { /* 10MHz 下约 12us，视为回传结束间隔 */
        ESC_FinishRxChannel(ch);
    }
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
    memset(s_rx,          0, sizeof(s_rx));
    memset(s_telem,       0, sizeof(s_telem));
    memset(s_service_cmd,   0, sizeof(s_service_cmd));
    memset(s_service_state, 0, sizeof(s_service_state));
    s_rx_start_pending = 0U;
    s_dma_busy = 0U;
    s_esc_tx_dma_done_bits = 0U;
    s_esc_tx_expect_mask    = 0U;

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
    ESC_Tim2ConfigTx();

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
    return ESC_SetThrottleBidirectionalEx(ch, throttle, false);
}

ESC_Status_t ESC_SetThrottleBidirectionalEx(ESC_Channel_t ch, int16_t throttle, bool request_telemetry)
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
    return ESC_SetThrottleRaw(ch, dshot_val, request_telemetry);
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
    s_rx_start_pending = 0U;
    s_esc_tx_dma_done_bits = 0U;
    s_esc_tx_expect_mask    = 0U;

    ESC_Tim2ConfigTx();

    {
        TIM_OC_InitTypeDef oc = {0};
        oc.OCMode     = TIM_OCMODE_PWM1;
        oc.Pulse      = 0U;
        oc.OCPolarity = TIM_OCPOLARITY_HIGH;
        oc.OCFastMode = TIM_OCFAST_DISABLE;
        (void)HAL_TIM_PWM_ConfigChannel(&s_htim2, &oc, TIM_CHANNEL_3);
        (void)HAL_TIM_PWM_ConfigChannel(&s_htim2, &oc, TIM_CHANNEL_4);
    }

    /* 每次发送前恢复为 TIM2 复用输出（上一次发送完成后会被切回 GPIO 低电平） */
    ESC_PinsToAF_TxBoth();

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
        s_dma_busy |= DMA_BUSY_CH3_TX;
        s_esc_tx_expect_mask |= ESC_TX_DMA_DONE_CH3;
    } else {
        s_dbg.start_dma_ch3_fail_count++;
    }

    /* 启动 DMA — CH4 (PA3)：ChannelState[3] 与 ChannelState[2] 独立，无冲突 */
    if (HAL_TIM_PWM_Start_DMA(&s_htim2, TIM_CHANNEL_4,
                               s_buf_ch4, DSHOT_FRAME_LEN) == HAL_OK) {
        s_dma_busy |= DMA_BUSY_CH4_TX;
        s_esc_tx_expect_mask |= ESC_TX_DMA_DONE_CH4;
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

bool ESC_GetTelemetry(ESC_Channel_t ch, ESC_Telemetry_t *out)
{
    if (ch >= ESC_CH_COUNT || out == NULL) {
        return false;
    }
    *out = s_telem[ch];
    s_telem[ch].updated = 0U;
    return (out->valid != 0U);
}

void ESC_Service_Init(void)
{
    for (uint8_t ch = 0U; ch < ESC_CH_COUNT; ch++) {
        s_service_cmd[ch].throttle = 0;
        s_service_cmd[ch].request_telemetry = 0U;
        s_service_state[ch].initialized = 1U;
        s_service_state[ch].last_update_ok = 1U;
    }
}

ESC_Status_t ESC_Service_WriteCommand(ESC_Channel_t ch, int16_t throttle, bool request_telemetry)
{
    if (ch >= ESC_CH_COUNT) {
        return ESC_ERR_PARAM;
    }
    if (throttle < 0) {
        throttle = 0;
    } else if (throttle > 1000) {
        throttle = 1000;
    }
    s_service_cmd[ch].throttle = throttle;
    s_service_cmd[ch].request_telemetry = request_telemetry ? 1U : 0U;
    return ESC_OK;
}

void ESC_Service_Tick(void)
{
    ESC_Status_t st = ESC_OK;
#if ESC_BIDIR_DSHOT
    s_bidir_tel_frame++;
#endif

    for (uint8_t ch = 0U; ch < ESC_CH_COUNT; ch++) {
        s_service_state[ch].tick_count++;
        bool req_tel = (s_service_cmd[ch].request_telemetry != 0U);
#if ESC_BIDIR_DSHOT
        if (req_tel) {
#if ESC_BIDIR_TELEM_EVERY_N > 1U
            if ((s_bidir_tel_frame % ESC_BIDIR_TELEM_EVERY_N) != 0U) {
                req_tel = false;
            }
#endif
#if ESC_BIDIR_TELEM_ALTERNATE
            if (req_tel) {
                uint32_t n_div = (ESC_BIDIR_TELEM_EVERY_N > 1U) ? ESC_BIDIR_TELEM_EVERY_N : 1U;
                uint32_t blk = s_bidir_tel_frame / n_div;
                req_tel = (((uint32_t)ch ^ blk) & 1U) == 0U;
            }
#endif
        }
#endif
        st = ESC_SetThrottleBidirectionalEx((ESC_Channel_t)ch,
                                            s_service_cmd[ch].throttle,
                                            req_tel);
        if (st != ESC_OK) {
            s_service_state[ch].last_update_ok = 0U;
            s_service_state[ch].update_err_count++;
        }
    }

    ESC_RecoverIfStuck(2U);
    if (!ESC_IsBusy()) {
        st = ESC_Update();
        for (uint8_t ch = 0U; ch < ESC_CH_COUNT; ch++) {
            if (st == ESC_OK) {
                s_service_state[ch].last_update_ok = 1U;
                s_service_state[ch].update_ok_count++;
            } else {
                s_service_state[ch].last_update_ok = 0U;
                s_service_state[ch].update_err_count++;
            }
        }
    }

    for (uint8_t ch = 0U; ch < ESC_CH_COUNT; ch++) {
        ESC_Telemetry_t t;
        if (ESC_GetTelemetry((ESC_Channel_t)ch, &t) && t.updated) {
            s_service_state[ch].last_telemetry = t;
            s_service_state[ch].rx_ok_count++;
        } else if (t.updated) {
            s_service_state[ch].last_telemetry = t;
            s_service_state[ch].rx_err_count++;
        }
    }
}

bool ESC_Service_ReadState(ESC_Channel_t ch, ESC_ServiceState_t *out)
{
    if (ch >= ESC_CH_COUNT || out == NULL) {
        return false;
    }
    *out = s_service_state[ch];
    return true;
}

void ESC_RecoverIfStuck(uint32_t timeout_ms)
{
    static uint32_t s_stuck_since_ms = 0U;
    uint32_t now = HAL_GetTick();

    /* 须先于 stuck 判定执行：微秒级结束无回传的 RX，尽快释放 BUSY */
    for (uint8_t ch = 0U; ch < ESC_CH_COUNT; ch++) {
        if (!s_rx[ch].active) {
            continue;
        }
        uint32_t dt_cyc = Timing_GetCycles() - s_rx[ch].start_cycles;
        if (Timing_CyclesToUs(dt_cyc) >= ESC_RX_TIMEOUT_US) {
            ESC_FinishRxChannel((ESC_Channel_t)ch);
        }
    }

    if (!s_dma_busy) {
        s_stuck_since_ms = 0U;
        return;
    }

    if (s_stuck_since_ms == 0U) {
        s_stuck_since_ms = now;
        return;
    }

    if ((now - s_stuck_since_ms) >= timeout_ms) {
        /* 强制停止：清 CCxDE、中止 DMA、禁用通道输出、关闭计数器 */
        HAL_TIM_PWM_Stop_DMA(&s_htim2, TIM_CHANNEL_3);
        HAL_TIM_PWM_Stop_DMA(&s_htim2, TIM_CHANNEL_4);
        ESC_StopRxChannel(ESC_CH1);
        ESC_StopRxChannel(ESC_CH2);
        s_rx_start_pending       = 0U;
        s_esc_tx_dma_done_bits   = 0U;
        s_esc_tx_expect_mask     = 0U;
        s_dma_busy               = 0U;
        s_dbg.dma_busy           = 0U;
        s_dbg.stuck_recover_count++;
        s_stuck_since_ms         = 0U;
    }
}
