/**
 * @file PIT.c
 * @brief PIT 驱动实现。通道映射：CH0→TIM6, CH1→TIM7, CH2→TIM13, CH3→TIM14, CH4→TIM16, CH5→TIM17
 *        TIM6/7/13/14 挂 APB1，TIM16/17 挂 APB2，时钟在运行时根据 RCC 计算
 */

#include "PIT.h"
#include "main.h"

/* ----------------------------------------------------------------------------
 * 通道与 TIM 映射
 * ---------------------------------------------------------------------------- */
typedef struct {
    TIM_TypeDef    *instance;
    IRQn_Type       irqn;
    uint8_t         is_apb2;   /* 1=APB2, 0=APB1 */
} PIT_TimMap_t;

static const PIT_TimMap_t s_tim_map[] = {
    { TIM6,  TIM6_DAC_IRQn,           0 },
    { TIM7,  TIM7_IRQn,               0 },
    { TIM13, TIM8_UP_TIM13_IRQn,      0 },
    { TIM14, TIM8_TRG_COM_TIM14_IRQn, 0 },
    { TIM16, TIM16_IRQn,             1 },
    { TIM17, TIM17_IRQn,             1 },
};

static void PIT_ClkEnable(TIM_TypeDef *inst)
{
    if (inst == TIM6)  __HAL_RCC_TIM6_CLK_ENABLE();
    else if (inst == TIM7)  __HAL_RCC_TIM7_CLK_ENABLE();
    else if (inst == TIM13) __HAL_RCC_TIM13_CLK_ENABLE();
    else if (inst == TIM14) __HAL_RCC_TIM14_CLK_ENABLE();
    else if (inst == TIM16) __HAL_RCC_TIM16_CLK_ENABLE();
    else if (inst == TIM17) __HAL_RCC_TIM17_CLK_ENABLE();
}

/* ----------------------------------------------------------------------------
 * 通道状态
 * ---------------------------------------------------------------------------- */
typedef struct {
    TIM_HandleTypeDef  htim;
    PIT_Callback_t     callback;
    uint8_t           priority;
    bool              initialized;
} PIT_ChannelState_t;

static PIT_ChannelState_t s_ch[PIT_CH_COUNT];
#define PIT_CH_TO_INDEX(ch) ((uint8_t)(ch))

/** 根据 TIM 实例查找通道索引 */
static int8_t PIT_FindChannelByInstance(TIM_TypeDef *inst)
{
    for (uint8_t i = 0; i < PIT_CH_COUNT; i++) {
        if (s_tim_map[i].instance == inst)
            return (int8_t)i;
    }
    return -1;
}

/* ----------------------------------------------------------------------------
 * 定时器时钟：APB prescaler!=1 时定时器时钟=2*PCLK
 * ---------------------------------------------------------------------------- */
static uint32_t PIT_GetTimerClockHz(uint8_t is_apb2)
{
    RCC_ClkInitTypeDef clk;
    uint32_t lat;
    HAL_RCC_GetClockConfig(&clk, &lat);
    uint32_t pclk = is_apb2 ? HAL_RCC_GetPCLK2Freq() : HAL_RCC_GetPCLK1Freq();
    uint32_t div = is_apb2 ? clk.APB2CLKDivider : clk.APB1CLKDivider;
    return (div == RCC_HCLK_DIV1) ? pclk : (2U * pclk);
}

/* ----------------------------------------------------------------------------
 * 定时器周期计算
 * ---------------------------------------------------------------------------- */
static void PIT_ComputePrescalerPeriod(uint32_t period_us, uint32_t clk_hz, uint32_t *prescaler, uint32_t *period)
{
    uint32_t total_ticks = (uint32_t)((uint64_t)period_us * clk_hz / 1000000ULL);
    if (total_ticks == 0) total_ticks = 1;

    if (total_ticks > 65536U) {
        *prescaler = (total_ticks - 1) / 65536;
        *period = (total_ticks / (*prescaler + 1)) - 1;
    } else {
        *prescaler = 0;
        *period = total_ticks - 1;
    }
    /* 16-bit 限制 */
    if (*prescaler > 65535) *prescaler = 65535;
    if (*period > 65535) *period = 65535;
}

/* ----------------------------------------------------------------------------
 * HAL MSP 回调：由 HAL_TIM_Base_Init 触发
 * ---------------------------------------------------------------------------- */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
    int8_t idx = PIT_FindChannelByInstance(htim->Instance);
    if (idx < 0) return;

    const PIT_TimMap_t *m = &s_tim_map[idx];
    PIT_ClkEnable(m->instance);
    HAL_NVIC_SetPriority(m->irqn, s_ch[idx].priority, 0);
    HAL_NVIC_EnableIRQ(m->irqn);
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef *htim)
{
    int8_t idx = PIT_FindChannelByInstance(htim->Instance);
    if (idx < 0) return;

    const PIT_TimMap_t *m = &s_tim_map[idx];
    HAL_NVIC_DisableIRQ(m->irqn);
    if (htim->Instance == TIM6)  __HAL_RCC_TIM6_CLK_DISABLE();
    else if (htim->Instance == TIM7)  __HAL_RCC_TIM7_CLK_DISABLE();
    else if (htim->Instance == TIM13) __HAL_RCC_TIM13_CLK_DISABLE();
    else if (htim->Instance == TIM14) __HAL_RCC_TIM14_CLK_DISABLE();
    else if (htim->Instance == TIM16) __HAL_RCC_TIM16_CLK_DISABLE();
    else if (htim->Instance == TIM17) __HAL_RCC_TIM17_CLK_DISABLE();
}

/* ----------------------------------------------------------------------------
 * 公共接口
 * ---------------------------------------------------------------------------- */
PIT_Status_t PIT_Init(PIT_Channel_t ch, const PIT_Config_t *cfg)
{
    if (ch >= PIT_CH_COUNT || cfg == NULL)
        return PIT_ERR_PARAM;
    if (cfg->period_us == 0)
        return PIT_ERR_PARAM;
    if (s_ch[PIT_CH_TO_INDEX(ch)].initialized)
        return PIT_ERR_BUSY;

    uint8_t idx = PIT_CH_TO_INDEX(ch);
    PIT_ChannelState_t *st = &s_ch[idx];
    const PIT_TimMap_t *m = &s_tim_map[idx];

    st->callback = cfg->callback;
    st->priority = cfg->priority > 15 ? 15 : cfg->priority;

    uint32_t clk_hz = PIT_GetTimerClockHz(m->is_apb2);
    uint32_t prescaler, period;
    PIT_ComputePrescalerPeriod(cfg->period_us, clk_hz, &prescaler, &period);

    st->htim.Instance = m->instance;
    st->htim.Init.Prescaler = prescaler;
    st->htim.Init.Period = period;
    st->htim.Init.CounterMode = TIM_COUNTERMODE_UP;
    st->htim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    st->htim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    st->htim.Init.RepetitionCounter = 0;

    if (HAL_TIM_Base_Init(&st->htim) != HAL_OK)
        return PIT_ERR_HAL;

    st->initialized = true;
    return PIT_OK;
}

PIT_Status_t PIT_Start(PIT_Channel_t ch)
{
    if (ch >= PIT_CH_COUNT)
        return PIT_ERR_PARAM;
    if (!s_ch[PIT_CH_TO_INDEX(ch)].initialized)
        return PIT_ERR_BUSY;

    if (HAL_TIM_Base_Start_IT(&s_ch[PIT_CH_TO_INDEX(ch)].htim) != HAL_OK)
        return PIT_ERR_HAL;
    return PIT_OK;
}

PIT_Status_t PIT_Stop(PIT_Channel_t ch)
{
    if (ch >= PIT_CH_COUNT)
        return PIT_ERR_PARAM;

    HAL_TIM_Base_Stop_IT(&s_ch[PIT_CH_TO_INDEX(ch)].htim);
    return PIT_OK;
}

PIT_Status_t PIT_DeInit(PIT_Channel_t ch)
{
    if (ch >= PIT_CH_COUNT)
        return PIT_ERR_PARAM;

    uint8_t idx = PIT_CH_TO_INDEX(ch);
    HAL_TIM_Base_Stop_IT(&s_ch[idx].htim);
    if (HAL_TIM_Base_DeInit(&s_ch[idx].htim) != HAL_OK)
        return PIT_ERR_HAL;
    s_ch[idx].initialized = false;
    return PIT_OK;
}

/* ----------------------------------------------------------------------------
 * HAL 周期回调：在 TIMx_IRQHandler -> HAL_TIM_IRQHandler 中触发
 * ---------------------------------------------------------------------------- */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    int8_t idx = PIT_FindChannelByInstance(htim->Instance);
    if (idx >= 0 && s_ch[idx].callback != NULL)
        s_ch[idx].callback();
}

/* ----------------------------------------------------------------------------
 * 中断向量：由 PIT 占用的 TIM 统一在此处理
 * TIM13/14 与 TIM8 共享向量，仅处理 PIT 使用的 TIM
 * ---------------------------------------------------------------------------- */
void TIM6_DAC_IRQHandler(void)
{
    if (s_ch[0].initialized)
        HAL_TIM_IRQHandler(&s_ch[0].htim);
}

void TIM7_IRQHandler(void)
{
    if (s_ch[1].initialized)
        HAL_TIM_IRQHandler(&s_ch[1].htim);
}

void TIM8_UP_TIM13_IRQHandler(void)
{
    if (s_ch[2].initialized)
        HAL_TIM_IRQHandler(&s_ch[2].htim);
}

void TIM8_TRG_COM_TIM14_IRQHandler(void)
{
    if (s_ch[3].initialized)
        HAL_TIM_IRQHandler(&s_ch[3].htim);
}

void TIM16_IRQHandler(void)
{
    if (s_ch[4].initialized)
        HAL_TIM_IRQHandler(&s_ch[4].htim);
}

void TIM17_IRQHandler(void)
{
    if (s_ch[5].initialized)
        HAL_TIM_IRQHandler(&s_ch[5].htim);
}
