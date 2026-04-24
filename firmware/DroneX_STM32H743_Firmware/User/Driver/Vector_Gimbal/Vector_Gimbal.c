#include "Driver/Vector_Gimbal/Vector_Gimbal.h"
#include "stm32h7xx_hal.h"

#define GIMBAL_FREQ_HZ          333U
#define GIMBAL_TIMER_TICK_HZ    1000000U /* 1 tick = 1us */
#define GIMBAL_OUTPUT_MIN_US    1000U
#define GIMBAL_OUTPUT_MAX_US    2000U
#define GIMBAL_CMD_MIN          (-10000)
#define GIMBAL_CMD_MAX          10000
#define GIMBAL_NORM_MIN         (-1.0f)
#define GIMBAL_NORM_MAX         1.0f
#define GIMBAL_FILTER_TIME_MS   50U

static TIM_HandleTypeDef s_htim4;

typedef struct {
    uint32_t channel;
    uint16_t pulse_min_us;
    uint16_t pulse_mid_us;
    uint16_t pulse_max_us;
    int8_t direction;
} VectorGimbal_AxisConfig_t;

typedef struct {
    float filtered_pulse_us;
    uint32_t last_update_ms;
    uint8_t initialized;
} VectorGimbal_AxisState_t;

/*
 * Per-axis servo settings.
 * Keep defaults identical to previous behavior; each axis can be tuned independently.
 */
static const VectorGimbal_AxisConfig_t s_axis_cfg[VECTOR_GIMBAL_AXIS_COUNT] = {
    /* VECTOR_GIMBAL_AXIS_X */ { TIM_CHANNEL_4, 1000U, 1500U, 2000U,  -1 },
    /* VECTOR_GIMBAL_AXIS_Y */ { TIM_CHANNEL_3, 1000U, 1500U, 2000U,  1 },
};

static VectorGimbal_AxisState_t s_axis_state[VECTOR_GIMBAL_AXIS_COUNT];
static uint16_t s_filter_time_ms = GIMBAL_FILTER_TIME_MS;

static float clamp_norm(float value)
{
    if (value != value) {
        return 0.0f;
    }
    if (value < GIMBAL_NORM_MIN) {
        return GIMBAL_NORM_MIN;
    }
    if (value > GIMBAL_NORM_MAX) {
        return GIMBAL_NORM_MAX;
    }
    return value;
}

static uint16_t clamp_output_pulse_us(uint16_t pulse)
{
    if (pulse < GIMBAL_OUTPUT_MIN_US) {
        return GIMBAL_OUTPUT_MIN_US;
    }
    if (pulse > GIMBAL_OUTPUT_MAX_US) {
        return GIMBAL_OUTPUT_MAX_US;
    }
    return pulse;
}

static uint16_t filter_output_pulse_us(VectorGimbal_Axis_t axis, uint16_t target_pulse)
{
    VectorGimbal_AxisState_t *state = &s_axis_state[axis];
    uint32_t now_ms = HAL_GetTick();
    uint32_t dt_ms;
    float filtered;
    float alpha;

    target_pulse = clamp_output_pulse_us(target_pulse);

    if ((state->initialized == 0U) || (s_filter_time_ms == 0U)) {
        state->filtered_pulse_us = (float)target_pulse;
        state->last_update_ms = now_ms;
        state->initialized = 1U;
        return target_pulse;
    }

    dt_ms = now_ms - state->last_update_ms;
    if (dt_ms == 0U) {
        return clamp_output_pulse_us((uint16_t)(state->filtered_pulse_us + 0.5f));
    }

    alpha = (float)dt_ms / ((float)s_filter_time_ms + (float)dt_ms);
    filtered = state->filtered_pulse_us + alpha * ((float)target_pulse - state->filtered_pulse_us);
    state->filtered_pulse_us = filtered;
    state->last_update_ms = now_ms;

    return clamp_output_pulse_us((uint16_t)(filtered + 0.5f));
}

static void set_axis_pulse(VectorGimbal_Axis_t axis, uint16_t target_pulse)
{
    uint16_t pulse = filter_output_pulse_us(axis, target_pulse);

    __HAL_TIM_SET_COMPARE(&s_htim4, s_axis_cfg[axis].channel, pulse);
}

/* Raw command mapping: input command is [-10000, 10000]. */
static uint16_t map_cmd_to_pulse_us(int16_t value, const VectorGimbal_AxisConfig_t *cfg)
{
    int32_t v = value;
    int32_t pulse;

    if (v < GIMBAL_CMD_MIN) v = GIMBAL_CMD_MIN;
    if (v > GIMBAL_CMD_MAX) v = GIMBAL_CMD_MAX;
    v *= (int32_t)cfg->direction;

    if (v >= 0) {
        pulse = (int32_t)cfg->pulse_mid_us
              + (v * ((int32_t)cfg->pulse_max_us - (int32_t)cfg->pulse_mid_us)) / GIMBAL_CMD_MAX;
    } else {
        pulse = (int32_t)cfg->pulse_mid_us
              + (v * ((int32_t)cfg->pulse_mid_us - (int32_t)cfg->pulse_min_us)) / (-GIMBAL_CMD_MIN);
    }

    if (pulse < (int32_t)cfg->pulse_min_us) pulse = cfg->pulse_min_us;
    if (pulse > (int32_t)cfg->pulse_max_us) pulse = cfg->pulse_max_us;

    return clamp_output_pulse_us((uint16_t)pulse);
}

/* Normalized mapping: -1 -> min, 0 -> mid, 1 -> max. */
static uint16_t map_norm_to_pulse_us(float value, const VectorGimbal_AxisConfig_t *cfg)
{
    float v = clamp_norm(value) * (float)cfg->direction;
    float pulse;

    if (v >= 0.0f) {
        pulse = (float)cfg->pulse_mid_us
              + v * ((float)cfg->pulse_max_us - (float)cfg->pulse_mid_us);
    } else {
        pulse = (float)cfg->pulse_mid_us
              + v * ((float)cfg->pulse_mid_us - (float)cfg->pulse_min_us);
    }

    if (pulse < (float)cfg->pulse_min_us) pulse = (float)cfg->pulse_min_us;
    if (pulse > (float)cfg->pulse_max_us) pulse = (float)cfg->pulse_max_us;

    return clamp_output_pulse_us((uint16_t)(pulse + 0.5f));
}

void Vector_Gimbal_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    TIM_OC_InitTypeDef oc = {0};
    uint32_t period_ticks = (GIMBAL_TIMER_TICK_HZ / GIMBAL_FREQ_HZ);
    uint32_t i;

    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();

    gpio.Pin = GPIO_PIN_14 | GPIO_PIN_15;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOD, &gpio);

    if (period_ticks == 0U) {
        period_ticks = 1U;
    }

    s_htim4.Instance = TIM4;
    s_htim4.Init.Prescaler = 239U; /* 240MHz / (239+1) = 1MHz */
    s_htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    s_htim4.Init.Period = period_ticks - 1U; /* about 333Hz */
    s_htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    s_htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&s_htim4) != HAL_OK) {
        return;
    }

    oc.OCMode = TIM_OCMODE_PWM1;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;

    for (i = 0U; i < (uint32_t)VECTOR_GIMBAL_AXIS_COUNT; i++) {
        oc.Pulse = clamp_output_pulse_us(s_axis_cfg[i].pulse_mid_us);
        s_axis_state[i].filtered_pulse_us = (float)oc.Pulse;
        s_axis_state[i].last_update_ms = HAL_GetTick();
        s_axis_state[i].initialized = 1U;
        if (HAL_TIM_PWM_ConfigChannel(&s_htim4, &oc, s_axis_cfg[i].channel) != HAL_OK) {
            return;
        }
    }

    for (i = 0U; i < (uint32_t)VECTOR_GIMBAL_AXIS_COUNT; i++) {
        (void)HAL_TIM_PWM_Start(&s_htim4, s_axis_cfg[i].channel);
    }
}

void Vector_Gimbal_SetFilterTimeMs(uint16_t time_ms)
{
    s_filter_time_ms = time_ms;
}

VectorGimbal_Status_t Vector_Gimbal_SetRaw(VectorGimbal_Axis_t axis, int16_t value)
{
    uint16_t pulse;

    if ((axis < 0) || (axis >= VECTOR_GIMBAL_AXIS_COUNT)) {
        return VECTOR_GIMBAL_ERR_PARAM;
    }

    pulse = map_cmd_to_pulse_us(value, &s_axis_cfg[axis]);
    set_axis_pulse(axis, pulse);

    return VECTOR_GIMBAL_OK;
}

VectorGimbal_Status_t Vector_Gimbal_SetNormalized(VectorGimbal_Axis_t axis, float value)
{
    uint16_t pulse;

    if ((axis < 0) || (axis >= VECTOR_GIMBAL_AXIS_COUNT)) {
        return VECTOR_GIMBAL_ERR_PARAM;
    }

    pulse = map_norm_to_pulse_us(value, &s_axis_cfg[axis]);
    set_axis_pulse(axis, pulse);

    return VECTOR_GIMBAL_OK;
}

VectorGimbal_Status_t Vector_Gimbal_Set(VectorGimbal_Axis_t axis, int16_t value)
{
    return Vector_Gimbal_SetRaw(axis, value);
}
