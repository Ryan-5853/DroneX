#include "Driver/Vector_Gimbal/Vector_Gimbal.h"
#include "stm32h7xx_hal.h"

#define GIMBAL_FREQ_HZ          333U
#define GIMBAL_TIMER_TICK_HZ    1000000U /* 1 tick = 1us */
#define GIMBAL_PULSE_MIN_US     500U
#define GIMBAL_PULSE_MID_US     1500U
#define GIMBAL_PULSE_MAX_US     2500U
#define GIMBAL_CMD_MIN          (-10000)
#define GIMBAL_CMD_MAX          10000

static TIM_HandleTypeDef s_htim4;

static uint16_t map_cmd_to_pulse_us(int16_t value)
{
    int32_t v = value;
    int32_t pulse;

    if (v < GIMBAL_CMD_MIN) v = GIMBAL_CMD_MIN;
    if (v > GIMBAL_CMD_MAX) v = GIMBAL_CMD_MAX;

    pulse = (int32_t)GIMBAL_PULSE_MID_US + (v * (int32_t)(GIMBAL_PULSE_MAX_US - GIMBAL_PULSE_MID_US)) / GIMBAL_CMD_MAX;
    if (pulse < (int32_t)GIMBAL_PULSE_MIN_US) pulse = GIMBAL_PULSE_MIN_US;
    if (pulse > (int32_t)GIMBAL_PULSE_MAX_US) pulse = GIMBAL_PULSE_MAX_US;

    return (uint16_t)pulse;
}

void Vector_Gimbal_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    TIM_OC_InitTypeDef oc = {0};
    uint32_t period_ticks = (GIMBAL_TIMER_TICK_HZ / GIMBAL_FREQ_HZ);

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
    oc.Pulse = GIMBAL_PULSE_MID_US;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&s_htim4, &oc, TIM_CHANNEL_3) != HAL_OK) {
        return;
    }
    if (HAL_TIM_PWM_ConfigChannel(&s_htim4, &oc, TIM_CHANNEL_4) != HAL_OK) {
        return;
    }

    (void)HAL_TIM_PWM_Start(&s_htim4, TIM_CHANNEL_3);
    (void)HAL_TIM_PWM_Start(&s_htim4, TIM_CHANNEL_4);
}

VectorGimbal_Status_t Vector_Gimbal_Set(VectorGimbal_Axis_t axis, int16_t value)
{
    uint32_t channel;
    uint16_t pulse = map_cmd_to_pulse_us(value);

    if (axis == VECTOR_GIMBAL_AXIS_0) {
        channel = TIM_CHANNEL_3;
    } else if (axis == VECTOR_GIMBAL_AXIS_1) {
        channel = TIM_CHANNEL_4;
    } else {
        return VECTOR_GIMBAL_ERR_PARAM;
    }

    __HAL_TIM_SET_COMPARE(&s_htim4, channel, pulse);
    return VECTOR_GIMBAL_OK;
}
