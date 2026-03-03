/**
 * @file ESC_configurer.c
 * @brief ESC 穿透调试模式实现。
 *        Debug 层模块，与 CMD 平级。
 *
 * 工作流程：
 *   1. 收到 num=1/2 → NVIC 屏蔽所有 PIT 定时器中断向量，仅保留 USART1 调试通信
 *   2. 初始化 PD5(推挽输出)、PD6(输入)、PA2(上拉开漏输出)
 *   3. 进入阻塞轮询：PD6→PA2 信号穿透，PA2→PD5 电平回读
 *   4. 循环内维持 Debug_UART_Process / Debug_Process 以接收退出指令
 *   5. 收到 num=0 → 标志位清零，退出循环，反初始化 IO，NVIC 恢复中断
 *
 * 屏蔽策略：直接 HAL_NVIC_DisableIRQ 屏蔽定时器中断向量，
 *           定时器本身继续运行不受影响，恢复时 EnableIRQ 即可无缝恢复。
 */

#include "Debug/ESC_configurer/ESC_configurer.h"
#include "Driver/Debug/Debug.h"
#include "Driver/Debug/Debug_UART.h"
#include "main.h"

/* PIT 通道对应的 TIM 实例与 NVIC 向量（与 PIT.c s_tim_map 一致） */
typedef struct {
    TIM_TypeDef *tim;
    IRQn_Type    irqn;
} ESC_TimIRQ_t;

static const ESC_TimIRQ_t s_pit_map[] = {
    { TIM6,  TIM6_DAC_IRQn           },
    { TIM7,  TIM7_IRQn               },
    { TIM13, TIM8_UP_TIM13_IRQn      },
    { TIM14, TIM8_TRG_COM_TIM14_IRQn },
    { TIM16, TIM16_IRQn              },
    { TIM17, TIM17_IRQn              },
};
#define PIT_MAP_COUNT  (sizeof(s_pit_map) / sizeof(s_pit_map[0]))

/* 穿透调试状态 */
static volatile uint8_t s_esc_debug_active;
static int              s_esc_channel;

/* 进入前各 IRQ 的启用状态快照，退出时只恢复原本启用的 */
static uint8_t s_irq_was_enabled[PIT_MAP_COUNT];

/* -----------------------------------------------------------------------
 * GPIO 初始化 / 反初始化
 * ----------------------------------------------------------------------- */
static void ESC_GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};

    /* PD5: 推挽输出 */
    gpio.Pin   = GPIO_PIN_5;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOD, &gpio);

    /* PD6: 输入 */
    gpio.Pin   = GPIO_PIN_6;
    gpio.Mode  = GPIO_MODE_INPUT;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOD, &gpio);

    /* PA2: 上拉开漏输出 */
    gpio.Pin   = GPIO_PIN_2;
    gpio.Mode  = GPIO_MODE_OUTPUT_OD;
    gpio.Pull  = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio);
}

static void ESC_GPIO_DeInit(void)
{
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_5 | GPIO_PIN_6);
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2);
}

/* -----------------------------------------------------------------------
 * NVIC 批量屏蔽 / 恢复 PIT 定时器中断向量
 * ----------------------------------------------------------------------- */
static void ESC_DisablePIT_IRQ(void)
{
    for (uint32_t i = 0; i < PIT_MAP_COUNT; i++) {
        s_irq_was_enabled[i] = NVIC_GetEnableIRQ(s_pit_map[i].irqn) ? 1U : 0U;
        HAL_NVIC_DisableIRQ(s_pit_map[i].irqn);
    }
}

static void ESC_EnablePIT_IRQ(void)
{
    for (uint32_t i = 0; i < PIT_MAP_COUNT; i++) {
        if (!s_irq_was_enabled[i])
            continue;
        /*
         * 三步清除，顺序不可调换：
         *   1. 清 TIM_SR：切断外设级中断源（UIF 等），否则硬件会立刻重新挂起 NVIC
         *   2. 清 NVIC pending：清除穿透期间累积的挂起位
         *   3. 使能 NVIC IRQ：此时无残留中断，ISR 等到下次真正溢出才触发
         */
        s_pit_map[i].tim->SR = 0U;
        HAL_NVIC_ClearPendingIRQ(s_pit_map[i].irqn);
        HAL_NVIC_EnableIRQ(s_pit_map[i].irqn);
    }
}

/* -----------------------------------------------------------------------
 * ESC_Config：服务函数入口
 * ----------------------------------------------------------------------- */
int ESC_Config(int num)
{
    if (num < 0 || num > 2)
        return -1;

    /* num=0：退出穿透模式（由阻塞循环内的 CMD 回调调用） */
    if (num == 0) {
        if (!s_esc_debug_active) {
            Debug_Printf("WARN not in ESC debug mode\r\n");
            return 0;
        }
        s_esc_debug_active = 0;
        return 0;
    }

    /* 防止重复进入 */
    if (s_esc_debug_active) {
        Debug_Printf("ERR already in ESC debug mode (ESC%d)\r\n", s_esc_channel);
        return -1;
    }

    s_esc_channel = num;

    /* NVIC 屏蔽所有 PIT 定时器中断，定时器照常运行但不触发 CPU */
    ESC_DisablePIT_IRQ();

    /* 初始化穿透 IO */
    ESC_GPIO_Init();

    Debug_Printf("ESC passthrough enter: ESC%d  PD6->PA2->PD5\r\n", num);
    Debug_UART_Flush();

    s_esc_debug_active = 1;

    /*
     * 阻塞式穿透轮询：
     *   PD6(输入) → PA2(开漏输出)：外部信号穿透至电调
     *   PA2(读回) → PD5(推挽输出)：电调实际电平回读
     *   同时维持调试串口服务，以接收 num=0 退出指令
     */
    while (s_esc_debug_active) {
        /* PD6 → PA2 */
        if (GPIOD->IDR & (1U << 6))
            GPIOA->BSRR = (1U << 2);
        else
            GPIOA->BSRR = (1U << 18);

        /* PA2 → PD5 */
        if (GPIOA->IDR & (1U << 2))
            GPIOD->BSRR = (1U << 5);
        else
            GPIOD->BSRR = (1U << 21);

        Debug_UART_Process();
        Debug_Process();
    }

    /* 退出穿透模式：反初始化 IO，NVIC 恢复定时器中断 */
    ESC_GPIO_DeInit();
    ESC_EnablePIT_IRQ();

    Debug_Printf("ESC passthrough exit: ESC%d\r\n", s_esc_channel);
    return 0;
}
