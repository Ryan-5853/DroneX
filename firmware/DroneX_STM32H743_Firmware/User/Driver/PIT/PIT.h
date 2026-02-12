/**
 * @file PIT.h
 * @brief 周期中断定时器（PIT）驱动：将底层 TIM 抽象为可配置周期、优先级、回调的定时器通道。
 *        每个通道对应一个独立 TIM 实例，支持微秒级周期配置。
 */

#ifndef __PIT_H__
#define __PIT_H__

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ----------------------------------------------------------------------------
 * 配置
 * -------------------------------------------------------------------------- */
#ifndef PIT_CHANNEL_COUNT
#define PIT_CHANNEL_COUNT  6   /* 通道数量：TIM6/7/13/14/16/17 */
#endif
/* 时钟由运行时根据 RCC 配置计算，APB1/APB2 定时器时钟可能不同 */

/* ----------------------------------------------------------------------------
 * 类型定义
 * ---------------------------------------------------------------------------- */
typedef enum {
    PIT_OK = 0,
    PIT_ERR_PARAM,      /* 参数无效（周期超出范围、通道越界等） */
    PIT_ERR_BUSY,       /* 通道已初始化/运行中 */
    PIT_ERR_HAL,        /* HAL 返回错误 */
} PIT_Status_t;

typedef enum {
    PIT_CH0 = 0,
    PIT_CH1,
    PIT_CH2,
    PIT_CH3,
    PIT_CH4,
    PIT_CH5,
    PIT_CH_COUNT = PIT_CHANNEL_COUNT,
} PIT_Channel_t;

/** 周期完成回调：在中断上下文中调用，应保持简短 */
typedef void (*PIT_Callback_t)(void);

/** 通道配置 */
typedef struct {
    uint32_t     period_us;   /* 周期（微秒），最小 1us，最大由 prescaler/period 限制 */
    uint8_t      priority;    /* NVIC 抢占优先级 0~15，数值越小优先级越高 */
    PIT_Callback_t callback;  /* 周期到达时调用的回调，可为 NULL */
} PIT_Config_t;

/* ----------------------------------------------------------------------------
 * 接口
 * ---------------------------------------------------------------------------- */

/**
 * @brief 初始化指定通道的定时器（配置周期、优先级、回调）。
 * @param ch  通道号
 * @param cfg 配置结构体
 * @return PIT_OK 成功；否则返回错误码
 */
PIT_Status_t PIT_Init(PIT_Channel_t ch, const PIT_Config_t *cfg);

/**
 * @brief 启动指定通道的定时器中断。
 */
PIT_Status_t PIT_Start(PIT_Channel_t ch);

/**
 * @brief 停止指定通道的定时器中断。
 */
PIT_Status_t PIT_Stop(PIT_Channel_t ch);

/**
 * @brief 反初始化指定通道（停止定时器并释放资源）。
 */
PIT_Status_t PIT_DeInit(PIT_Channel_t ch);

/* ---------------------------------------------------------------------------
 * 使用说明：
 * 1. 在 User_Main_Init 中调用 PIT_Init(ch, &cfg) 配置周期、优先级、回调；
 * 2. 调用 PIT_Start(ch) 启动定时器；
 * 3. 回调在中断上下文中执行，应保持简短；
 * 4. 通道映射：CH0→TIM6, CH1→TIM7, CH2→TIM13, CH3→TIM14, CH4→TIM16, CH5→TIM17。
 *    TIM6/7/13/14 挂 APB1，TIM16/17 挂 APB2，时钟由 RCC 自动区分。
 *
 * 示例：
 *   static void my_1ms_cb(void) { ... }
 *   PIT_Config_t cfg = { .period_us = 1000, .priority = 5, .callback = my_1ms_cb };
 *   PIT_Init(PIT_CH0, &cfg);
 *   PIT_Start(PIT_CH0);
 * --------------------------------------------------------------------------- */

#ifdef __cplusplus
}
#endif

#endif /* __PIT_H__ */
