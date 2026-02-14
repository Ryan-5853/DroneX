/**
 * @file Timing.h
 * @brief 基于 DWT 周期计数器的微秒级计时。
 */

#ifndef __TIMING_H__
#define __TIMING_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 DWT 周期计数器（在系统时钟配置完成后调用）。
 */
void Timing_Init(void);

/**
 * @brief 获取当前周期计数值。
 */
uint32_t Timing_GetCycles(void);

/**
 * @brief 将周期数转换为微秒（依赖 SystemCoreClock）。
 */
uint32_t Timing_CyclesToUs(uint32_t cycles);

#ifdef __cplusplus
}
#endif

#endif /* __TIMING_H__ */
