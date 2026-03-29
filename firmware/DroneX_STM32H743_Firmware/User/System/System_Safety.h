#ifndef __SYSTEM_SAFETY_H__
#define __SYSTEM_SAFETY_H__

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 安全模块初始化。
 */
void System_Safety_Init(void);

/**
 * @brief 安全模块周期更新（故障检测与位图汇总）。
 * @param now_ms 当前系统毫秒计数。
 */
void System_Safety_Update(uint32_t now_ms);

/**
 * @brief 获取当前故障位图。
 */
uint32_t System_Safety_GetFaultFlags(void);

/**
 * @brief 是否触发保护类故障（fault_flags != 0）。
 */
bool System_Safety_IsTriggered(void);

#ifdef __cplusplus
}
#endif

#endif /* __SYSTEM_SAFETY_H__ */

