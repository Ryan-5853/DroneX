#ifndef __SYSTEM_DEBUG_H__
#define __SYSTEM_DEBUG_H__

#include <stdint.h>

#include "System/System_Def.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 系统调试模块初始化。
 */
void System_Debug_Init(void);

/**
 * @brief 根据状态变化输出系统日志。
 */
void System_Debug_Update(const System_State_t *state);

#ifdef __cplusplus
}
#endif

#endif /* __SYSTEM_DEBUG_H__ */

