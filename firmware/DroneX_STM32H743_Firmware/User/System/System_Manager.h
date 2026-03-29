#ifndef __SYSTEM_MANAGER_H__
#define __SYSTEM_MANAGER_H__

#include "System/System_Def.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 系统管理器初始化。
 */
void System_Manager_Init(void);

/**
 * @brief 1ms 级调度入口（内部再分发 10ms/100ms 任务）。
 */
void System_Manager_Tick1ms(void);

/**
 * @brief 10ms 任务：状态输入采样、安全判定、状态机更新。
 */
void System_Manager_Tick10ms(void);

/**
 * @brief 100ms 任务：遥测打包与调试输出。
 */
void System_Manager_Tick100ms(void);

/**
 * @brief 读取系统状态快照。
 */
const System_State_t *System_Manager_GetState(void);

#ifdef __cplusplus
}
#endif

#endif /* __SYSTEM_MANAGER_H__ */

