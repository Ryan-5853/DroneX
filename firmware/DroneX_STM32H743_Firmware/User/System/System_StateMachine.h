#ifndef __SYSTEM_STATE_MACHINE_H__
#define __SYSTEM_STATE_MACHINE_H__

#include <stdbool.h>
#include <stdint.h>

#include "System/System_Def.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 状态机输入条件（由 Manager 每周期填充）
 *
 * 说明：
 * - 上电阶段仅使用 self_check_passed/self_check_failed 完成首跳。
 * - arm_condition_ok 表示“解锁姿态条件满足”（通道组合、油门位置等）。
 * - cmd_to_* 为模式切换命令（来自遥控器拨杆或 CLI）。
 * - 任意保护事件触发后，状态机应锁定到 PROTECT。
 */
typedef struct {
    bool self_check_passed;
    bool self_check_failed;
    bool arm_condition_ok;
    bool cmd_to_ready;
    bool cmd_to_rc;
    bool cmd_to_run;
    bool cmd_to_standby;
    bool estop;
    bool telemetry_abnormal;
    bool hardware_abnormal;
    bool cli_reboot_request;
} System_SM_Input_t;

/**
 * @brief 状态机输出信息（用于日志、遥测）
 */
typedef struct {
    System_Mode_t mode;
    System_TransitionReason_t last_reason;
    uint8_t power_unit_activated;
    uint8_t power_output_enabled;
    uint8_t reboot_required;
} System_SM_Output_t;

/**
 * @brief 初始化状态机（上电默认待命前阶段）。
 */
void System_SM_Init(void);

/**
 * @brief 状态机周期更新。
 */
void System_SM_Update(const System_SM_Input_t *in);

/**
 * @brief 强制进入保护锁定状态。
 */
void System_SM_ForceProtect(System_TransitionReason_t reason);

/**
 * @brief 获取状态机完整输出。
 */
void System_SM_GetOutput(System_SM_Output_t *out);

/**
 * @brief 兼容接口：仅获取 mode。
 */
System_Mode_t System_SM_GetMode(void);

/**
 * @brief 兼容接口：返回动力单元是否激活。
 */
bool System_SM_IsArmed(void);

#ifdef __cplusplus
}
#endif

#endif /* __SYSTEM_STATE_MACHINE_H__ */

