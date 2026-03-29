#ifndef __SYSTEM_TELEMETRY_H__
#define __SYSTEM_TELEMETRY_H__

#include <stdbool.h>
#include <stdint.h>

#include "System/System_Def.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t timestamp_ms;
    System_Mode_t mode;
    uint32_t fault_flags;
    uint8_t armed;
    uint8_t safety_triggered;
    uint8_t power_unit_activated;
    uint8_t power_output_enabled;
    uint8_t reboot_required;
    uint8_t rc_online;
    int16_t throttle_cmd;
    uint32_t esc_update_err_count;
} System_TelemetryPacket_t;

/**
 * @brief 遥测模块初始化。
 */
void System_Telemetry_Init(void);

/**
 * @brief 按当前系统状态更新一帧遥测快照。
 */
void System_Telemetry_Update(const System_State_t *state, uint32_t now_ms);

/**
 * @brief 读取最近一帧遥测。
 */
bool System_Telemetry_GetLastPacket(System_TelemetryPacket_t *out);

#ifdef __cplusplus
}
#endif

#endif /* __SYSTEM_TELEMETRY_H__ */

