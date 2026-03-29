#ifndef __SYSTEM_DEF_H__
#define __SYSTEM_DEF_H__

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 系统宏观状态（规划版）
 *
 * 状态语义：
 * - STANDBY：待命，动力单元未激活。
 * - READY：就绪，动力单元已激活但不输出动力。
 * - ERROR：上电自检失败进入，不允许激活动力单元。
 * - RC：遥控模式，手动通道直通（油门/云台）。
 * - RUN：运行模式，留给自主决策飞行。
 * - PROTECT：保护锁定，触发后仅允许 CLI/断电重启恢复。
 */
typedef enum {
    SYS_MODE_STANDBY = 0,
    SYS_MODE_READY,
    SYS_MODE_ERROR,
    SYS_MODE_RC,
    SYS_MODE_RUN,
    SYS_MODE_PROTECT
} System_Mode_t;

/**
 * @brief 系统故障位图（可按需扩展）
 */
typedef enum {
    SYS_FAULT_NONE              = 0x00000000UL,
    SYS_FAULT_SELF_CHECK_FAIL   = 0x00000001UL,
    SYS_FAULT_RC_LOST           = 0x00000002UL,
    SYS_FAULT_TELEMETRY_ABN     = 0x00000004UL,
    SYS_FAULT_HW_ABN            = 0x00000008UL,
    SYS_FAULT_ESTOP             = 0x00000010UL,
    SYS_FAULT_ESC_ERROR         = 0x00000020UL,
    SYS_FAULT_INTERNAL          = 0x80000000UL
} System_FaultFlag_t;

/**
 * @brief 系统状态快照（供调试、遥测、上层决策读取）
 */
typedef struct {
    System_Mode_t mode;             /* 当前宏观状态 */
    uint32_t fault_flags;           /* 故障位图 */
    uint32_t uptime_ms;             /* 上电运行时间 */
    uint8_t armed;                  /* 兼容字段：等价于动力单元已激活 */
    uint8_t safety_triggered;       /* 兼容字段：任意保护类故障触发 */
    uint8_t power_unit_activated;   /* 动力单元激活（READY/RC/RUN） */
    uint8_t power_output_enabled;   /* 动力输出使能（RC/RUN） */
    uint8_t reboot_required;        /* 进入 PROTECT/ERROR 后需重启恢复 */
} System_State_t;

/**
 * @brief 状态迁移原因（便于日志与遥测说明）
 */
typedef enum {
    SYS_TRANS_REASON_NONE = 0,
    SYS_TRANS_REASON_BOOT_SELF_CHECK_PASS,
    SYS_TRANS_REASON_BOOT_SELF_CHECK_FAIL,
    SYS_TRANS_REASON_ARM_CONDITION_OK,
    SYS_TRANS_REASON_CMD_TO_READY,
    SYS_TRANS_REASON_CMD_TO_RC,
    SYS_TRANS_REASON_CMD_TO_RUN,
    SYS_TRANS_REASON_CMD_TO_STANDBY,
    SYS_TRANS_REASON_ESTOP,
    SYS_TRANS_REASON_TELEMETRY_ABN,
    SYS_TRANS_REASON_HW_ABN,
    SYS_TRANS_REASON_CLI_REBOOT
} System_TransitionReason_t;

/* 兼容旧枚举名，避免影响已接入代码。 */
#define SYS_MODE_FAULT    SYS_MODE_ERROR
#define SYS_MODE_ARMED    SYS_MODE_READY
#define SYS_MODE_FLIGHT   SYS_MODE_RUN
#define SYS_MODE_LANDING  SYS_MODE_STANDBY
#define SYS_MODE_INIT     SYS_MODE_STANDBY

#ifdef __cplusplus
}
#endif

#endif /* __SYSTEM_DEF_H__ */

