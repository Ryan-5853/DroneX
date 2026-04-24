#include "System/System_Manager.h"

#include <string.h>

#include "Driver/ESC/ESC.h"
#include "Driver/RC/RC.h"
#include "stm32h7xx_hal.h"
#include "System/System_Debug.h"
#include "System/System_Safety.h"
#include "System/System_StateMachine.h"
#include "System/System_Telemetry.h"

#define SYS_RC_ZERO_THR                 80
#define SYS_SWITCH_DOWN_THR             700
#define SYS_SWITCH_UP_THR              -700
#define SYS_SWITCH_MID_THR              200
#define SYS_SAFE_STEP_TIMEOUT_MS       1000U
#define SYS_ESC_THROTTLE_LIMIT          900
#define SYS_ESC_THROTTLE_LIMIT_LIM_DOWN 300

typedef enum {
    SYS_ARM_WAIT_SAFE_MID = 0,
    SYS_ARM_WAIT_SAFE_DOWN,
    SYS_ARM_WAIT_SAFE_UP
} System_ArmSeqState_t;

static System_State_t s_state;
static uint32_t s_last_tick_10ms = 0U;
static uint32_t s_last_tick_100ms = 0U;
static bool s_arm_unlocked = false;
static System_ArmSeqState_t s_arm_seq_state = SYS_ARM_WAIT_SAFE_MID;
static uint32_t s_arm_seq_deadline_ms = 0U;

static bool System_Manager_IsSwitchDown(int16_t value)
{
    return (value >= SYS_SWITCH_DOWN_THR);
}

static bool System_Manager_IsSwitchUp(int16_t value)
{
    return (value <= SYS_SWITCH_UP_THR);
}

static bool System_Manager_IsSwitchMid(int16_t value)
{
    return (value > -SYS_SWITCH_MID_THR) && (value < SYS_SWITCH_MID_THR);
}

static bool System_Manager_IsRcZero4(const RC_Signal_t *rc)
{
    if (rc == 0) {
        return false;
    }

    for (uint32_t i = 0U; i < 4U; i++) {
        if ((rc->ch[i] > SYS_RC_ZERO_THR) || (rc->ch[i] < -SYS_RC_ZERO_THR)) {
            return false;
        }
    }
    return true;
}

static int16_t System_Manager_MapRcToEscLimit(int16_t rc_norm, int16_t esc_limit)
{
    int32_t limit = esc_limit;
    int32_t throttle;

    if (limit < 0) {
        limit = 0;
    } else if (limit > 1000) {
        limit = 1000;
    }

    throttle = ((int32_t)rc_norm) * limit / 1000;
    if (throttle < 0) {
        throttle = 0;
    } else if (throttle > limit) {
        throttle = limit;
    }
    return (int16_t)throttle;
}

static int16_t System_Manager_GetEscLimit(const RC_Signal_t *rc)
{
    if ((rc != 0) && System_Manager_IsSwitchDown(rc->ch[6])) { /* CH7: LIM */
        return SYS_ESC_THROTTLE_LIMIT_LIM_DOWN;
    }
    return SYS_ESC_THROTTLE_LIMIT;
}

static void System_Manager_UpdateArmSequence(const RC_Signal_t *rc, uint32_t now_ms)
{
    bool base_condition_ok;
    bool ma_manual_down;
    bool lim_down;
    bool safe_mid;
    bool safe_down;
    bool safe_up;

    if (s_arm_unlocked) {
        return;
    }

    if (rc == 0) {
        s_arm_seq_state = SYS_ARM_WAIT_SAFE_MID;
        s_arm_seq_deadline_ms = 0U;
        return;
    }

    ma_manual_down = System_Manager_IsSwitchDown(rc->ch[4]); /* CH5: 最左 M/A */
    lim_down = System_Manager_IsSwitchDown(rc->ch[6]);       /* CH7: 次右 LIM */
    safe_mid = System_Manager_IsSwitchMid(rc->ch[7]);        /* CH8: 最右 SAFE */
    safe_down = System_Manager_IsSwitchDown(rc->ch[7]);
    safe_up = System_Manager_IsSwitchUp(rc->ch[7]);

    base_condition_ok = rc->is_online && System_Manager_IsRcZero4(rc) && ma_manual_down && lim_down;
    if (!base_condition_ok) {
        s_arm_seq_state = SYS_ARM_WAIT_SAFE_MID;
        s_arm_seq_deadline_ms = 0U;
        return;
    }

    switch (s_arm_seq_state) {
        case SYS_ARM_WAIT_SAFE_MID:
            if (safe_mid) {
                s_arm_seq_state = SYS_ARM_WAIT_SAFE_DOWN;
            }
            break;

        case SYS_ARM_WAIT_SAFE_DOWN:
            if (safe_down) {
                s_arm_seq_state = SYS_ARM_WAIT_SAFE_UP;
                s_arm_seq_deadline_ms = now_ms + SYS_SAFE_STEP_TIMEOUT_MS;
            } else if (!safe_mid) {
                s_arm_seq_state = SYS_ARM_WAIT_SAFE_MID;
            }
            break;

        case SYS_ARM_WAIT_SAFE_UP:
            if ((int32_t)(now_ms - s_arm_seq_deadline_ms) > 0) {
                s_arm_seq_state = SYS_ARM_WAIT_SAFE_MID;
                s_arm_seq_deadline_ms = 0U;
            } else if (safe_up) {
                s_arm_unlocked = true;
            }
            break;

        default:
            s_arm_seq_state = SYS_ARM_WAIT_SAFE_MID;
            s_arm_seq_deadline_ms = 0U;
            break;
    }
}

static void System_Manager_UpdateStateSnapshot(uint32_t now_ms)
{
    System_SM_Output_t sm_out;
    System_SM_GetOutput(&sm_out);

    s_state.mode = System_SM_GetMode();
    s_state.fault_flags = System_Safety_GetFaultFlags();
    s_state.uptime_ms = now_ms;
    s_state.armed = System_SM_IsArmed() ? 1U : 0U;
    s_state.safety_triggered = System_Safety_IsTriggered() ? 1U : 0U;
    s_state.power_unit_activated = sm_out.power_unit_activated;
    s_state.power_output_enabled = sm_out.power_output_enabled;
    s_state.reboot_required = sm_out.reboot_required;
}

void System_Manager_Init(void)
{
    memset(&s_state, 0, sizeof(s_state));
    System_SM_Init();
    System_Safety_Init();
    System_Telemetry_Init();
    System_Debug_Init();

    s_last_tick_10ms = HAL_GetTick();
    s_last_tick_100ms = s_last_tick_10ms;
    s_arm_unlocked = false;
    s_arm_seq_state = SYS_ARM_WAIT_SAFE_MID;
    s_arm_seq_deadline_ms = 0U;
    System_Manager_UpdateStateSnapshot(s_last_tick_10ms);
}

void System_Manager_Tick10ms(void)
{
    System_SM_Input_t in = {0};
    const RC_Signal_t *rc = RC_GetSignal();
    System_Mode_t current_mode = System_SM_GetMode();
    uint32_t now_ms = HAL_GetTick();

    System_Safety_Update(now_ms);

    /* TODO: 用真实自检流程填充。当前默认自检通过。 */
    in.self_check_passed = true;
    in.self_check_failed = false;

    if (rc != 0) {
        bool safe_down_now;
        bool force_to_standby = false;

        System_Manager_UpdateArmSequence(rc, now_ms);
        safe_down_now = System_Manager_IsSwitchDown(rc->ch[7]); /* CH8: SAFE */

        /* 已激活后，SAFE 一旦下拨，立即失活并回待命，后续需重新解锁。 */
        if (s_arm_unlocked && safe_down_now) {
            s_arm_unlocked = false;
            s_arm_seq_state = SYS_ARM_WAIT_SAFE_MID;
            s_arm_seq_deadline_ms = 0U;
            force_to_standby = true;
        }

        in.arm_condition_ok = s_arm_unlocked;
        /* cmd_to_ready 仅用于 STANDBY -> READY，避免在激活态持续为真造成 READY/RC/RUN 抖动。 */
        in.cmd_to_ready = s_arm_unlocked && (current_mode == SYS_MODE_STANDBY);

        /* 左侧 M/A: CH5 下位=手动直通，非下位=自动控制。 */
        in.cmd_to_rc = s_arm_unlocked && !force_to_standby && System_Manager_IsSwitchDown(rc->ch[4]);
        in.cmd_to_run = s_arm_unlocked && !force_to_standby && !System_Manager_IsSwitchDown(rc->ch[4]);
        in.cmd_to_standby = force_to_standby;
    }

    /* 保护条件占位：先用 Safety 汇总结果兜底。 */
    in.estop = false;
    in.telemetry_abnormal = false;
    in.hardware_abnormal = System_Safety_IsTriggered();
    in.cli_reboot_request = false;

    System_SM_Update(&in);
    System_Manager_UpdateStateSnapshot(now_ms);
}

void System_Manager_PowerControlTask(void)
{
    const RC_Signal_t *rc = RC_GetSignal();
    System_SM_Output_t sm_out;
    int16_t esc_throttle = 0;
    int16_t esc_limit = System_Manager_GetEscLimit(rc);

    System_SM_GetOutput(&sm_out);

    /* 未解锁/未使能输出时，不发 DShot 波形。 */
    if (sm_out.power_output_enabled == 0U) {
        return;
    }

    if (((sm_out.mode == SYS_MODE_RC) || (sm_out.mode == SYS_MODE_RUN)) &&
        (rc != 0) && (rc->is_online != 0U)) {
        esc_throttle = System_Manager_MapRcToEscLimit(rc->ch[2], esc_limit);
    } else {
        esc_throttle = 0;
    }

#if ESC_BIDIR_DSHOT
    const bool esc_req_telemetry = true;
#else
    const bool esc_req_telemetry = false;
#endif

    (void)ESC_Service_WriteCommand(ESC_CH1, esc_throttle, esc_req_telemetry);
    (void)ESC_Service_WriteCommand(ESC_CH2, esc_throttle, esc_req_telemetry);
    ESC_Service_Tick();
}

void System_Manager_Tick100ms(void)
{
    uint32_t now_ms = HAL_GetTick();
    System_Manager_UpdateStateSnapshot(now_ms);
    System_Telemetry_Update(&s_state, now_ms);
    System_Debug_Update(&s_state);
}

void System_Manager_Tick1ms(void)
{
    uint32_t now_ms = HAL_GetTick();

    if ((now_ms - s_last_tick_10ms) >= 10U) {
        s_last_tick_10ms = now_ms;
        System_Manager_Tick10ms();
    }

    if ((now_ms - s_last_tick_100ms) >= 100U) {
        s_last_tick_100ms = now_ms;
        System_Manager_Tick100ms();
    }
}

const System_State_t *System_Manager_GetState(void)
{
    return &s_state;
}

