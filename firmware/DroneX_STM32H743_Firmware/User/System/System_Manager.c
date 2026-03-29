#include "System/System_Manager.h"

#include <string.h>

#include "Driver/RC/RC.h"
#include "stm32h7xx_hal.h"
#include "System/System_Debug.h"
#include "System/System_Safety.h"
#include "System/System_StateMachine.h"
#include "System/System_Telemetry.h"

static System_State_t s_state;
static uint32_t s_last_tick_10ms = 0U;
static uint32_t s_last_tick_100ms = 0U;

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
    System_Manager_UpdateStateSnapshot(s_last_tick_10ms);
}

void System_Manager_Tick10ms(void)
{
    System_SM_Input_t in = {0};
    const RC_Signal_t *rc = RC_GetSignal();
    uint32_t now_ms = HAL_GetTick();

    System_Safety_Update(now_ms);

    /* TODO: 用真实自检流程填充。当前默认自检通过。 */
    in.self_check_passed = true;
    in.self_check_failed = false;

    if (rc != 0) {
        /* 占位映射（后续按你的遥控器通道定义替换）：
         * - CH17=1 且油门最低：请求进入 READY（满足解锁条件）
         * - CH18=1：切换到 RC；CH18=0：切换到 RUN
         * - CH17=0：请求回 STANDBY
         */
        in.arm_condition_ok = (rc->ch17 != 0U) && (rc->ch[2] <= 50);
        in.cmd_to_ready = (rc->ch17 != 0U);
        in.cmd_to_standby = (rc->ch17 == 0U);
        in.cmd_to_rc = (rc->ch18 != 0U);
        in.cmd_to_run = (rc->ch18 == 0U) && (rc->ch17 != 0U);
    }

    /* 保护条件占位：先用 Safety 汇总结果兜底。 */
    in.estop = false;
    in.telemetry_abnormal = false;
    in.hardware_abnormal = System_Safety_IsTriggered();
    in.cli_reboot_request = false;

    System_SM_Update(&in);
    System_Manager_UpdateStateSnapshot(now_ms);
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

