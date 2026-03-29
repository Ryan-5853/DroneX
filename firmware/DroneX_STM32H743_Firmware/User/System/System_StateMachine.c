#include "System/System_StateMachine.h"

static System_SM_Output_t s_out;

void System_SM_Init(void)
{
    s_out.mode = SYS_MODE_STANDBY;
    s_out.last_reason = SYS_TRANS_REASON_NONE;
    s_out.power_unit_activated = 0U;
    s_out.power_output_enabled = 0U;
    s_out.reboot_required = 0U;
}

static void System_SM_Enter(System_Mode_t mode, System_TransitionReason_t reason)
{
    s_out.mode = mode;
    s_out.last_reason = reason;

    switch (mode) {
        case SYS_MODE_READY:
            s_out.power_unit_activated = 1U;
            s_out.power_output_enabled = 0U;
            s_out.reboot_required = 0U;
            break;
        case SYS_MODE_RC:
        case SYS_MODE_RUN:
            s_out.power_unit_activated = 1U;
            s_out.power_output_enabled = 1U;
            s_out.reboot_required = 0U;
            break;
        case SYS_MODE_STANDBY:
            s_out.power_unit_activated = 0U;
            s_out.power_output_enabled = 0U;
            s_out.reboot_required = 0U;
            break;
        case SYS_MODE_ERROR:
        case SYS_MODE_PROTECT:
            s_out.power_unit_activated = 0U;
            s_out.power_output_enabled = 0U;
            s_out.reboot_required = 1U;
            break;
        default:
            break;
    }
}

void System_SM_Update(const System_SM_Input_t *in)
{
    if (in == 0) {
        return;
    }

    if (in->cli_reboot_request) {
        System_SM_Init();
        s_out.last_reason = SYS_TRANS_REASON_CLI_REBOOT;
        return;
    }

    /* 错误态和保护态为锁定态，必须重启恢复。 */
    if (s_out.mode == SYS_MODE_ERROR || s_out.mode == SYS_MODE_PROTECT) {
        return;
    }

    if (in->estop) {
        System_SM_Enter(SYS_MODE_PROTECT, SYS_TRANS_REASON_ESTOP);
        return;
    }
    if (in->telemetry_abnormal) {
        System_SM_Enter(SYS_MODE_PROTECT, SYS_TRANS_REASON_TELEMETRY_ABN);
        return;
    }
    if (in->hardware_abnormal) {
        System_SM_Enter(SYS_MODE_PROTECT, SYS_TRANS_REASON_HW_ABN);
        return;
    }
    if (in->self_check_failed) {
        System_SM_Enter(SYS_MODE_ERROR, SYS_TRANS_REASON_BOOT_SELF_CHECK_FAIL);
        return;
    }
    if (in->self_check_passed && s_out.mode == SYS_MODE_STANDBY) {
        s_out.last_reason = SYS_TRANS_REASON_BOOT_SELF_CHECK_PASS;
    }

    switch (s_out.mode) {
        case SYS_MODE_STANDBY:
            if (in->arm_condition_ok && in->cmd_to_ready) {
                System_SM_Enter(SYS_MODE_READY, SYS_TRANS_REASON_ARM_CONDITION_OK);
            }
            break;

        case SYS_MODE_READY:
            if (in->cmd_to_standby) {
                System_SM_Enter(SYS_MODE_STANDBY, SYS_TRANS_REASON_CMD_TO_STANDBY);
            } else if (in->cmd_to_rc) {
                System_SM_Enter(SYS_MODE_RC, SYS_TRANS_REASON_CMD_TO_RC);
            } else if (in->cmd_to_run) {
                System_SM_Enter(SYS_MODE_RUN, SYS_TRANS_REASON_CMD_TO_RUN);
            }
            break;

        case SYS_MODE_RC:
            if (in->cmd_to_standby) {
                System_SM_Enter(SYS_MODE_STANDBY, SYS_TRANS_REASON_CMD_TO_STANDBY);
            } else if (in->cmd_to_ready) {
                System_SM_Enter(SYS_MODE_READY, SYS_TRANS_REASON_CMD_TO_READY);
            } else if (in->cmd_to_run) {
                System_SM_Enter(SYS_MODE_RUN, SYS_TRANS_REASON_CMD_TO_RUN);
            }
            break;

        case SYS_MODE_RUN:
            if (in->cmd_to_standby) {
                System_SM_Enter(SYS_MODE_STANDBY, SYS_TRANS_REASON_CMD_TO_STANDBY);
            } else if (in->cmd_to_ready) {
                System_SM_Enter(SYS_MODE_READY, SYS_TRANS_REASON_CMD_TO_READY);
            } else if (in->cmd_to_rc) {
                System_SM_Enter(SYS_MODE_RC, SYS_TRANS_REASON_CMD_TO_RC);
            }
            break;

        case SYS_MODE_ERROR:
        case SYS_MODE_PROTECT:
        default:
            break;
    }
}

void System_SM_ForceProtect(System_TransitionReason_t reason)
{
    if (reason == SYS_TRANS_REASON_NONE) {
        reason = SYS_TRANS_REASON_HW_ABN;
    }
    System_SM_Enter(SYS_MODE_PROTECT, reason);
}

void System_SM_GetOutput(System_SM_Output_t *out)
{
    if (out == 0) {
        return;
    }
    *out = s_out;
}

System_Mode_t System_SM_GetMode(void)
{
    return s_out.mode;
}

bool System_SM_IsArmed(void)
{
    return (s_out.power_unit_activated != 0U);
}

