#include "System/System_Safety.h"

#include "Driver/ESC/ESC.h"
#include "Driver/RC/RC.h"
#include "System/System_Def.h"

#define SYS_RC_TIMEOUT_MS         200U
#define SYS_ESC_ERR_THRESHOLD     10U

static uint32_t s_fault_flags = SYS_FAULT_NONE;
static bool s_triggered = false;

void System_Safety_Init(void)
{
    s_fault_flags = SYS_FAULT_NONE;
    s_triggered = false;
}

void System_Safety_Update(uint32_t now_ms)
{
    const RC_Signal_t *rc = RC_GetSignal();
    ESC_ServiceState_t esc1;
    ESC_ServiceState_t esc2;
    bool esc1_ok = ESC_Service_ReadState(ESC_CH1, &esc1);
    bool esc2_ok = ESC_Service_ReadState(ESC_CH2, &esc2);

    s_fault_flags = SYS_FAULT_NONE;

    if (rc == 0) {
        s_fault_flags |= SYS_FAULT_INTERNAL;
    } else {
        if ((!rc->is_online) || ((now_ms - rc->last_update_ms) > SYS_RC_TIMEOUT_MS)) {
            s_fault_flags |= SYS_FAULT_RC_LOST;
        }
    }

    if (esc1_ok && (esc1.update_err_count >= SYS_ESC_ERR_THRESHOLD)) {
        s_fault_flags |= SYS_FAULT_ESC_ERROR;
    }
    if (esc2_ok && (esc2.update_err_count >= SYS_ESC_ERR_THRESHOLD)) {
        s_fault_flags |= SYS_FAULT_ESC_ERROR;
    }

    s_triggered = (s_fault_flags != SYS_FAULT_NONE);
}

uint32_t System_Safety_GetFaultFlags(void)
{
    return s_fault_flags;
}

bool System_Safety_IsTriggered(void)
{
    return s_triggered;
}

