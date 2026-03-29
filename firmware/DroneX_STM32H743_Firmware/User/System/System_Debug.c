#include "System/System_Debug.h"

#include "Driver/Debug/Debug.h"

static System_Mode_t s_last_mode = SYS_MODE_INIT;
static uint32_t s_last_fault = SYS_FAULT_NONE;
static uint8_t s_initialized = 0U;

void System_Debug_Init(void)
{
    s_last_mode = SYS_MODE_INIT;
    s_last_fault = SYS_FAULT_NONE;
    s_initialized = 1U;
}

void System_Debug_Update(const System_State_t *state)
{
    if ((state == 0) || (s_initialized == 0U)) {
        return;
    }

    if ((state->mode != s_last_mode) || (state->fault_flags != s_last_fault)) {
        Debug_Printf("[SYS] mode=%d fault=0x%08lx pwr_act=%u pwr_out=%u reboot=%u safety=%u\r\n",
                     (int)state->mode,
                     (unsigned long)state->fault_flags,
                     (unsigned)state->power_unit_activated,
                     (unsigned)state->power_output_enabled,
                     (unsigned)state->reboot_required,
                     (unsigned)state->safety_triggered);
        s_last_mode = state->mode;
        s_last_fault = state->fault_flags;
    }
}

