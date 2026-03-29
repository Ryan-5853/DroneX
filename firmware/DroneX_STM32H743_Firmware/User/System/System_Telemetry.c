#include "System/System_Telemetry.h"

#include <string.h>

#include "Driver/ESC/ESC.h"
#include "Driver/RC/RC.h"

static System_TelemetryPacket_t s_last_packet;
static bool s_has_packet = false;

void System_Telemetry_Init(void)
{
    memset(&s_last_packet, 0, sizeof(s_last_packet));
    s_has_packet = false;
}

void System_Telemetry_Update(const System_State_t *state, uint32_t now_ms)
{
    const RC_Signal_t *rc = RC_GetSignal();
    ESC_ServiceState_t esc1;
    ESC_ServiceState_t esc2;
    uint32_t esc_err = 0U;

    if (state == 0) {
        return;
    }

    if (ESC_Service_ReadState(ESC_CH1, &esc1)) {
        esc_err += esc1.update_err_count;
    }
    if (ESC_Service_ReadState(ESC_CH2, &esc2)) {
        esc_err += esc2.update_err_count;
    }

    s_last_packet.timestamp_ms = now_ms;
    s_last_packet.mode = state->mode;
    s_last_packet.fault_flags = state->fault_flags;
    s_last_packet.armed = state->armed;
    s_last_packet.safety_triggered = state->safety_triggered;
    s_last_packet.power_unit_activated = state->power_unit_activated;
    s_last_packet.power_output_enabled = state->power_output_enabled;
    s_last_packet.reboot_required = state->reboot_required;
    s_last_packet.rc_online = (rc != 0) ? rc->is_online : 0U;
    s_last_packet.throttle_cmd = (rc != 0) ? rc->ch[2] : 0;
    s_last_packet.esc_update_err_count = esc_err;
    s_has_packet = true;
}

bool System_Telemetry_GetLastPacket(System_TelemetryPacket_t *out)
{
    if (!s_has_packet || out == 0) {
        return false;
    }
    *out = s_last_packet;
    return true;
}

