#include "state_manager_logic.h"

static void state_manager_set_mode(uint8_t mode, DcuStateMode *out)
{
    if (out == 0) return;
    out->mode = mode;
    out->init = (mode == DCU_STATE_INIT) ? 1u : 0u;
    out->ready = (mode == DCU_STATE_READY) ? 1u : 0u;
    out->cruise = (mode == DCU_STATE_CRUISE) ? 1u : 0u;
    out->approach_stop = (mode == DCU_STATE_APPROACH_STOP) ? 1u : 0u;
    out->stopped = (mode == DCU_STATE_STOPPED) ? 1u : 0u;
    out->degraded = (mode == DCU_STATE_DEGRADED) ? 1u : 0u;
    out->failsafe = (mode == DCU_STATE_FAILSAFE) ? 1u : 0u;
}

void state_manager_process(const DcuStateConfig *cfg, const DcuFaultStatus *fault, const DcuPerceptionResult *perception, DcuStateMode *out)
{
    float stop_distance = 1.0f;
    float approach_distance = 5.0f;
    uint8_t red_light_near = 0u;

    if (out == 0) return;
    if ((fault == 0) || (perception == 0))
    {
        state_manager_set_mode(DCU_STATE_INIT, out);
        return;
    }
    if (cfg != 0)
    {
        stop_distance = cfg->stop_distance_m;
        approach_distance = cfg->approach_distance_m;
    }

    if ((perception->target_valid != 0u) && (perception->traffic_light_state == DCU_TL_RED))
    {
        red_light_near = (perception->traffic_light_distance_m <= approach_distance) ? 1u : 0u;
    }

    if (fault->fail_safe != 0u)
    {
        state_manager_set_mode(DCU_STATE_FAILSAFE, out);
    }
    else if ((red_light_near != 0u) && (perception->traffic_light_distance_m <= stop_distance))
    {
        state_manager_set_mode(DCU_STATE_STOPPED, out);
    }
    else if (red_light_near != 0u)
    {
        state_manager_set_mode(DCU_STATE_APPROACH_STOP, out);
    }
    else if (fault->lane_fault != 0u)
    {
        state_manager_set_mode(DCU_STATE_DEGRADED, out);
    }
    else
    {
        state_manager_set_mode(DCU_STATE_CRUISE, out);
    }
}
