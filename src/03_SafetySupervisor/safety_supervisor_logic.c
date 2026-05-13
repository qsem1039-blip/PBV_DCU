#include "safety_supervisor_logic.h"

void safety_supervisor_process(const DcuSafetyConfig *cfg, const DcuFaultStatus *fault, const DcuPerceptionResult *perception, DcuSafetyConstraint *out)
{
    float nominal = 1.0f;
    float degraded = 0.35f;
    float stop_distance = 1.0f;
    float target_near_distance = 5.0f;

    if (out == 0) return;
    if (cfg != 0)
    {
        nominal = cfg->nominal_max_speed_mps;
        degraded = cfg->degraded_max_speed_mps;
        stop_distance = cfg->stop_distance_m;
        target_near_distance = cfg->target_near_distance_m;
    }

    out->stale_data = ((fault != 0) && (fault->timeout_fault != 0u)) ? 1u : 0u;
    out->degraded_lane = ((fault != 0) && (fault->lane_fault != 0u)) ? 1u : 0u;
    out->failsafe = ((fault != 0) && (fault->fail_safe != 0u)) ? 1u : 0u;
    out->red_light = ((perception != 0) && (perception->target_valid != 0u) && (perception->traffic_light_state == DCU_TL_RED)) ? 1u : 0u;
    out->target_near = ((perception != 0) && (perception->target_valid != 0u) && (perception->traffic_light_distance_m <= target_near_distance)) ? 1u : 0u;
    out->stop_line = ((perception != 0) && (perception->target_valid != 0u) && (perception->traffic_light_distance_m <= stop_distance)) ? 1u : 0u;

    out->safety_bits = 0u;
    if (out->stale_data != 0u) out->safety_bits |= 0x0001u;
    if (out->red_light != 0u) out->safety_bits |= 0x0002u;
    if (out->target_near != 0u) out->safety_bits |= 0x0004u;
    if (out->degraded_lane != 0u) out->safety_bits |= 0x0008u;
    if (out->failsafe != 0u) out->safety_bits |= 0x0010u;
    if (out->stop_line != 0u) out->safety_bits |= 0x0020u;

    if ((out->failsafe != 0u) || ((out->red_light != 0u) && (out->stop_line != 0u)))
    {
        out->max_speed_mps = 0.0f;
        out->max_steer_norm = 0.0f;
    }
    else if (out->degraded_lane != 0u)
    {
        out->max_speed_mps = degraded;
        out->max_steer_norm = 0.35f;
    }
    else if ((out->red_light != 0u) && (out->target_near != 0u))
    {
        out->max_speed_mps = degraded;
        out->max_steer_norm = 0.70f;
    }
    else
    {
        out->max_speed_mps = nominal;
        out->max_steer_norm = 1.0f;
    }
}
