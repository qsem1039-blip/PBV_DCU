#include "decision_control_logic.h"

void decision_control_process(const DcuDecisionConfig *cfg,
                              const DcuPerceptionResult *perception,
                              const DcuFaultStatus *fault,
                              const DcuStateMode *state,
                              const DcuSafetyConstraint *safety,
                              DcuControlCommand *out)
{
    float base_speed = 0.8f;
    float rpm_per_mps = 220.0f;
    float offset_gain = 0.75f;
    float heading_gain = 0.45f;
    float max_speed = 0.0f;
    float max_steer = 0.0f;
    float steer = 0.0f;

    if (out == 0) return;
    if (cfg != 0)
    {
        base_speed = cfg->base_speed_mps;
        rpm_per_mps = cfg->rpm_per_mps;
        offset_gain = cfg->lane_offset_gain;
        heading_gain = cfg->lane_heading_gain;
    }

    max_speed = (safety != 0) ? safety->max_speed_mps : base_speed;
    max_steer = (safety != 0) ? safety->max_steer_norm : 1.0f;

    out->drive_state = (state != 0) ? state->mode : DCU_STATE_INIT;
    out->fault_summary = (fault != 0) ? fault->fault_bits : 0u;
    out->enable = 1u;
    out->estop = 0u;

    if (((fault != 0) && (fault->fail_safe != 0u)) || ((safety != 0) && (safety->failsafe != 0u)) ||
        ((state != 0) && (state->mode == DCU_STATE_FAILSAFE)))
    {
        out->enable = 0u;
        out->estop = 1u;
        out->target_speed_mps = 0.0f;
        out->target_rpm = 0.0f;
        out->steering_cmd = 0.0f;
        out->speed_level = 0u;
        return;
    }

    out->target_speed_mps = dcu_clampf(base_speed, 0.0f, max_speed);
    if ((state != 0) && ((state->mode == DCU_STATE_STOPPED) || (state->mode == DCU_STATE_APPROACH_STOP)))
    {
        out->target_speed_mps = dcu_clampf(out->target_speed_mps, 0.0f, max_speed);
    }

    if ((perception != 0) && (perception->lane_valid != 0u))
    {
        steer = -((perception->lane_offset_m * offset_gain) + (perception->lane_heading_rad * heading_gain));
    }
    else
    {
        steer = 0.0f;
    }

    out->steering_cmd = dcu_clampf(steer, -max_steer, max_steer);
    out->target_rpm = out->target_speed_mps * rpm_per_mps;
    out->speed_level = dcu_speed_level_from_speed(out->target_speed_mps, base_speed);
}
