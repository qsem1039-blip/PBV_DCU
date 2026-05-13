#include "fault_management_logic.h"

void fault_management_process(const DcuFaultConfig *cfg, DcuFaultMemory *mem, const DcuPerceptionResult *perception, DcuFaultStatus *out)
{
    uint32_t age_ms = 0u;
    uint32_t timeout_ms = 200u;
    float lane_min_confidence = 0.45f;
    uint16_t faults = 0u;

    if ((perception == 0) || (out == 0))
    {
        return;
    }
    if (cfg != 0)
    {
        timeout_ms = cfg->timeout_ms;
        lane_min_confidence = cfg->lane_min_confidence;
        age_ms = (cfg->now_ms >= perception->timestamp_ms) ? (cfg->now_ms - perception->timestamp_ms) : 0u;
    }

    if (perception->camera_ok == 0u)
    {
        faults |= DCU_FAULT_CAMERA;
    }
    if ((perception->lane_valid == 0u) || (perception->lane_confidence < lane_min_confidence))
    {
        faults |= DCU_FAULT_LANE;
    }
    if (age_ms > timeout_ms)
    {
        faults |= DCU_FAULT_TIMEOUT;
    }

    if (mem != 0)
    {
        if ((faults & DCU_FAULT_TIMEOUT) != 0u) mem->stale_counter++;
        if ((faults & DCU_FAULT_CAMERA) != 0u) mem->lost_counter++;
        out->stale_counter = mem->stale_counter;
        out->lost_counter = mem->lost_counter;
    }
    else
    {
        out->stale_counter = 0u;
        out->lost_counter = 0u;
    }

    out->fault_bits = faults;
    out->timeout_fault = ((faults & DCU_FAULT_TIMEOUT) != 0u) ? 1u : 0u;
    out->lane_fault = ((faults & DCU_FAULT_LANE) != 0u) ? 1u : 0u;
    out->tps_fault = 0u;
    out->fail_safe = ((faults & (DCU_FAULT_TIMEOUT | DCU_FAULT_CAMERA)) != 0u) ? 1u : 0u;
    out->requested_gate = (out->fail_safe == 0u) ? 1u : 0u;
    out->sensor_health = (uint8_t)((faults == 0u) ? 100u : ((out->fail_safe != 0u) ? 0u : 50u));
}
