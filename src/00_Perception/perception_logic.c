#include "perception_logic.h"

void perception_process(const DcuPerceptionConfig *cfg, const DcuOakdFeatures *input, DcuPerceptionResult *out)
{
    float lane_threshold = 0.55f;
    float target_threshold = 0.50f;

    if ((input == 0) || (out == 0))
    {
        return;
    }
    if (cfg != 0)
    {
        lane_threshold = cfg->lane_valid_threshold;
        target_threshold = cfg->target_valid_threshold;
    }

    out->timestamp_ms = input->timestamp_ms;
    out->camera_ok = input->camera_ok;
    out->lane_confidence = input->lane_confidence;
    out->lane_offset_m = input->lane_offset_m;
    out->lane_heading_rad = input->lane_heading_rad;
    out->traffic_light_state = input->traffic_light_state;
    out->traffic_light_confidence = input->traffic_light_confidence;
    out->traffic_light_distance_m = input->traffic_light_distance_m;
    out->lane_valid = ((input->camera_ok != 0u) && (input->lane_confidence >= lane_threshold)) ? 1u : 0u;
    out->target_valid = ((input->camera_ok != 0u) && (input->traffic_light_confidence >= target_threshold)) ? 1u : 0u;
}
