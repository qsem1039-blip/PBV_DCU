#include "pbv_dcu_ros2/dcu_types.h"

float dcu_clampf(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }
    return value;
}

uint8_t dcu_speed_level_from_speed(float speed_mps, float nominal_max_speed_mps)
{
    float ratio;

    if ((speed_mps <= 0.01f) || (nominal_max_speed_mps <= 0.01f))
    {
        return 0u;
    }

    ratio = speed_mps / nominal_max_speed_mps;
    if (ratio < 0.20f) return 1u;
    if (ratio < 0.40f) return 2u;
    if (ratio < 0.65f) return 3u;
    if (ratio < 0.85f) return 4u;
    return 5u;
}
