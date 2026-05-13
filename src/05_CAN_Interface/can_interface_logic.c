#include "can_interface_logic.h"

static void put_s16_le(uint8_t *dst, int16_t value)
{
    uint16_t raw = (uint16_t)value;
    dst[0] = (uint8_t)(raw & 0xFFu);
    dst[1] = (uint8_t)((raw >> 8) & 0xFFu);
}

void can_interface_pack_drive_cmd(const DcuControlCommand *cmd, float max_speed_mps, uint8_t seq, uint8_t data[8])
{
    float speed_norm;
    int16_t vx;
    int16_t wz;
    uint8_t flags = 0u;

    if (data == 0) return;
    data[0] = seq;
    data[1] = 0u;
    data[2] = 0u;
    data[3] = 0u;
    data[4] = 0u;
    data[5] = 0u;
    data[6] = 0u;
    data[7] = 0u;

    if (cmd == 0) return;
    if (cmd->enable != 0u) flags |= PBV_CAN_FLAG_ENABLE;
    if (cmd->estop != 0u) flags |= PBV_CAN_FLAG_ESTOP;

    speed_norm = (max_speed_mps > 0.01f) ? (cmd->target_speed_mps / max_speed_mps) : 0.0f;
    speed_norm = dcu_clampf(speed_norm, -1.0f, 1.0f);
    vx = (int16_t)(speed_norm * 10000.0f);
    wz = (int16_t)(dcu_clampf(cmd->steering_cmd, -1.0f, 1.0f) * 10000.0f);

    data[0] = seq;
    data[1] = flags;
    data[2] = cmd->speed_level;
    data[3] = cmd->drive_state;
    put_s16_le(&data[4], vx);
    put_s16_le(&data[6], wz);
}


void can_interface_pack_steer_cmd(const DcuControlCommand *cmd, uint8_t seq, uint8_t data[8])
{
    int16_t steer_norm;
    uint8_t flags = 0u;

    if (data == 0) return;
    data[0] = seq;
    data[1] = 0u;
    data[2] = 0u;
    data[3] = 0u;
    data[4] = 0u;
    data[5] = 0u;
    data[6] = 0u;
    data[7] = 0u;

    if (cmd == 0) return;
    if (cmd->enable != 0u) flags |= PBV_CAN_FLAG_ENABLE;
    if (cmd->estop != 0u) flags |= PBV_CAN_FLAG_ESTOP;

    steer_norm = (int16_t)(dcu_clampf(cmd->steering_cmd, -1.0f, 1.0f) * 10000.0f);

    data[0] = seq;
    data[1] = ((cmd->enable != 0u) && (cmd->estop == 0u)) ? 1u : 0u;
    put_s16_le(&data[2], steer_norm);
    data[4] = cmd->speed_level;
    data[5] = flags;
    data[6] = 0u;
    data[7] = 0u;
}

int16_t can_interface_get_s16_le(const uint8_t data[8], uint8_t index)
{
    uint16_t raw = (uint16_t)((uint16_t)data[index] | ((uint16_t)data[index + 1u] << 8));
    return (int16_t)raw;
}

uint16_t can_interface_get_u16_le(const uint8_t data[8], uint8_t index)
{
    return (uint16_t)((uint16_t)data[index] | ((uint16_t)data[index + 1u] << 8));
}
