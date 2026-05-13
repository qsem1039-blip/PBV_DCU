#ifndef PBV_DCU_CAN_INTERFACE_LOGIC_H_
#define PBV_DCU_CAN_INTERFACE_LOGIC_H_

#include "pbv_dcu_ros2/dcu_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PBV_CAN_ID_DRIVE_CMD 0x200u
#define PBV_CAN_ID_STEER_CMD 0x201u
#define PBV_CAN_ID_SPEED_FB  0x100u
#define PBV_CAN_ID_PID       0x101u
#define PBV_CAN_ID_DUTY      0x102u
#define PBV_CAN_ID_ENC       0x103u
#define PBV_CAN_FLAG_ENABLE  0x01u
#define PBV_CAN_FLAG_ESTOP   0x02u

void can_interface_pack_drive_cmd(const DcuControlCommand *cmd, float max_speed_mps, uint8_t seq, uint8_t data[8]);
void can_interface_pack_steer_cmd(const DcuControlCommand *cmd, uint8_t seq, uint8_t data[8]);
int16_t can_interface_get_s16_le(const uint8_t data[8], uint8_t index);
uint16_t can_interface_get_u16_le(const uint8_t data[8], uint8_t index);

#ifdef __cplusplus
}
#endif

#endif
