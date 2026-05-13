#ifndef PBV_DCU_STATE_MANAGER_LOGIC_H_
#define PBV_DCU_STATE_MANAGER_LOGIC_H_

#include "pbv_dcu_ros2/dcu_types.h"

#ifdef __cplusplus
extern "C" {
#endif

void state_manager_process(const DcuStateConfig *cfg, const DcuFaultStatus *fault, const DcuPerceptionResult *perception, DcuStateMode *out);

#ifdef __cplusplus
}
#endif

#endif
