#ifndef PBV_DCU_FAULT_MANAGEMENT_LOGIC_H_
#define PBV_DCU_FAULT_MANAGEMENT_LOGIC_H_

#include "pbv_dcu_ros2/dcu_types.h"

#ifdef __cplusplus
extern "C" {
#endif

void fault_management_process(const DcuFaultConfig *cfg, DcuFaultMemory *mem, const DcuPerceptionResult *perception, DcuFaultStatus *out);

#ifdef __cplusplus
}
#endif

#endif
