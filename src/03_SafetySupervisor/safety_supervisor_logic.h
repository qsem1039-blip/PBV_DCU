#ifndef PBV_DCU_SAFETY_SUPERVISOR_LOGIC_H_
#define PBV_DCU_SAFETY_SUPERVISOR_LOGIC_H_

#include "pbv_dcu_ros2/dcu_types.h"

#ifdef __cplusplus
extern "C" {
#endif

void safety_supervisor_process(const DcuSafetyConfig *cfg, const DcuFaultStatus *fault, const DcuPerceptionResult *perception, DcuSafetyConstraint *out);

#ifdef __cplusplus
}
#endif

#endif
