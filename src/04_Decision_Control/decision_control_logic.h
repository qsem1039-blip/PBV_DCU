#ifndef PBV_DCU_DECISION_CONTROL_LOGIC_H_
#define PBV_DCU_DECISION_CONTROL_LOGIC_H_

#include "pbv_dcu_ros2/dcu_types.h"

#ifdef __cplusplus
extern "C" {
#endif

void decision_control_process(const DcuDecisionConfig *cfg,
                              const DcuPerceptionResult *perception,
                              const DcuFaultStatus *fault,
                              const DcuStateMode *state,
                              const DcuSafetyConstraint *safety,
                              DcuControlCommand *out);

#ifdef __cplusplus
}
#endif

#endif
