#ifndef PBV_DCU_PERCEPTION_LOGIC_H_
#define PBV_DCU_PERCEPTION_LOGIC_H_

#include "pbv_dcu_ros2/dcu_types.h"

#ifdef __cplusplus
extern "C" {
#endif

void perception_process(const DcuPerceptionConfig *cfg, const DcuOakdFeatures *input, DcuPerceptionResult *out);

#ifdef __cplusplus
}
#endif

#endif
