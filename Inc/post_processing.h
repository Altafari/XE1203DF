#ifndef POST_PROCESSING_H_
#define POST_PROCESSING_H_

#include <arm_math.h>
#include <inttypes.h>

#ifdef __cplusplus
 extern "C" {
#endif

void DSP_PP_init();
void DSP_PP_updateFilterState(float a, float b, float c, float peak);

#ifdef __cplusplus
}
#endif

#endif /* POST_PROCESSING_H_ */
