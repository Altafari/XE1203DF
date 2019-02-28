#ifndef POST_PROCESSING_H_
#define POST_PROCESSING_H_

#ifdef __cplusplus
 extern "C" {
#endif

void DSP_PP_init();
void DSP_PP_updateFilterState(float deltaPhi, float sigMagnitude, float deltaPeak);

#ifdef __cplusplus
}
#endif

#endif /* POST_PROCESSING_H_ */
