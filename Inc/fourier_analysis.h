#ifndef __FOURIER_ANALYSIS_H
#define __FOURIER_ANALYSIS_H

#include<inttypes.h>
#include "arm_math.h"

#ifdef __cplusplus
 extern "C" {
#endif

void DSP_FFT_init();
void DSP_FFT_receiveData(q15_t* pDataRe, q15_t* pDataIm);
void DSP_FFT_processDataFromLoop();

#ifdef __cplusplus
}
#endif

#endif /*__FOURIER_ANALYSIS_H */
