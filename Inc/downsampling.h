#ifndef __DOWNSAMPLING_H
#define __DOWNSAMPLING_H

#include<inttypes.h>

#define FIR_STAGE1_N_TAPS 32
#define FIR_STAGE2_N_TAPS 128
#define FIR_STAGE1_BLOCK_SIZE 256
#define FIR_STAGE1_M 2
#define FIR_STAGE2_BLOCK_SIZE (FIR_STAGE1_BLOCK_SIZE / FIR_STAGE1_M)
#define FIR_STAGE2_M 4
#define DC_LEVEL 2047
#define FIR_OUTPUT_BLOCK_SIZE (FIR_STAGE1_BLOCK_SIZE / (FIR_STAGE1_M * FIR_STAGE2_M))

#ifdef __cplusplus
 extern "C" {
#endif

void DSP_FIR_init();
void DSP_FIR_processBuffer(uint16_t* pBuff);

#ifdef __cplusplus
}
#endif

#endif /*__DOWNSAMPLING_H */
