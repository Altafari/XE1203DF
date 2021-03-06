#ifndef __DOWNSAMPLING_H
#define __DOWNSAMPLING_H

#include<inttypes.h>

#define FIR_STAGE1_BLOCK_SIZE 256
#define FIR_OUTPUT_BLOCK_SIZE 8

#ifdef __cplusplus
 extern "C" {
#endif

void DSP_FIR_init();
void DSP_FIR_processBuffer(uint16_t* pBuffI, uint16_t* pBuffQ);

#ifdef __cplusplus
}
#endif

#endif /*__DOWNSAMPLING_H */
