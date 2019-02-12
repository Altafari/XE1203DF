#include <inttypes.h>
#define __FPU_PRESENT 1U
#include "arm_math.h"
#define FIR_STAGE1_N_TAPS 128
#define FIR_STAGE1_BLOCK_SIZE 256
#define FIR_STAGE1_M 4

arm_fir_decimate_instance_q15 fir_stage1_i;
arm_fir_decimate_instance_q15 fir_stage1_q;
arm_fir_decimate_instance_q15 i_channel_fir_stage2;
arm_fir_decimate_instance_q15 q_channel_fir_stage2;

q15_t fir_stage1_coeff[FIR_STAGE1_N_TAPS];
q15_t fir_stage1_state_i[FIR_STAGE1_BLOCK_SIZE / FIR_STAGE1_M + FIR_STAGE1_N_TAPS + 1];
q15_t fir_stage1_state_q[FIR_STAGE1_BLOCK_SIZE / FIR_STAGE1_M + FIR_STAGE1_N_TAPS + 1];

q15_t temp_buffer[FIR_STAGE1_BLOCK_SIZE];

void DSP_CORE_Init() {
    arm_status res = arm_fir_decimate_init_q15(&fir_stage1_i, FIR_STAGE1_M, FIR_STAGE1_N_TAPS, fir_stage1_coeff, fir_stage1_state_i, FIR_STAGE1_BLOCK_SIZE);
    if (res != 0) {
        arm_fir_decimate_init_q15(&fir_stage1_q, FIR_STAGE1_M, FIR_STAGE1_N_TAPS, fir_stage1_coeff, fir_stage1_state_q, FIR_STAGE1_BLOCK_SIZE);
    }
}

void DSP_CORE_ProcessBufferI(uint16_t* pBuff) {
    arm_fir_decimate_fast_q15(
      &fir_stage1_i,
      (q15_t*) pBuff,
      temp_buffer,
      FIR_STAGE1_BLOCK_SIZE);
}
