#include <stm32f446xx.h>
#include <arm_math.h>
#include "downsampling.h"
#include "fourier_analysis.h"
#include "stm32f4xx_hal.h"

#define FIR_STAGE1_N_TAPS 32
#define FIR_STAGE2_N_TAPS 128
#define FIR_STAGE2_BLOCK_SIZE 64
#define FIR_STAGE1_M 4
#define FIR_STAGE2_M 8
#define DC_LEVEL 1670

static arm_fir_decimate_instance_q15 fir_stage1_i;
static arm_fir_decimate_instance_q15 fir_stage1_q;
static arm_fir_decimate_instance_q15 fir_stage2_i;
static arm_fir_decimate_instance_q15 fir_stage2_q;

static q15_t fir_stage1_coeff[FIR_STAGE1_N_TAPS] = { -87,-234,-480,-793,-1093,-1241,-1052,-334,1057,3166,5892,8982,12060,14689,16460,17086,16460,
                                                    14689,12060,8982,5892,3166,1057,-334,-1052,-1241,-1093,-793,-480,-234,-87,0 };

static q15_t fir_stage2_coeff[FIR_STAGE2_N_TAPS] = { -18,-126,-128,-193,-254,-318,-376,-422,-450,-452,-426,-368,-278,-162,-25,122,267,394,490,543,542,483,365,
                                                    196,-12,-240,-465,-664,-812,-886,-872,-761,-553,-263,89,471,844,1167,1398,1502,1453,1236,854,327,-307,-994,
                                                    -1668,-2254,-2677,-2866,-2760,-2318,-1520,-371,1094,2816,4711,6680,8610,10387,11901,13058,13783,14030,13783,
                                                    13058,11901,10387,8610,6680,4711,2816,1094,-371,-1520,-2318,-2760,-2866,-2677,-2254,-1668,-994,-307,327,854,
                                                    1236,1453,1502,1398,1167,844,471,89,-263,-553,-761,-872,-886,-812,-664,-465,-240,-12,196,365,483,542,543,490,
                                                    394,267,122,-25,-162,-278,-368,-426,-452,-450,-422,-376,-318,-254,-193,-128,-126,-18, 0 };

static q15_t fir_stage1_input[FIR_STAGE1_BLOCK_SIZE];
static q15_t fir_stage1_state_i[FIR_STAGE1_BLOCK_SIZE + FIR_STAGE1_N_TAPS - 1];
static q15_t fir_stage1_state_q[FIR_STAGE1_BLOCK_SIZE + FIR_STAGE1_N_TAPS - 1];

static q15_t fir_stage2_input[FIR_STAGE2_BLOCK_SIZE];
static q15_t fir_stage2_state_i[FIR_STAGE2_BLOCK_SIZE + FIR_STAGE2_N_TAPS - 1];
static q15_t fir_stage2_state_q[FIR_STAGE2_BLOCK_SIZE + FIR_STAGE2_N_TAPS - 1];

static q15_t output_buffer_i[FIR_OUTPUT_BLOCK_SIZE];
static q15_t output_buffer_q[FIR_OUTPUT_BLOCK_SIZE];

static void convert12bitU_to_q15(uint16_t* pIn, q15_t* pOut, uint16_t nSamples);

void DSP_FIR_init() {
    arm_fir_decimate_init_q15(&fir_stage1_i, FIR_STAGE1_N_TAPS, FIR_STAGE1_M, fir_stage1_coeff, fir_stage1_state_i, FIR_STAGE1_BLOCK_SIZE);
    arm_fir_decimate_init_q15(&fir_stage1_q, FIR_STAGE1_N_TAPS, FIR_STAGE1_M, fir_stage1_coeff, fir_stage1_state_q, FIR_STAGE1_BLOCK_SIZE);
    arm_fir_decimate_init_q15(&fir_stage2_i, FIR_STAGE2_N_TAPS, FIR_STAGE2_M, fir_stage2_coeff, fir_stage2_state_i, FIR_STAGE2_BLOCK_SIZE);
    arm_fir_decimate_init_q15(&fir_stage2_q, FIR_STAGE2_N_TAPS, FIR_STAGE2_M, fir_stage2_coeff, fir_stage2_state_q, FIR_STAGE2_BLOCK_SIZE);
}

void DSP_FIR_processBuffer(uint16_t* pBuff_i, uint16_t* pBuff_q) {
//    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
    convert12bitU_to_q15(pBuff_i, fir_stage1_input, FIR_STAGE1_BLOCK_SIZE);
    arm_fir_decimate_fast_q15(&fir_stage1_i, fir_stage1_input, fir_stage2_input, FIR_STAGE1_BLOCK_SIZE);
    arm_fir_decimate_fast_q15(&fir_stage2_i, fir_stage2_input, output_buffer_i, FIR_STAGE2_BLOCK_SIZE);
    convert12bitU_to_q15(pBuff_q, fir_stage1_input, FIR_STAGE1_BLOCK_SIZE);
    arm_fir_decimate_fast_q15(&fir_stage1_q, fir_stage1_input, fir_stage2_input, FIR_STAGE1_BLOCK_SIZE);
    arm_fir_decimate_fast_q15(&fir_stage2_q, fir_stage2_input, output_buffer_q, FIR_STAGE2_BLOCK_SIZE);
    DSP_FFT_receiveData(output_buffer_i, output_buffer_q);
//    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
}

static void convert12bitU_to_q15(uint16_t* pIn, q15_t* pOut, uint16_t nSamples) {
    for (uint16_t i = 0; i < nSamples; i += 8) {
        *pOut++ = (*pIn++) - DC_LEVEL;
        *pOut++ = (*pIn++) - DC_LEVEL;
        *pOut++ = (*pIn++) - DC_LEVEL;
        *pOut++ = (*pIn++) - DC_LEVEL;
        *pOut++ = (*pIn++) - DC_LEVEL;
        *pOut++ = (*pIn++) - DC_LEVEL;
        *pOut++ = (*pIn++) - DC_LEVEL;
        *pOut++ = (*pIn++) - DC_LEVEL;
    }
}
