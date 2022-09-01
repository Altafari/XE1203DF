#include <assert.h>
#include "stm32f446xx.h"
#include "arm_math.h"
#include "downsampling.h"
#include "fourier_analysis.h"
#include "stm32f4xx_hal.h"

static arm_fir_decimate_instance_q15 fir_stage1_i;
static arm_fir_decimate_instance_q15 fir_stage1_q;
static arm_fir_decimate_instance_q15 fir_stage2_i;
static arm_fir_decimate_instance_q15 fir_stage2_q;

static q15_t fir_stage1_coeff[FIR_STAGE1_N_TAPS] = {-9,-8,27,70,19,-163,-266,15,581,704,-264,-1712,-1675,
                                                    1432,6726,10901,10901,6726,1432,-1675,-1712,-264,704,
                                                    581,15,-266,-163,19,70,27,-8,-9};

static q15_t fir_stage2_coeff[FIR_STAGE2_N_TAPS] = {1,2,1,1,-1,-3,-4,-4,-2,2,7,11,10,4,-5,-16,-23,-21,-9,
                                                    10,30,42,38,16,-19,-54,-74,-66,-28,31,89,121,107,45,
                                                    -50,-141,-191,-168,-70,77,219,295,260,108,-118,-337,
                                                    -455,-402,-168,185,533,727,653,278,-314,-932,-1324,
                                                    -1251,-570,704,2386,4150,5614,6444,6444,5614,4150,
                                                    2386,704,-570,-1251,-1324,-932,-314,278,653,727,533,
                                                    185,-168,-402,-455,-337,-118,108,260,295,219,77,-70,
                                                    -168,-191,-141,-50,45,107,121,89,31,-28,-66,-74,-54,
                                                    -19,16,38,42,30,10,-9,-21,-23,-16,-5,4,10,11,7,2,
                                                    -2,-4,-4,-3,-1,1,1,2,1};

static q15_t fir_stage1_input_i[FIR_STAGE1_BLOCK_SIZE];
static q15_t fir_stage1_input_q[FIR_STAGE1_BLOCK_SIZE];
static q15_t fir_stage1_state_i[FIR_STAGE1_BLOCK_SIZE + FIR_STAGE1_N_TAPS - 1];
static q15_t fir_stage1_state_q[FIR_STAGE1_BLOCK_SIZE + FIR_STAGE1_N_TAPS - 1];

static q15_t fir_stage2_input_i[FIR_STAGE2_BLOCK_SIZE];
static q15_t fir_stage2_input_q[FIR_STAGE2_BLOCK_SIZE];
static q15_t fir_stage2_state_i[FIR_STAGE2_BLOCK_SIZE + FIR_STAGE2_N_TAPS - 1];
static q15_t fir_stage2_state_q[FIR_STAGE2_BLOCK_SIZE + FIR_STAGE2_N_TAPS - 1];

static q15_t output_buffer_i[FIR_OUTPUT_BLOCK_SIZE];
static q15_t output_buffer_q[FIR_OUTPUT_BLOCK_SIZE];

static q15_t filtered_iq_buffer[FIR_OUTPUT_BLOCK_SIZE * 2];
static q15_t magnitude_buffer[FIR_OUTPUT_BLOCK_SIZE];

static void multiply_f0_convert_to_q15(uint16_t* pIn, q15_t* pOutI, q15_t* pOutQ, uint16_t nSamples);
static void compose_iq_interlaced(q15_t* pOut, q15_t* pInI, q15_t* pInQ, uint16_t nSamples);

void DSP_FIR_init() {
  assert(FIR_OUTPUT_BLOCK_SIZE > 8);
  assert(FIR_OUTPUT_BLOCK_SIZE % 8 == 0);
    arm_fir_decimate_init_q15(
        &fir_stage1_i,
        FIR_STAGE1_N_TAPS,
        FIR_STAGE1_M,
        fir_stage1_coeff,
        fir_stage1_state_i,
        FIR_STAGE1_BLOCK_SIZE);

    arm_fir_decimate_init_q15(
        &fir_stage1_q,
        FIR_STAGE1_N_TAPS,
        FIR_STAGE1_M,
        fir_stage1_coeff,
        fir_stage1_state_q,
        FIR_STAGE1_BLOCK_SIZE);

    arm_fir_decimate_init_q15(
        &fir_stage2_i,
        FIR_STAGE2_N_TAPS,
        FIR_STAGE2_M,
        fir_stage2_coeff,
        fir_stage2_state_i,
        FIR_STAGE2_BLOCK_SIZE);

    arm_fir_decimate_init_q15(
        &fir_stage2_q,
        FIR_STAGE2_N_TAPS,
        FIR_STAGE2_M,
        fir_stage2_coeff,
        fir_stage2_state_q,
        FIR_STAGE2_BLOCK_SIZE);
}

void DSP_FIR_processBuffer(uint16_t* pBuff) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);

    multiply_f0_convert_to_q15(
        pBuff,
        fir_stage1_input_i,
        fir_stage1_input_q,
        FIR_STAGE1_BLOCK_SIZE);

    arm_fir_decimate_fast_q15(
        &fir_stage1_i,
        fir_stage1_input_i,
        fir_stage2_input_i,
        FIR_STAGE1_BLOCK_SIZE);

    arm_fir_decimate_fast_q15(
        &fir_stage2_i,
        fir_stage2_input_i,
        output_buffer_i,
        FIR_STAGE2_BLOCK_SIZE);

    arm_fir_decimate_fast_q15(
        &fir_stage1_q,
        fir_stage1_input_q,
        fir_stage2_input_q,
        FIR_STAGE1_BLOCK_SIZE);

    arm_fir_decimate_fast_q15(
        &fir_stage2_q,
        fir_stage2_input_q,
        output_buffer_q,
        FIR_STAGE2_BLOCK_SIZE);

    compose_iq_interlaced(
        filtered_iq_buffer,
        output_buffer_i,
        output_buffer_q,
        FIR_OUTPUT_BLOCK_SIZE * 2);

    arm_cmplx_mag_q15(
        filtered_iq_buffer,
        magnitude_buffer,
        FIR_OUTPUT_BLOCK_SIZE * 2);
    //DSP_FFT_receiveData(output_buffer_i, output_buffer_q);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
}

static void multiply_f0_convert_to_q15(uint16_t* pIn, q15_t* pOutI, q15_t* pOutQ, uint16_t nSamples) {
    for (uint16_t i = 0; i < nSamples; i += 8) {
        *pOutI++ = ((int16_t)(*pIn++) - DC_LEVEL) << 3;
        *pOutQ++ = ((int16_t)(*pIn++) - DC_LEVEL) << 3;
        *pOutI++ = (DC_LEVEL - (int16_t)(*pIn++)) << 3;
        *pOutQ++ = (DC_LEVEL - (int16_t)(*pIn++)) << 3;
        *pOutI++ = ((int16_t)(*pIn++) - DC_LEVEL) << 3;
        *pOutQ++ = ((int16_t)(*pIn++) - DC_LEVEL) << 3;
        *pOutI++ = (DC_LEVEL - (int16_t)(*pIn++)) << 3;
        *pOutQ++ = (DC_LEVEL - (int16_t)(*pIn++)) << 3;
    }
}

static void compose_iq_interlaced(q15_t* pOut, q15_t* pInI, q15_t* pInQ, uint16_t nSamples) {
    for (uint16_t i = 0; i < nSamples; i += 8) {
        *pOut++ = *pInI++;
        *pOut++ = *pInQ++;
        *pOut++ = *pInI++;
        *pOut++ = *pInQ++;
        *pOut++ = *pInI++;
        *pOut++ = *pInQ++;
        *pOut++ = *pInI++;
        *pOut++ = *pInQ++;
    }
}
