#include <stm32f446xx.h>
#include <math.h>
#include <arm_const_structs.h>
#include "downsampling.h"
#include "fourier_analysis.h"
#include "post_processing.h"
#include "stm32f4xx_hal.h"
#include "dac_scope.h"

#define FFT_SIZE 1024
#define NUM_FRAMES_TO_PROCESS 50
#define FFT_WINDOW_SIZE (FIR_OUTPUT_BLOCK_SIZE * NUM_FRAMES_TO_PROCESS)
#define DB_NORM 156.0f
#define TRIPLET_DELTA (FFT_SIZE / (FIR_OUTPUT_BLOCK_SIZE * 2))
#define SEARCH_RANGE (TRIPLET_DELTA / 2)
#define MIN_IDX 400
#define MAX_IDX 850
#define FFT_SCALING (-5)
#define PRODUCT_RSHIFT 8
#define PT_RATE 8

static q15_t switch_window[FIR_OUTPUT_BLOCK_SIZE];
static q15_t fft_window[FFT_WINDOW_SIZE * 2];

static volatile uint8_t bufferIdx;
static volatile uint8_t bufferReady;
static volatile uint16_t roundCtr;

static uint16_t frameCtr;
static q15_t* pBuffer;
static q15_t* pBufferOther;
static q15_t storageBufferA[2][FFT_WINDOW_SIZE * 2];
static q15_t storageBufferB[2][FFT_WINDOW_SIZE * 2];

static uint16_t refIdx;
static q15_t fftBuffer[FFT_SIZE * 2];
static q31_t fftBufferA[FFT_SIZE * 2];
static q31_t fftBufferB[FFT_SIZE * 2];

static float fftMagnitude[FFT_SIZE];
static q31_t fftDiffAngle[FFT_SIZE * 2];

static uint16_t scope_buffer[FFT_SIZE * 2];

static void DSP_FFT_fillBlackmanWindowQ15(q15_t* pBuffer, uint16_t size);
static void DSP_FFT_fillBlackmanWindowComplexQ15(q15_t* pBuffer, uint16_t size);
static q15_t DSP_FFT_computeBlackmanWindow(uint16_t n, uint16_t size);
static void DSP_FFT_applyZeroPaddingAndWindowComplexQ15(q15_t* pData, q15_t* pBuffer);
static void DSP_FFT_computeMagnitude(q31_t* pData, float* pBuffer, uint16_t size);
static uint16_t DSP_FFT_findMaximum(float* pData, uint16_t start, uint16_t end);
static void DSP_FFT_computeDeltaPhi(q31_t* pDataA, q31_t* pDataB, q31_t* pResult, uint16_t size);
static float DSP_FFT_findPeakLocation(float* pData, uint16_t idx);

void DSP_FFT_init() {
    frameCtr = 0;
    roundCtr = 0;
    bufferIdx = 0;
    refIdx = (MAX_IDX + MIN_IDX) / 2;
    pBuffer = &storageBufferA[bufferIdx][0];
    pBufferOther = &storageBufferB[bufferIdx][0];
    bufferReady = 0;
    DSP_FFT_fillBlackmanWindowQ15(switch_window, FIR_OUTPUT_BLOCK_SIZE);
    DSP_FFT_fillBlackmanWindowComplexQ15(fft_window, FFT_WINDOW_SIZE);
}

void DSP_FFT_receiveData(q15_t* pDataRe, q15_t* pDataIm) {
    q15_t buff_i[FIR_OUTPUT_BLOCK_SIZE];
    q15_t buff_q[FIR_OUTPUT_BLOCK_SIZE];
    arm_mult_q15(pDataRe, switch_window, buff_i, FIR_OUTPUT_BLOCK_SIZE);
    arm_mult_q15(pDataIm, switch_window, buff_q, FIR_OUTPUT_BLOCK_SIZE);
    pDataRe = buff_i;
    pDataIm = buff_q;
    for (uint8_t i = 0; i < FIR_OUTPUT_BLOCK_SIZE; i += 4) {
        *pBuffer++ = *pDataRe++;
        *pBuffer++ = *pDataIm++;
        *pBuffer++ = *pDataRe++;
        *pBuffer++ = *pDataIm++;
        *pBuffer++ = *pDataRe++;
        *pBuffer++ = *pDataIm++;
        *pBuffer++ = *pDataRe++;
        *pBuffer++ = *pDataIm++;
    }
    if (roundCtr != 0) {
        pBufferOther += FIR_OUTPUT_BLOCK_SIZE * 2;
        q15_t* tmp = pBuffer;
        pBuffer = pBufferOther;
        pBufferOther = tmp;
    }
    frameCtr++;
    if (frameCtr >= NUM_FRAMES_TO_PROCESS) {
        frameCtr = 0;
        bufferIdx ^= 1;
        pBuffer = &storageBufferA[bufferIdx][0];
        pBufferOther = &storageBufferB[bufferIdx][0];
        bufferReady = 1;
        if (roundCtr == 0) {
            TIM1->CCR2 = 255;
        }
        roundCtr++;
        if (roundCtr >= PT_RATE) {
            roundCtr = 0;
            TIM1->CCR2 = 0;
        }
    }
}

void DSP_FFT_processDataFromLoop() {
    if (!bufferReady) return;
    // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
    bufferReady = 0;
    uint8_t bufferIdxToUse = bufferIdx ^ 1;
    DSP_FFT_applyZeroPaddingAndWindowComplexQ15(storageBufferA[bufferIdxToUse], fftBuffer);
    arm_q15_to_q31(fftBuffer, fftBufferA, FFT_SIZE * 2);
    arm_shift_q31(fftBufferA, FFT_SCALING,  fftBufferA, FFT_SIZE * 2);
    arm_cfft_q31(&arm_cfft_sR_q31_len1024, fftBufferA, 0, 1);
    if (roundCtr != 1) {
        DSP_FFT_applyZeroPaddingAndWindowComplexQ15(storageBufferB[bufferIdxToUse], fftBuffer);
        arm_q15_to_q31(fftBuffer, fftBufferB, FFT_SIZE * 2);
        arm_shift_q31(fftBufferB, FFT_SCALING, fftBufferB, FFT_SIZE * 2);
        arm_cfft_q31(&arm_cfft_sR_q31_len1024, fftBufferB, 0, 1);
        DSP_FFT_computeDeltaPhi(fftBufferA, fftBufferB, fftDiffAngle, FFT_SIZE);
        DSP_FFT_computeMagnitude(fftDiffAngle, fftMagnitude, FFT_SIZE);

        for (uint32_t i = 0; i < FFT_SIZE; i++) {
            scope_buffer[i * 2] = (uint16_t) roundf(fftMagnitude[i] * 300.0f + 50000.0f);
            scope_buffer[i * 2 + 1] = 0;
        }

        DACScope_startDisplay((uint32_t*) scope_buffer, FFT_SIZE);

        uint16_t idx = DSP_FFT_findMaximum(fftMagnitude, refIdx - SEARCH_RANGE, refIdx + SEARCH_RANGE);
        float peak = DSP_FFT_findPeakLocation(fftMagnitude, idx);
        q31_t angle_re = fftDiffAngle[idx * 2];
        q31_t angle_im = fftDiffAngle[idx * 2 + 1];
        float magnitude = fftMagnitude[idx] / 2;
        DSP_PP_updateFilterState(angle_re, angle_im, magnitude, peak);
    }
    else {
        DSP_FFT_computeMagnitude(fftBufferA, fftMagnitude, FFT_SIZE);
        refIdx = DSP_FFT_findMaximum(fftMagnitude, MIN_IDX, MAX_IDX);
    }
    // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
}

static void DSP_FFT_applyZeroPaddingAndWindowComplexQ15(q15_t* pData, q15_t* pBuffer) {
    arm_mult_q15(pData, fft_window, &pBuffer[FFT_SIZE * 2 - FFT_WINDOW_SIZE], FFT_WINDOW_SIZE);
    arm_mult_q15(&pData[FFT_WINDOW_SIZE], &fft_window[FFT_WINDOW_SIZE], pBuffer, FFT_WINDOW_SIZE);
    arm_fill_q15(0, &pBuffer[FFT_WINDOW_SIZE], FFT_SIZE * 2 - FFT_WINDOW_SIZE * 2);
}

static void DSP_FFT_fillBlackmanWindowQ15(q15_t* pBuffer, uint16_t size) {
    for (uint16_t i = 0; i < size; i++) {
        *pBuffer++ = DSP_FFT_computeBlackmanWindow(i, size - 1);
    }
}

static void DSP_FFT_fillBlackmanWindowComplexQ15(q15_t* pBuffer, uint16_t size) {
    for (uint16_t i = 0; i < size; i++) {
        q15_t value = DSP_FFT_computeBlackmanWindow(i, size - 1);
        *pBuffer++ = value;
        *pBuffer++ = value;
    }
}

static q15_t DSP_FFT_computeBlackmanWindow(uint16_t n, uint16_t size) {
    float angle = (2 * M_PI * n) / size;
    return round(32767 * (0.42f - 0.5f * cos(angle) + 0.08f * cos(angle * 2)));
}

static void DSP_FFT_computeMagnitude(q31_t* pData, float* pBuffer, uint16_t size) {
    for (uint16_t i = 0; i < size * 2; i += 2) {
        float re = (float) *pData++;
        float im = (float) *pData++;
        *pBuffer++ = log10f(re * re + im * im) * 10 - DB_NORM;
    }
}

static uint16_t DSP_FFT_findMaximum(float* pData, uint16_t start, uint16_t end) {
    float currMaxMag = -INFINITY;
    uint16_t currMaxIdx = 0;
    for (uint16_t i = start; i < end; i++) {
        float res = pData[i];
        if (res > currMaxMag) {
            currMaxMag = res;
            currMaxIdx = i;
        }
    }
    return currMaxIdx;
}

static void DSP_FFT_computeDeltaPhi(q31_t* pDataA, q31_t* pDataB, q31_t* pResult, uint16_t size) {
    for (uint16_t i = 0; i < size * 2; i += 2) {
        q31_t reA = pDataA[i] >> PRODUCT_RSHIFT;
        q31_t imA = pDataA[i + 1] >> PRODUCT_RSHIFT;
        q31_t reB = pDataB[i] >> PRODUCT_RSHIFT;
        q31_t imB = pDataB[i + 1] >> PRODUCT_RSHIFT;
        pResult[i] = reA * reB + imA * imB;
        pResult[i + 1] = reA * imB - imA * reB;
    }
}

static float DSP_FFT_findPeakLocation(float* pData, uint16_t idx) {
    float alpha = pData[idx - 1];
    float beta = pData[idx];
    float gamma = pData[idx + 1];
    float p = (alpha - gamma) * (2 * alpha - 4 * beta + 2 * gamma);
    return p + idx;
}
