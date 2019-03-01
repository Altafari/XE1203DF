#include <stm32f446xx.h>
#include <math.h>
#include <arm_const_structs.h>
#include "downsampling.h"
#include "fourier_analysis.h"
#include "post_processing.h"

#define FFT_SIZE 1024
#define NUM_FRAMES_TO_PROCESS 50
#define FFT_WINDOW_SIZE (FIR_OUTPUT_BLOCK_SIZE * NUM_FRAMES_TO_PROCESS)
#define DB_NORM 150.0f
#define TRIPLET_DELTA (FFT_SIZE / (FIR_OUTPUT_BLOCK_SIZE * 2))
#define DC_CORR_UPDATE 1.0E-2f
#define MIN_IDX 750
#define MAX_IDX 850

static q15_t switch_window[FIR_OUTPUT_BLOCK_SIZE];
static q15_t fft_window[FFT_WINDOW_SIZE * 2];

static uint8_t bufferIdx;
static uint16_t frameCtr;
static q15_t* pBuffer;
static q15_t* pBufferOther;
static q15_t storageBufferA[2][FFT_WINDOW_SIZE * 2];
static q15_t storageBufferB[2][FFT_WINDOW_SIZE * 2];

static uint8_t bufferReady;
static q15_t fftBuffer[FFT_SIZE * 2];
static q31_t fftBufferA[FFT_SIZE * 2];
static q31_t fftBufferB[FFT_SIZE * 2];

static float fftMagnitudeA[FFT_SIZE];
static float fftMagnitudeB[FFT_SIZE];

static float dcCorrRe;
static float dcCorrIm;
static q15_t dcCorrReQ15;
static q15_t dcCorrImQ15;

static void DSP_FFT_fillBlackmanWindowQ15(q15_t* pBuffer, uint16_t size);
static void DSP_FFT_fillBlackmanWindowComplexQ15(q15_t* pBuffer, uint16_t size);
static q15_t DSP_FFT_computeBlackmanWindow(uint16_t n, uint16_t size);
static void DSP_FFT_applyZeroPaddingAndWindowComplexQ15(q15_t* pData, q15_t* pBuffer);
static void DSP_FFT_convertQ15BufferToQ31(q15_t* pQ15, q31_t* pQ31);
static void DSP_FFT_computeMagnitude(q31_t* pData, float* pBuffer);
static uint16_t DSP_FFT_findMaximumTriplet(float* pData, uint16_t start, uint16_t end);
static float DSP_FFT_computeDeltaPhi(q31_t* pDataA, q31_t* pDataB, uint16_t idx);
static float DSP_FFT_findPeakLocation(float* pData, uint16_t idx);
static void DSP_FFT_updateDcCorrection(q15_t* pData, float* pState, q15_t corr);

void DSP_FFT_init() {
    frameCtr = 0;
    pBuffer = &storageBufferA[bufferIdx][0];
    pBufferOther = &storageBufferB[bufferIdx][0];
    bufferReady = 0;
    DSP_FFT_fillBlackmanWindowQ15(switch_window, FIR_OUTPUT_BLOCK_SIZE);
    DSP_FFT_fillBlackmanWindowComplexQ15(fft_window, FFT_WINDOW_SIZE);
    dcCorrRe = 0.0f;
    dcCorrIm = 0.0f;
    dcCorrReQ15 = 0;
    dcCorrImQ15 = 0;
}

void DSP_FFT_receiveData(q15_t* pDataRe, q15_t* pDataIm) {
    q15_t buff_i[FIR_OUTPUT_BLOCK_SIZE];
    q15_t buff_q[FIR_OUTPUT_BLOCK_SIZE];
    DSP_FFT_updateDcCorrection(pDataRe, &dcCorrRe, dcCorrReQ15);
    DSP_FFT_updateDcCorrection(pDataIm, &dcCorrIm, dcCorrImQ15);
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

    pBufferOther += FIR_OUTPUT_BLOCK_SIZE * 2;
    q15_t* tmp = pBuffer;
    pBuffer = pBufferOther;
    pBufferOther = tmp;
    frameCtr++;
    if (frameCtr >= NUM_FRAMES_TO_PROCESS) {
        frameCtr = 0;
        bufferIdx++;
        bufferIdx &= 1;
        pBuffer = &storageBufferA[bufferIdx][0];
        pBufferOther = &storageBufferB[bufferIdx][0];
        dcCorrReQ15 = roundf(dcCorrRe);
        dcCorrImQ15 = roundf(dcCorrIm);
        bufferReady = 1;
    }
}

void DSP_FFT_processDataFromLoop() {
    if (!bufferReady) return;
    bufferReady = 0;
    uint8_t bufferIdxToUse = bufferIdx ^ 1;
    DSP_FFT_applyZeroPaddingAndWindowComplexQ15(storageBufferA[bufferIdxToUse], fftBuffer);
    DSP_FFT_convertQ15BufferToQ31(fftBuffer, fftBufferA);
    DSP_FFT_applyZeroPaddingAndWindowComplexQ15(storageBufferB[bufferIdxToUse], fftBuffer);
    DSP_FFT_convertQ15BufferToQ31(fftBuffer, fftBufferB);
    arm_cfft_q31(&arm_cfft_sR_q31_len1024, fftBufferA, 0, 1);
    arm_cfft_q31(&arm_cfft_sR_q31_len1024, fftBufferB, 0, 1);
    DSP_FFT_computeMagnitude(fftBufferA, fftMagnitudeA);
    DSP_FFT_computeMagnitude(fftBufferB, fftMagnitudeB);
    uint16_t idxA = DSP_FFT_findMaximumTriplet(fftMagnitudeA, MIN_IDX, MAX_IDX);
    uint16_t idxB = DSP_FFT_findMaximumTriplet(fftMagnitudeB, MIN_IDX, MAX_IDX);
    float peakA = DSP_FFT_findPeakLocation(fftMagnitudeA, idxA);
    float peakB = DSP_FFT_findPeakLocation(fftMagnitudeB, idxB);
    arm_add_f32(fftMagnitudeA, fftMagnitudeB, fftMagnitudeA, FFT_SIZE);
    uint16_t idx = DSP_FFT_findMaximumTriplet(fftMagnitudeA, MIN_IDX, MAX_IDX);
    volatile float peakDelta = peakA - peakB;
    float dPhi = DSP_FFT_computeDeltaPhi(fftBufferA, fftBufferB, idx);
    DSP_PP_updateFilterState(dPhi, fftMagnitudeA[idx] / 2, peakDelta);
}

void DSP_FFT_applyZeroPaddingAndWindowComplexQ15(q15_t* pData, q15_t* pBuffer) {
    arm_mult_q15(pData, fft_window, &pBuffer[FFT_SIZE * 2 - FFT_WINDOW_SIZE], FFT_WINDOW_SIZE);
    arm_mult_q15(&pData[FFT_WINDOW_SIZE], &fft_window[FFT_WINDOW_SIZE], pBuffer, FFT_WINDOW_SIZE);
    arm_fill_q15(0, &pBuffer[FFT_WINDOW_SIZE], FFT_SIZE * 2 - FFT_WINDOW_SIZE * 2);
}

void DSP_FFT_fillBlackmanWindowQ15(q15_t* pBuffer, uint16_t size) {
    for (uint16_t i = 0; i < size; i++) {
        *pBuffer++ = DSP_FFT_computeBlackmanWindow(i, size - 1);
    }
}

void DSP_FFT_fillBlackmanWindowComplexQ15(q15_t* pBuffer, uint16_t size) {
    for (uint16_t i = 0; i < size; i++) {
        q15_t value = DSP_FFT_computeBlackmanWindow(i, size - 1);
        *pBuffer++ = value;
        *pBuffer++ = value;
    }
}

q15_t DSP_FFT_computeBlackmanWindow(uint16_t n, uint16_t size) {
    float angle = (2 * M_PI * n) / size;
    return round(32767 * (0.42f - 0.5f * cos(angle) + 0.08f * cos(angle * 2)));
}

void DSP_FFT_convertQ15BufferToQ31(q15_t* pQ15, q31_t* pQ31) {
    for (uint16_t i = 0; i < FFT_SIZE * 2; i += 4) {
        *pQ31++ = (*pQ15++) << 16;
        *pQ31++ = (*pQ15++) << 16;
        *pQ31++ = (*pQ15++) << 16;
        *pQ31++ = (*pQ15++) << 16;
    }
}

static void DSP_FFT_updateDcCorrection(q15_t* pData, float* pState, q15_t corr) {
    q31_t sum = 0;
    for (uint16_t i = 0; i < FIR_OUTPUT_BLOCK_SIZE; i++) {
        sum += pData[i];
        pData[i] -= corr;
    }
   *pState = (*pState) * (1 - DC_CORR_UPDATE) + sum * (DC_CORR_UPDATE / FIR_OUTPUT_BLOCK_SIZE);
}


void DSP_FFT_computeMagnitude(q31_t* pData, float* pBuffer) {
    for (uint16_t i = 0; i < FFT_SIZE * 2; i += 2) {
        float re = (float) *pData++;
        float im = (float) *pData++;
        *pBuffer++ = log10(re * re + im * im) * 10 - DB_NORM;
    }
}

uint16_t DSP_FFT_findMaximumTriplet(float* pData, uint16_t start, uint16_t end) {
    float currMaxMag = -INFINITY;
    uint16_t currMaxIdx = 0;
    for (uint16_t i = start; i < end; i++) {
        float res = pData[i - TRIPLET_DELTA] + pData[i] + pData[i + TRIPLET_DELTA];
        if (res > currMaxMag) {
            currMaxMag = res;
            currMaxIdx = i;
        }
    }
    return currMaxIdx;
}

float DSP_FFT_computeDeltaPhi(q31_t* pDataA, q31_t* pDataB, uint16_t idx) {
    float reA = (float) pDataA[idx * 2];
    float imA = (float) pDataA[idx * 2 + 1];
    float reB = (float) pDataB[idx * 2];
    float imB = (float) pDataB[idx * 2 + 1];
    return fmodf(atan2f(reA, imA) - atan2f(reB, imB), M_PI);
}

float DSP_FFT_findPeakLocation(float* pData, uint16_t idx) {
    float alpha = pData[idx - 1];
    float beta = pData[idx];
    float gamma = pData[idx + 1];
    float p = (alpha - gamma) * (2 * alpha - 4 * beta + 2 * gamma);
    return p + idx;
}
