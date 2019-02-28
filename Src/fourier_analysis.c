#include <stm32f446xx.h>
#include <math.h>
#include "downsampling.h"
#include "fourier_analysis.h"
#include "arm_const_structs.h"
#include "stm32f4xx_hal.h"

#define FFT_SIZE 1024
#define NUM_FRAMES_TO_PROCESS 50
#define FFT_WINDOW_SIZE (FIR_OUTPUT_BLOCK_SIZE * NUM_FRAMES_TO_PROCESS)

static q15_t switch_window[FIR_OUTPUT_BLOCK_SIZE] = { 32767, 32767, 32767, 32767, 32767, 32767, 32767, 32767  };
static q15_t fft_window[FFT_WINDOW_SIZE * 2];

static uint8_t bufferIdx;
static uint16_t frameCtr;
static q15_t* pBuffer;
static q15_t* pBufferOther;
static q15_t storageBufferA[2][FFT_WINDOW_SIZE * 2];
static q15_t storageBufferB[2][FFT_WINDOW_SIZE * 2];

static uint8_t bufferReady;
static q15_t fftBufferA[FFT_SIZE * 2];
static q15_t fftBufferB[FFT_SIZE * 2];

static void DSP_FFT_fillBlackmanWindowQ15(q15_t* pBuffer, uint16_t size);
static void DSP_FFT_fillBlackmanWindowComplexQ15(q15_t* pBuffer, uint16_t size);
static q15_t DSP_FFT_computeBlackmanWindow(uint16_t n, uint16_t size);

void DSP_FFT_init() {
    frameCtr = 0;
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
        bufferReady = 1;
    }
}

void DSP_FFT_processDataFromLoop() {
    if (!bufferReady) return;
    bufferReady = 0;
    uint8_t bufferIdxToUse = bufferIdx ^ 1;
    arm_mult_q15(&storageBufferA[bufferIdxToUse][0], fft_window, &fftBufferA[FFT_SIZE-FFT_WINDOW_SIZE], FFT_WINDOW_SIZE);
    arm_mult_q15(&storageBufferA[bufferIdxToUse][FFT_WINDOW_SIZE], &fft_window[FFT_WINDOW_SIZE], fftBufferA, FFT_WINDOW_SIZE);
    arm_fill_q15(0, &fftBufferA[FFT_WINDOW_SIZE], FFT_SIZE * 2 - FFT_WINDOW_SIZE * 2);
    arm_mult_q15(&storageBufferB[bufferIdxToUse][0], fft_window, &fftBufferB[FFT_SIZE-FFT_WINDOW_SIZE], FFT_WINDOW_SIZE);
    arm_mult_q15(&storageBufferB[bufferIdxToUse][FFT_WINDOW_SIZE], &fft_window[FFT_WINDOW_SIZE], fftBufferB, FFT_WINDOW_SIZE);
    arm_fill_q15(0, &fftBufferB[FFT_WINDOW_SIZE], FFT_SIZE * 2 - FFT_WINDOW_SIZE * 2);
    arm_cfft_q15(&arm_cfft_sR_q15_len1024, fftBufferA, 0, 1);
    arm_cfft_q15(&arm_cfft_sR_q15_len1024, fftBufferB, 0, 1);
}

void DSP_FFT_fillBlackmanWindowQ15(q15_t* pBuffer, uint16_t size) {
    for (uint16_t i = 0; i < size; i++) {
        *pBuffer++ = DSP_FFT_computeBlackmanWindow(i, size - 1);
    }
}

void DSP_FFT_fillBlackmanWindowComplexQ15(q15_t* pBuffer, uint16_t size) {
    for (uint16_t i = 0; i < size * 2; i++) {
        q15_t value = DSP_FFT_computeBlackmanWindow(i, size - 1);
        *pBuffer++ = value;
        *pBuffer++ = value;
    }
}

q15_t DSP_FFT_computeBlackmanWindow(uint16_t n, uint16_t size) {
    float angle = (M_PI_2 * n) / size;
    return 32767 * round(0.45f - 0.5f * cos(angle) + 0.08f * cos(angle * 2));
}
