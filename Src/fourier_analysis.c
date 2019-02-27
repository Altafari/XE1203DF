#include <stm32f446xx.h>
#include "downsampling.h"
#include "fourier_analysis.h"
#include "stm32f4xx_hal.h"

#define FFT_SIZE 1024
#define NUM_FRAMES_TO_PROCESS 50

static q15_t window_function[FIR_OUTPUT_BLOCK_SIZE] = { 32767, 32767, 32767, 32767, 32767, 32767, 32767, 32767  };

static uint8_t bufferIdx;
static uint16_t frameCtr;
static q15_t* pBuffer;
static q15_t* pBufferOther;
static q15_t storageBufferA[2][FIR_OUTPUT_BLOCK_SIZE * NUM_FRAMES_TO_PROCESS];
static q15_t storageBufferB[2][FIR_OUTPUT_BLOCK_SIZE * NUM_FRAMES_TO_PROCESS];

static arm_cfft_instance_q15 fftInstance;
static uint8_t bufferReady;
static q15_t fftBufferA[FFT_SIZE * 2];
static q15_t fftBufferB[FFT_SIZE * 2];

void DSP_FFT_init() {
    frameCtr = 0;
    pBuffer = &storageBufferA[bufferIdx][0];
    pBufferOther = &storageBufferB[bufferIdx][0];
    bufferReady = 0;
}

void DSP_FFT_receiveData(q15_t* pDataRe, q15_t* pDataIm) {
    q15_t buff_i[FIR_OUTPUT_BLOCK_SIZE];
    q15_t buff_q[FIR_OUTPUT_BLOCK_SIZE];
    arm_mult_q15(pDataRe, window_function, buff_i, FIR_OUTPUT_BLOCK_SIZE);
    arm_mult_q15(pDataIm, window_function, buff_q, FIR_OUTPUT_BLOCK_SIZE);
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
        DSP_FFT_processDataFromLoop();
    }
}

void DSP_FFT_processDataFromLoop() {
    if (!bufferReady) return;
    uint8_t bufferIdxToUse = bufferIdx ^ 1;
    arm_copy_q15(&storageBufferA[bufferIdxToUse][0], fftBufferA, FIR_OUTPUT_BLOCK_SIZE * NUM_FRAMES_TO_PROCESS * 2);
    arm_fill_q15(0, &fftBufferA[FIR_OUTPUT_BLOCK_SIZE * NUM_FRAMES_TO_PROCESS], FFT_SIZE - FIR_OUTPUT_BLOCK_SIZE * NUM_FRAMES_TO_PROCESS * 2);
    arm_copy_q15(&storageBufferB[bufferIdxToUse][0], fftBufferB, FIR_OUTPUT_BLOCK_SIZE * NUM_FRAMES_TO_PROCESS * 2);
    arm_fill_q15(0, &fftBufferB[FIR_OUTPUT_BLOCK_SIZE * NUM_FRAMES_TO_PROCESS], FFT_SIZE - FIR_OUTPUT_BLOCK_SIZE * NUM_FRAMES_TO_PROCESS * 2);
    arm_cfft_q15(&fftInstance, fftBufferA, 0, 1);
    arm_cfft_q15(&fftInstance, fftBufferB, 0, 1);
}
