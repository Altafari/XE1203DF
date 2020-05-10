#include <stm32f446xx.h>
#include <math.h>
#include <arm_const_structs.h>
#include "downsampling.h"
#include "fourier_analysis.h"
#include "post_processing.h"
#include "stm32f4xx_hal.h"
#include "dac_scope.h"

#define FFT_SIZE 1024
#define NUM_FRAMES_TO_PROCESS 64
#define FFT_WINDOW_SIZE (FIR_OUTPUT_BLOCK_SIZE * NUM_FRAMES_TO_PROCESS)
#define DB_NORM 156.0f
#define TRIPLET_DELTA (FFT_SIZE / (FIR_OUTPUT_BLOCK_SIZE * 2))
#define SEARCH_RANGE (TRIPLET_DELTA / 2)
#define MIN_IDX 400
#define MAX_IDX 850
#define FFT_SCALING (-5)
#define PASSTHROUGH_RATE 8
#define HARMONIC_NORM (M_PI / 4.0f)

typedef struct {
    float re, im;
} Complex_Float_t;

typedef struct {
    Complex_Float_t a, b, c;
} Complex_Float_Triplet_t;

static q15_t switch_window[FIR_OUTPUT_BLOCK_SIZE];
static q15_t fft_window[FFT_WINDOW_SIZE * 2];

static volatile uint32_t bufferIdx;
static volatile uint32_t bufferReady;
static volatile uint32_t roundCtr;

static uint32_t frameCtr;
static q15_t *pBuffer;
static q15_t storageBuffer[2][FFT_WINDOW_SIZE * 2];

static uint32_t refIdx;
static q15_t inputBuffer[FFT_SIZE * 2];
static q31_t fftBuffer[FFT_SIZE * 2];

static float fftMagnitude[FFT_SIZE];

static uint32_t displayBuffer[FFT_SIZE];

static void DSP_FFT_fillBlackmanWindowQ15(q15_t* pBuffer, uint32_t size);
static void DSP_FFT_fillBlackmanWindowComplexQ15(q15_t* pBuffer, uint32_t size);
static q15_t DSP_FFT_computeBlackmanWindow(uint32_t n, uint32_t size);
static void DSP_FFT_applyZeroPaddingAndWindowComplexQ15(q15_t* pData, q15_t* pBuffer);

static void DSP_FFT_computeMagnitude(q31_t* pData, float* pBuffer, uint32_t size);

static uint32_t DSP_FFT_findMaximum(float* pData, uint32_t start, uint32_t end);
static uint32_t DSP_FFT_findMaximumTriplet(float* pData, uint32_t start, uint32_t end);

static float DSP_FFT_findTripletPeakLocation(float *pData, uint32_t idx);

static Complex_Float_t DSP_FFT_vectorAngle(Complex_Float_t a, Complex_Float_t c);
static float DSP_FFT_misalignmentAngle(Complex_Float_t a, Complex_Float_t c);
static Complex_Float_t DSP_FFT_rotateVector(Complex_Float_t a, float phi);

static void DSP_FFT_displaySpectrum(float *buff, uint32_t size);

static float DSP_FFT_linearInterpolation(float p, float a, float b, float c);
static Complex_Float_t DSP_FFT_interpolateFloat(float p, q31_t *pComplexData, int idx);
static Complex_Float_Triplet_t DSP_FFT_interpolateFloatTriplet(float p, q31_t *pComplexData, int idx);

void DSP_FFT_init() {
    frameCtr = 0;
    roundCtr = 0;
    bufferIdx = 0;
    pBuffer = &storageBuffer[0][0];
    refIdx = (MAX_IDX + MIN_IDX) / 2;
    bufferReady = 0;
    DSP_FFT_fillBlackmanWindowQ15(switch_window, FIR_OUTPUT_BLOCK_SIZE);
    DSP_FFT_fillBlackmanWindowComplexQ15(fft_window, FFT_WINDOW_SIZE);
}

void DSP_FFT_receiveData(q15_t* pDataRe, q15_t* pDataIm) {
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
    frameCtr++;
    if (frameCtr >= NUM_FRAMES_TO_PROCESS) {
        frameCtr = 0;
        bufferIdx ^= 1;
        pBuffer = &storageBuffer[bufferIdx][0];
        bufferReady = 1;
        if (roundCtr == 0) {
            TIM1->CCR1 = 255;
        }
        roundCtr++;
        if (roundCtr >= PASSTHROUGH_RATE) {
            roundCtr = 0;
            TIM1->CCR1 = 0;
        }
    }
}

void DSP_FFT_processDataFromLoop() {
    if (!bufferReady) return;
    // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
    bufferReady = 0;
    uint8_t bufferIdxToUse = bufferIdx ^ 1;
    DSP_FFT_applyZeroPaddingAndWindowComplexQ15(storageBuffer[bufferIdxToUse], inputBuffer);
    arm_q15_to_q31(inputBuffer, fftBuffer, FFT_SIZE * 2);
    arm_shift_q31(fftBuffer, FFT_SCALING,  fftBuffer, FFT_SIZE * 2);
    arm_cfft_q31(&arm_cfft_sR_q31_len1024, fftBuffer, 0, 1);
    DSP_FFT_computeMagnitude(fftBuffer, fftMagnitude, FFT_SIZE);
    if (roundCtr != 1) {
        uint32_t idx = DSP_FFT_findMaximumTriplet(fftMagnitude, refIdx - SEARCH_RANGE, refIdx + SEARCH_RANGE);
        float p = DSP_FFT_findTripletPeakLocation(fftMagnitude, idx);
        Complex_Float_Triplet_t trp = DSP_FFT_interpolateFloatTriplet(p, fftBuffer, idx);
        float alpha = DSP_FFT_misalignmentAngle(trp.a, trp.c);
        trp.a = DSP_FFT_rotateVector(trp.a, alpha);
        trp.c = DSP_FFT_rotateVector(trp.c, -alpha);
        alpha = DSP_FFT_misalignmentAngle(trp.a, trp.c);
        Complex_Float_t delta;
        delta.re = (trp.c.re + trp.a.re) * HARMONIC_NORM;
        delta.im = (trp.c.im + trp.a.im) * HARMONIC_NORM;
        Complex_Float_t x, y;
        x.re = trp.b.re + delta.re;
        x.im = trp.b.im + delta.im;
        y.re = trp.b.re - delta.re;
        y.im = trp.b.im - delta.im;
        Complex_Float_t av = DSP_FFT_vectorAngle(x, y);
        float dphi = atan2(av.im, av.re);
        float xMag = sqrtf(x.re * x.re + x.im * x.im);
        float yMag = sqrtf(y.re * y.re + y.im * y.im);
        float imb = 20 * log10f(xMag / yMag);
        DSP_PP_updateFilterState(dphi, alpha , imb, refIdx);
        fftMagnitude[idx] = 0;
        fftMagnitude[idx - TRIPLET_DELTA] = 0;
        fftMagnitude[idx + TRIPLET_DELTA] = 0;
        DSP_FFT_displaySpectrum(fftMagnitude, FFT_SIZE);
    }
    else {
        //DSP_FFT_displaySpectrum(fftMagnitude, FFT_SIZE);
        refIdx = DSP_FFT_findMaximum(fftMagnitude, MIN_IDX, MAX_IDX);
    }
    // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
}

static void DSP_FFT_applyZeroPaddingAndWindowComplexQ15(q15_t* pData, q15_t* pBuffer) {
    arm_mult_q15(pData, fft_window, &pBuffer[FFT_SIZE * 2 - FFT_WINDOW_SIZE], FFT_WINDOW_SIZE);
    arm_mult_q15(&pData[FFT_WINDOW_SIZE], &fft_window[FFT_WINDOW_SIZE], pBuffer, FFT_WINDOW_SIZE);
    arm_fill_q15(0, &pBuffer[FFT_WINDOW_SIZE], FFT_SIZE * 2 - FFT_WINDOW_SIZE * 2);
}

static void DSP_FFT_fillBlackmanWindowQ15(q15_t* pBuffer, uint32_t size) {
    for (uint16_t i = 0; i < size; i++) {
        *pBuffer++ = DSP_FFT_computeBlackmanWindow(i, size - 1);
    }
}

static void DSP_FFT_fillBlackmanWindowComplexQ15(q15_t* pBuffer, uint32_t size) {
    for (uint16_t i = 0; i < size; i++) {
        q15_t value = DSP_FFT_computeBlackmanWindow(i, size - 1);
        *pBuffer++ = value;
        *pBuffer++ = value;
    }
}

static q15_t DSP_FFT_computeBlackmanWindow(uint32_t n, uint32_t size) {
    float angle = (2 * M_PI * n) / size;
    return round(32767 * (0.42f - 0.5f * cos(angle) + 0.08f * cos(angle * 2)));
}

static void DSP_FFT_computeMagnitude(q31_t* pData, float* pBuffer, uint32_t size) {
    for (uint32_t i = 0; i < size * 2; i += 2) {
        float re = (float) *pData++;
        float im = (float) *pData++;
        *pBuffer++ = log10f(re * re + im * im) * 10 - DB_NORM;
    }
}

static uint32_t DSP_FFT_findMaximum(float* pData, uint32_t start, uint32_t end) {
    float currMaxMag = -INFINITY;
    uint32_t currMaxIdx = 0;
    for (uint32_t i = start; i < end; i++) {
        float res = pData[i];
        if (res > currMaxMag) {
            currMaxMag = res;
            currMaxIdx = i;
        }
    }
    return currMaxIdx;
}

static uint32_t DSP_FFT_findMaximumTriplet(float* pData, uint32_t start, uint32_t end) {
    float currMaxMag = -INFINITY;
    uint32_t currMaxIdx = 0;
    for (uint32_t i = start; i < end; i++) {
        float res = pData[i - TRIPLET_DELTA] + pData[i] + pData[i + TRIPLET_DELTA];
        if (res > currMaxMag) {
            currMaxMag = res;
            currMaxIdx = i;
        }
    }
    return currMaxIdx;
}

static float DSP_FFT_findTripletPeakLocation(float *pData, uint32_t idx) {
    float alpha = pData[idx - 1 - TRIPLET_DELTA] + pData[idx - 1] + pData[idx - 1 + TRIPLET_DELTA];
    float beta = pData[idx - TRIPLET_DELTA] + pData[idx] + pData[idx + TRIPLET_DELTA];
    float gamma = pData[idx + 1 - TRIPLET_DELTA] + pData[idx + 1] + pData[idx + 1 + TRIPLET_DELTA];
    return (alpha - gamma) / (2.0f * alpha - 4.0f * beta + 2.0f * gamma);
}

static float DSP_FFT_linearInterpolation(float p, float a, float b, float c) {
    if (p > 0) {
        return (1.0f - p) * b + p * c;
    } else {
        return -p * a + (1.0f + p) * b;
    }
}

static Complex_Float_t DSP_FFT_interpolateFloat(float p, q31_t *pComplexData, int idx) {
    Complex_Float_t res;
    float a = (float) pComplexData[2 * (idx - 1)];
    float b = (float) pComplexData[2 * idx];
    float c = (float) pComplexData[2 * (idx + 1)];
    res.re = DSP_FFT_linearInterpolation(p, a, b, c);
    a = (float) pComplexData[2 * (idx - 1) + 1];
    b = (float) pComplexData[2 * idx + 1];
    c = (float) pComplexData[2 * (idx + 1) + 1];
    res.im = DSP_FFT_linearInterpolation(p, a, b, c);
    return res;
}

static Complex_Float_Triplet_t DSP_FFT_interpolateFloatTriplet(float p, q31_t *pComplexData, int idx) {
    Complex_Float_Triplet_t res;
    res.a = DSP_FFT_interpolateFloat(p, pComplexData, idx - TRIPLET_DELTA);
    res.b = DSP_FFT_interpolateFloat(p, pComplexData, idx);
    res.c = DSP_FFT_interpolateFloat(p, pComplexData, idx + TRIPLET_DELTA);
    return res;
}

static Complex_Float_t DSP_FFT_vectorAngle(Complex_Float_t a, Complex_Float_t c) {
    Complex_Float_t res;
    res.re = a.re * c.re + a.im * c.im;
    res.im = a.re * c.im - a.im * c.re;
    return res;
}

static float DSP_FFT_misalignmentAngle(Complex_Float_t a, Complex_Float_t c) {
    Complex_Float_t d;
    d = DSP_FFT_vectorAngle(a, c);
    return 0.5f * atan2f(d.im, d.re);
}

static Complex_Float_t DSP_FFT_rotateVector(Complex_Float_t a, float phi) {
    float c_cos = cosf(phi);
    float c_sin = sinf(phi);
    Complex_Float_t res;
    res.re = a.re * c_cos - a.im * c_sin;
    res.im = a.im * c_cos + a.re * c_sin;
    return res;
}

static void DSP_FFT_displaySpectrum(float *buff, uint32_t size) {
    for (uint32_t i = 0; i < size; i++) {
        displayBuffer[i] = (uint32_t)roundf(buff[i] * 300 + 50000) & 0x0000FFFF;
    }
    displayBuffer[0] |= 0xFFFF0000;
    DACScope_startDisplay(displayBuffer, size);
}
