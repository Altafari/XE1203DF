#include <math.h>
#include <string.h>
#include "stm32f4xx_hal.h"

#define DELTA_PEAK_THR 2.0f
#define UPD_RATE 0.2f

static float deltaPhiState;
static float deltaPhiDevState;
static float signalMagnitudeState;
static float deltaPeakState;
static float validityState;

static char textBuffer[128];
static UART_HandleTypeDef* hUart;

static void DSP_PP_outputFormattedData();

void DSP_PP_init(UART_HandleTypeDef* huart) {
    hUart = huart;
    deltaPhiState = 0.0f;
    deltaPhiDevState = 0.0f;
    signalMagnitudeState = 0.0f;
    deltaPeakState = 0.0f;
    validityState = 0.0f;
}

void DSP_PP_updateFilterState(float deltaPhi, float sigMagnitude, float deltaPeak) {
    if (fabs(deltaPeak) <= DELTA_PEAK_THR) {
        validityState = (1 - UPD_RATE) * validityState + UPD_RATE;
        float deltaPhiDev = deltaPhiState - deltaPhi;
        deltaPhiState = (1 - UPD_RATE) * deltaPhiState + UPD_RATE * deltaPhi;
        deltaPhiDevState = (1 - UPD_RATE) * deltaPhiDevState + UPD_RATE * deltaPhiDev * deltaPhiDev;
        signalMagnitudeState = (1 - UPD_RATE) * signalMagnitudeState + UPD_RATE * sigMagnitude;
        deltaPeakState = (1 - UPD_RATE) * deltaPeakState + UPD_RATE * deltaPeak * deltaPeak;
    } else {
        validityState = (1 - UPD_RATE);
    }

    if (validityState > 0.5f) {
        DSP_PP_outputFormattedData();
    }
}

void DSP_PP_outputFormattedData() {
    float devPhiRms = sqrtf(deltaPhiDevState * UPD_RATE);
    float devPeakRms = sqrtf(deltaPeakState);
    float rad2deg = 180.0f / M_PI;
    sprintf(textBuffer, "dPhi = %3.1f, phAcc = %1.3f, RSSI = %3.1fdB, dPeak = %2.2f\r\n", deltaPhiState * rad2deg, devPhiRms * rad2deg, signalMagnitudeState, devPeakRms);
    uint16_t strLength = strlen(textBuffer);
    hUart->gState = HAL_UART_STATE_READY;
    HAL_UART_Transmit_DMA(hUart, (uint8_t*)textBuffer, strLength);
}
