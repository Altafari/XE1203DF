#include <math.h>
#include <string.h>
#include "stm32f4xx_hal.h"
#include "post_processing.h"

static char textBuffer[256];
static UART_HandleTypeDef* hUart;
static const float rad2deg = 180.0f / M_PI;

void DSP_PP_init(UART_HandleTypeDef* huart) {
    hUart = huart;
}

void DSP_PP_updateFilterState(q31_t angle_re, q31_t angle_im, float magnitude, float peak) {

    float dPhi = atan2f((float)angle_im, (float)angle_re);
    sprintf(textBuffer, "dPhi = %3.1f, RSSI = %1.3f, RefIdx=%4.0f\n\r", dPhi * rad2deg, magnitude, peak);
//    sprintf(textBuffer, "dPhi = %3.1f, phAcc = %1.3f, RSSI = %3.1fdB, dPeak = %2.2f\r\n", deltaPhiState * rad2deg, devPhiRms * rad2deg, signalMagnitudeState, devPeakRms);
    uint16_t strLength = strlen(textBuffer);
    hUart->gState = HAL_UART_STATE_READY;
    HAL_UART_Transmit_DMA(hUart, (uint8_t*)textBuffer, strLength);
}

