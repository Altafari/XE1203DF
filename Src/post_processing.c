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

void DSP_PP_updateFilterState(float a, float b, float c, float peak) {
    sprintf(textBuffer, "a = %1.3f, b = %1.3f, c = %1.3f\n\r", a, b, c);
//    sprintf(textBuffer, "dPhi = %3.1f, phAcc = %1.3f, RSSI = %3.1fdB, dPeak = %2.2f\r\n", deltaPhiState * rad2deg, devPhiRms * rad2deg, signalMagnitudeState, devPeakRms);
    uint16_t strLength = strlen(textBuffer);
    hUart->gState = HAL_UART_STATE_READY;
    HAL_UART_Transmit_DMA(hUart, (uint8_t*)textBuffer, strLength);
}

