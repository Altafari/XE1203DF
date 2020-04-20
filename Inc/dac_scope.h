#ifndef DAC_SCOPE_H_
#define DAC_SCOPE_H_

#include "stm32f4xx_hal.h"

void DACScope_init(DAC_HandleTypeDef* hdac, DMA_HandleTypeDef* h_dma_mem);

void DACScope_startDisplay(uint32_t *pData, uint32_t length);

#endif /* DAC_SCOPE_H_ */
