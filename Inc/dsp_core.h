#ifndef __STM32F4xx_IT_H
#define __STM32F4xx_IT_H

#ifdef __cplusplus
 extern "C" {
#endif

void DSP_CORE_Init();
void DSP_CORE_ProcessBufferI(uint16_t* pBuff);

#ifdef __cplusplus
}
#endif

#endif /* __STM32F4xx_IT_H */
