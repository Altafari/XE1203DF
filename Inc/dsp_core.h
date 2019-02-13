#ifndef __STM32F4xx_IT_H
#define __STM32F4xx_IT_H

#ifdef __cplusplus
 extern "C" {
#endif

void DSP_CORE_Init();
void DSP_CORE_ProcessBuffer(uint16_t* pBuffI, uint16_t* pBuffQ);

#ifdef __cplusplus
}
#endif

#endif /* __STM32F4xx_IT_H */
