/*
 * xe1203_driver.h
 * Defines functions to communicate with XE1203 chip via STM32 Cube HAL SPI driver
 */
#ifndef __XE1203_DRIVER_H
#define __XE1203_DRIVER_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f4xx_hal.h"

void XE_1203_SetSpiHandle(SPI_HandleTypeDef *hspi);

HAL_StatusTypeDef XE_1203_WriteData(uint8_t addr, uint8_t data);

HAL_StatusTypeDef XE1203_ReadData(uint8_t addr, uint8_t *data);

#ifdef __cplusplus
}
#endif

#endif /*__XE1203_DRIVER_H */

