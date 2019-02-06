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

typedef enum uint8_t  {
    ConfigSwitch = 0,
    RTParam0 = 1,
    RTParam1 = 2,
    FSParam0 = 3,
    FSParam1 = 4,
    FSParam2 = 5,
    SWParam0 = 6,
    SWParam1 = 7,
    SWParam2 = 8,
    SWParam3 = 9,
    SWParam4 = 10,
    SWParam5 = 11,
    DataOut0 = 12,
    DataOut1 = 13,
    ADParam0 = 14,
    ADParam1 = 15,
    ADParam2 = 16,
    ADParam3 = 17,
    ADParam4 = 18,
    Pattern0 = 19,
    Pattern1 = 20,
    Pattern2 = 21,
    Pattern3 = 22
} XE1203_AddrTypeDef;

typedef union
{
    uint8_t buffer[23];
    __attribute__((__packed__)) struct
    {
        uint8_t ConfigSwitch;
        uint8_t RTParam0;
        uint8_t RTParam1;
        uint8_t FSParam0;
        uint8_t FSParam1;
        uint8_t FSParam2;
        uint8_t SWParam0;
        uint8_t SWParam2;
        uint8_t SWParam1;
        uint8_t SWParam3;
        uint8_t SWParam4;
        uint8_t SWParam5;
        uint8_t DataOut0;
        uint8_t DataOut1;
        uint8_t ADParam0;
        uint8_t ADParam1;
        uint8_t ADParam2;
        uint8_t ADParam3;
        uint8_t ADParam4;
        uint8_t Pattern0;
        uint8_t Pattern1;
        uint8_t Pattern2;
        uint8_t Pattern3;
    };
} XE1203_RegStruct;

XE1203_RegStruct XE1203_Config;

void XE1203_SetSpiHandle(SPI_HandleTypeDef *hspi);

void XE1203_SetupConfig();

HAL_StatusTypeDef XE1203_WriteData(uint8_t addr, uint8_t data);

HAL_StatusTypeDef XE1203_ReadData(uint8_t addr, uint8_t *pData);

void XE1203_WriteDataStruct(uint8_t *pReg);

void XE1203_ReadDataStruct(uint8_t *pReg);

#ifdef __cplusplus
}
#endif

#endif /*__XE1203_DRIVER_H */

