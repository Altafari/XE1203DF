/*
 * xe1203_driver.h
 * Defines functions to communicate with XE1203 chip via STM32 Cube HAL SPI driver
 */
#include "xe1203_driver.h"
#define GPIO_CS GPIOB
#define GPIO_PIN_CS GPIO_PIN_12

static SPI_HandleTypeDef *pHspi;
static uint8_t rx_buffer[3];
static uint8_t tx_buffer[3];


void XE1203_SetSpiHandle(SPI_HandleTypeDef *p_Hspi)
{
    pHspi = p_Hspi;
    tx_buffer[2] = 0xFF;
}

HAL_StatusTypeDef XE1203_WriteData(uint8_t addr, uint8_t data)
{
    HAL_GPIO_WritePin(GPIO_CS, GPIO_PIN_CS, GPIO_PIN_RESET);
    tx_buffer[0] = addr & 0b00011111;
    tx_buffer[0] |= 0b10000000;
    tx_buffer[1] = data;
    HAL_StatusTypeDef res = HAL_SPI_Transmit(pHspi, tx_buffer, 3, 100);
    HAL_GPIO_WritePin(GPIO_CS, GPIO_PIN_CS, GPIO_PIN_SET);
    return res;
}

HAL_StatusTypeDef XE1203_ReadData(uint8_t addr, uint8_t *pData)
{
    HAL_GPIO_WritePin(GPIO_CS, GPIO_PIN_CS, GPIO_PIN_RESET);
    tx_buffer[0] = addr & 0b00011111;
    tx_buffer[0] |= 0b10100000;
    tx_buffer[1] = 0xFF;
    HAL_StatusTypeDef res = HAL_SPI_TransmitReceive(pHspi, tx_buffer, rx_buffer, 3, 100);
    HAL_GPIO_WritePin(GPIO_CS, GPIO_PIN_CS, GPIO_PIN_SET);
    *pData = rx_buffer[1];
    return res;
}

void XE1203_WriteDataStruct(uint8_t *pReg)
{
    for(uint8_t i = 0; i < sizeof(XE1203_RegStruct); i++)
    {
        XE1203_WriteData(i, pReg[i]);
    }
}

void XE1203_ReadDataStruct(uint8_t *pReg)
{
    for(uint8_t i = 0; i < sizeof(XE1203_RegStruct); i++)
    {
        XE1203_ReadData(i, &pReg[i]);
    }
}
