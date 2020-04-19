#include <math.h>
#include "dac_scope.h"

#define NUM_PERIODS 5.0f
#define BUFF_LEN 1024

/* Private function prototypes -----------------------------------------------*/
static void DAC_DMAConvCpltDual(DMA_HandleTypeDef *hdma);
static void DAC_DMAErrorDual(DMA_HandleTypeDef *hdma);
static void DAC_DMAHalfConvCpltDual(DMA_HandleTypeDef *hdma);
static void fillBuffer();

static DAC_HandleTypeDef* h_dac;
static uint32_t buffer[BUFF_LEN];

/**
  * @brief  Enables DAC and starts conversion of channel.
  * @param  hdac pointer to a DAC_HandleTypeDef structure that contains
  *         the configuration information for the specified DAC.
  * @param  pData The destination peripheral Buffer address.
  * @param  Length The length of data to be transferred from memory to DAC peripheral
  * @param  Alignment Specifies the data alignment for DAC channel.
  *          This parameter can be one of the following values:
  *            @arg DAC_ALIGN_8B_R: 8bit right data alignment selected
  *            @arg DAC_ALIGN_12B_L: 12bit left data alignment selected
  *            @arg DAC_ALIGN_12B_R: 12bit right data alignment selected
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DAC_Start_DMA_Dual(DAC_HandleTypeDef* hdac, uint32_t* pData, uint32_t Length, uint32_t Alignment)
{
    uint32_t tmpreg = 0U;

    /* Check the parameters */
    assert_param(IS_DAC_ALIGN(Alignment));

    /* Process locked */
    __HAL_LOCK(hdac);

    /* Change DAC state */
    hdac->State = HAL_DAC_STATE_BUSY;

    /* Set the DMA transfer complete callback for channel1 */
    hdac->DMA_Handle1->XferCpltCallback = DAC_DMAConvCpltDual;

    /* Set the DMA half transfer complete callback for channel1 */
    hdac->DMA_Handle1->XferHalfCpltCallback = DAC_DMAHalfConvCpltDual;

    /* Set the DMA error callback for channel1 */
    hdac->DMA_Handle1->XferErrorCallback = DAC_DMAErrorDual;

    /* Enable the selected DAC channel1 DMA request */
    hdac->Instance->CR |= DAC_CR_DMAEN1;

    /* Case of use of channel 1 */
    switch(Alignment)
    {
      case DAC_ALIGN_12B_R:
        /* Get DHR12R1 address */
        tmpreg = (uint32_t)&hdac->Instance->DHR12RD;
        break;
      case DAC_ALIGN_12B_L:
        /* Get DHR12L1 address */
        tmpreg = (uint32_t)&hdac->Instance->DHR12LD;
        break;
      case DAC_ALIGN_8B_R:
        /* Get DHR8R1 address */
        tmpreg = (uint32_t)&hdac->Instance->DHR8RD;
        break;
      default:
        break;
    }

    /* Enable the DAC DMA underrun interrupt */
    __HAL_DAC_ENABLE_IT(hdac, DAC_IT_DMAUDR1);

    /* Enable the DMA Stream */
    HAL_DMA_Start_IT(hdac->DMA_Handle1, (uint32_t)pData, tmpreg, Length);

    /* Enable the Peripheral */
    hdac->Instance->CR |=  (DAC_CR_EN1 << DAC_CHANNEL_1) | (DAC_CR_EN1 << DAC_CHANNEL_2);

    /* Process Unlocked */
    __HAL_UNLOCK(hdac);

    /* Return function status */
    return HAL_OK;
}

/**
  * @brief  DMA conversion complete callback.
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *                the configuration information for the specified DMA module.
  * @retval None
  */
static void DAC_DMAConvCpltDual(DMA_HandleTypeDef *hdma)
{
  DAC_HandleTypeDef* hdac = ( DAC_HandleTypeDef* )((DMA_HandleTypeDef* )hdma)->Parent;

  HAL_DAC_ConvCpltCallbackCh1(hdac);

  hdac->State= HAL_DAC_STATE_READY;
}


/**
  * @brief  DMA half transfer complete callback.
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *                the configuration information for the specified DMA module.
  * @retval None
  */
static void DAC_DMAHalfConvCpltDual(DMA_HandleTypeDef *hdma)
{
  DAC_HandleTypeDef* hdac = ( DAC_HandleTypeDef* )((DMA_HandleTypeDef* )hdma)->Parent;
  /* Conversion complete callback */
  HAL_DAC_ConvHalfCpltCallbackCh1(hdac);
}

/**
  * @brief  DMA error callback
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *                the configuration information for the specified DMA module.
  * @retval None
  */
static void DAC_DMAErrorDual(DMA_HandleTypeDef *hdma)
{
  DAC_HandleTypeDef* hdac = ( DAC_HandleTypeDef* )((DMA_HandleTypeDef* )hdma)->Parent;

  /* Set DAC error code to DMA error */
  hdac->ErrorCode |= HAL_DAC_ERROR_DMA;

  HAL_DAC_ErrorCallbackCh1(hdac);

  hdac->State= HAL_DAC_STATE_READY;
}

static void fillBuffer() {
    for(uint32_t i = 0; i < BUFF_LEN; i++) {
        uint32_t ch_i = (uint32_t) roundf(cosf(NUM_PERIODS * 2 * M_PI * (float)i / BUFF_LEN) * 0x7FFF + 0x8000);
        uint32_t ch_q = (uint32_t) roundf(sinf(NUM_PERIODS * 2 * M_PI * (float)i / BUFF_LEN) * 0x7FFF + 0x8000);
        buffer[i] = ch_i | (ch_q << 16);
    }
}

void DACScope_init(DAC_HandleTypeDef* hdac) {
    h_dac = hdac;
    fillBuffer();
    DACScope_startDisplay(buffer, BUFF_LEN);
}

void DACScope_startDisplay(uint32_t *pData, uint32_t length) {
    HAL_DAC_Start_DMA_Dual(h_dac, pData, length, DAC_ALIGN_12B_L);
}


void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef* hdac) {
    HAL_GPIO_WritePin(GPIOA, Scope_sync_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, Scope_sync_Pin, GPIO_PIN_RESET);
}
