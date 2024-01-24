/**
  ******************************************************************************
  * @file    ADC/ADC_DualModeInterleaved/CM7/Src/stm32h7xx_hal_msp.c
  * @author  MCD Application Team
  * @brief   HAL MSP module.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2018 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/** @addtogroup STM32H7xx_HAL_Examples
  * @{
  */

/** @defgroup ADC_DualModeInterleaved
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/** @defgroup HAL_MSP_Private_Functions
  * @{
  */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
static void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

/**
* @brief UART MSP Initialization
* This function configures the hardware resources used in this example
* @param huart: UART handle pointer
* @retval None
*/
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(huart->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspInit 0 */

  /* USER CODE END USART3_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3;
    PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* Peripheral clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();

    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**USART3 GPIO Configuration
    PD8     ------> USART3_TX
    PD9     ------> USART3_RX
    */
    GPIO_InitStruct.Pin = STLINK_RX_Pin|STLINK_TX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* USART3 interrupt Init */
    HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspInit 1 */

  /* USER CODE END USART3_MspInit 1 */
  }

}

/**
  * @brief ADC MSP initialization
  *        This function configures the hardware resources used in this example:
  *          - Enable clock of ADC peripheral
  *          - Configure the GPIO associated to the peripheral channels
  *          - Configure the DMA associated to the peripheral
  *          - Configure the NVIC associated to the peripheral interruptions
  * @param hadc: ADC handle pointer
  * @retval None
  */
void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc)
{
  GPIO_InitTypeDef          GPIO_InitStruct;
  static DMA_HandleTypeDef  DmaHandle;

  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* Enable clock of GPIO associated to the peripheral channels */
  ADCx_CHANNELa_GPIO_CLK_ENABLE();
  
  /* Enable clock of ADCx peripheral */
  ADCx_CLK_ENABLE();
  /* ADC Periph interface clock configuration */
  __HAL_RCC_ADC_CONFIG(RCC_ADCCLKSOURCE_CLKP);
  

  if (hadc->Instance == ADCx)
  { 
    /* Enable clock of DMA associated to the peripheral */
    ADCx_DMA_CLK_ENABLE();

    /*##-2- Configure peripheral GPIO ##########################################*/ 
    /* Configure GPIO pins of the selected ADC channels */
    GPIO_InitStruct.Pin = ADCx_CHANNELa_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ADCx_CHANNELa_GPIO_PORT, &GPIO_InitStruct);
    
    //GPIO_InitStruct.Pin = ADCy_CHANNELa_PIN;
    //HAL_GPIO_Init(ADCy_CHANNELa_GPIO_PORT, &GPIO_InitStruct);

    /* Note: ADC slave does not need additional configuration, since it shares  */
    /*       the same clock domain, same GPIO pins (interleaved on the same     */
    /*       channel) and same DMA as ADC master.                               */
  
    /*##-3- Configure the DMA ##################################################*/
    /* Configure DMA parameters (ADC master) */
    DmaHandle.Instance = ADCx_DMA;

    DmaHandle.Init.Request             = DMA_REQUEST_ADC1;
    DmaHandle.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    DmaHandle.Init.PeriphInc           = DMA_PINC_DISABLE;
    DmaHandle.Init.MemInc              = DMA_MINC_ENABLE;
    DmaHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;       /* Transfer from ADC by word to match with ADC configuration: Dual mode, ADC master contains conversion results on data register (32 bits) of ADC master and ADC slave  */
    DmaHandle.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;       /* Transfer to memory by word to match with buffer variable type: word */
    DmaHandle.Init.Mode                = DMA_CIRCULAR;              /* DMA in circular mode to match with ADC configuration: DMA continuous requests */
    DmaHandle.Init.Priority            = DMA_PRIORITY_HIGH;
  
   /* Deinitialize  & Initialize the DMA for new transfer */
    HAL_DMA_DeInit(&DmaHandle);
    HAL_DMA_Init(&DmaHandle);

    /* Associate the initialized DMA handle to the ADC handle */
    __HAL_LINKDMA(hadc, DMA_Handle, DmaHandle);
  
    /*##-4- Configure the NVIC #################################################*/

     /* NVIC configuration for DMA interrupt (transfer completion or error) */
    /* Priority: high-priority */
    HAL_NVIC_SetPriority(ADCx_DMA_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(ADCx_DMA_IRQn);
  }
  
  /* NVIC configuration for ADC interrupt */
  /* Priority: high-priority */
  HAL_NVIC_SetPriority(ADCx_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(ADCx_IRQn);
  
  //HAL_NVIC_SetPriority(ADCy_IRQn, 0, 0);
  //HAL_NVIC_EnableIRQ(ADCy_IRQn);

}

/**
  * @brief ADC MSP de-initialization
  *        This function frees the hardware resources used in this example:
  *          - Disable clock of ADC peripheral
  *          - Revert GPIO associated to the peripheral channels to their default state
  *          - Revert DMA associated to the peripheral to its default state
  *          - Revert NVIC associated to the peripheral interruptions to its default state
  * @param hadc: ADC handle pointer
  * @retval None
  */
void HAL_ADC_MspDeInit(ADC_HandleTypeDef *hadc)
  { 
  /*##-1- Reset peripherals ##################################################*/
  ADCx_FORCE_RESET();
  ADCx_RELEASE_RESET();

  /*##-2- Disable peripherals and GPIO Clocks ################################*/
  /* De-initialize GPIO pin of the selected ADC channel */
  HAL_GPIO_DeInit(ADCx_CHANNELa_GPIO_PORT, ADCx_CHANNELa_PIN);

  /*##-3- Disable the DMA ####################################################*/
  /* De-Initialize the DMA associated to the peripheral */
  if(hadc->DMA_Handle != NULL)
  {
    HAL_DMA_DeInit(hadc->DMA_Handle);
  }
  /*##-4- Disable the NVIC ###################################################*/
  /* Disable the NVIC configuration for DMA interrupt */
  HAL_NVIC_DisableIRQ(ADCx_DMA_IRQn);
  
  /* Disable the NVIC configuration for ADC interrupt */
  HAL_NVIC_DisableIRQ(ADCx_IRQn);
}

/**
  * @brief TIM MSP initialization
  *        This function configures the hardware resources used in this example:
  *          - Enable clock of peripheral
  * @param htim: TIM handle pointer
  * @retval None
  */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
  { 
  /* TIM peripheral clock enable */
  if (htim->Instance == TIMx)
  {
    TIMx_CLK_ENABLE();
  }
#if defined WAVEFORM_VOLTAGE_GENERATION_FOR_TEST
  else if (htim->Instance == TIMy)
  {

    TIMy_CLK_ENABLE();
  }
#endif /* WAVEFORM_VOLTAGE_GENERATION_FOR_TEST */
  else
  { 
    /* Error management can be implemented here */
  }

}
  
/**
  * @brief TIM MSP de-initialization
  *        This function frees the hardware resources used in this example:
  *          - Disable clock of peripheral
  * @param htim: TIM handle pointer
  * @retval None
  */
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef *htim)
{
  /* TIM peripheral clock reset */
  if (htim->Instance == TIM3)
  {
    TIMx_FORCE_RESET();
    TIMx_RELEASE_RESET();
  }
#if defined WAVEFORM_VOLTAGE_GENERATION_FOR_TEST
  else if (htim->Instance == TIMy)
  {
    TIMy_FORCE_RESET();
    TIMy_RELEASE_RESET();
  }
#endif /* WAVEFORM_VOLTAGE_GENERATION_FOR_TEST */
  else
  {
    /* Error management can be implemented here */
  }
}
  

#if defined WAVEFORM_VOLTAGE_GENERATION_FOR_TEST
/**
  * @brief DAC MSP initialization
  *        This function configures the hardware resources used in this example:
  *          - Enable clock of peripheral
  *          - Configure the GPIO associated to the peripheral channels
  *          - Configure the DMA associated to the peripheral
  *          - Configure the NVIC associated to the peripheral interruptions
  * @param hdac: DAC handle pointer
  * @retval None
  */
void HAL_DAC_MspInit(DAC_HandleTypeDef *hdac)
{
  GPIO_InitTypeDef          GPIO_InitStruct;

  static DMA_HandleTypeDef  DmaHandle;


  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* Enable GPIO clock */
  DACx_CHANNEL_GPIO_CLK_ENABLE();
  /* DAC peripheral clock enable */
  DACx_CLK_ENABLE();

  /* Enable clock of DMA associated to the peripheral */
  DACx_CHANNELa_DMA_CLK_ENABLE();


  /*##-2- Configure peripheral GPIO ##########################################*/
  /* DAC Channel1 GPIO pin configuration */
  GPIO_InitStruct.Pin = DACx_CHANNELa_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DACx_CHANNELa_GPIO_PORT, &GPIO_InitStruct);
  
  
  /*##-3- Configure the DMA streams ##########################################*/
  /* Configure DMA parameters */
  DmaHandle.Instance = DACx_CHANNELa_DMA;

  DmaHandle.Init.Request             = DMA_REQUEST_DAC1;
  DmaHandle.Init.Direction           = DMA_MEMORY_TO_PERIPH;
  DmaHandle.Init.PeriphInc           = DMA_PINC_DISABLE;
  DmaHandle.Init.MemInc              = DMA_MINC_ENABLE;
  DmaHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;   /* Transfer to DAC by byte to match with DAC configuration: DAC resolution 8 bits */
  DmaHandle.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;   /* Transfer to DAC by byte to match with DAC configuration: DAC resolution 8 bits */
  DmaHandle.Init.Mode                = DMA_CIRCULAR;
  DmaHandle.Init.Priority            = DMA_PRIORITY_HIGH;
  
  /* Deinitialize  & Initialize the DMA for new transfer */
  HAL_DMA_DeInit(&DmaHandle);  
  HAL_DMA_Init(&DmaHandle);

  /* Associate the initialized DMA handle to the DAC handle */
  if (DmaHandle.Instance == DACx_CHANNELa_DMA)
  {
    __HAL_LINKDMA(hdac, DMA_Handle1, DmaHandle);
  }
  else
  {
    /* Error management can be implemented here */
  }

  /*##-4- Configure the NVIC #################################################*/

  /* NVIC configuration for DMA interrupt (transfer completion or error) */
  /* Priority: high-priority */
  HAL_NVIC_SetPriority(DACx_CHANNELa_DMA_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DACx_CHANNELa_DMA_IRQn);

  /* NVIC configuration for DAC interrupt */
  /* Priority: mid-priority */
  HAL_NVIC_SetPriority(DACx_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DACx_IRQn);
}

/**
  * @brief DAC MSP de-initialization
  *        This function frees the hardware resources used in this example:
  *          - Disable clock of peripheral
  *          - Revert GPIO associated to the peripheral channels to their default state
  *          - Revert DMA associated to the peripheral to its default state
  *          - Revert NVIC associated to the peripheral interruptions to its default state
  * @param hadc: DAC handle pointer
  * @retval None
  */
void HAL_DAC_MspDeInit(DAC_HandleTypeDef *hdac)
{
  /*##-1- Reset peripherals ##################################################*/
  DACx_FORCE_RESET();
  DACx_RELEASE_RESET();

  /*##-2- Disable peripherals and GPIO Clocks ################################*/
  /* De-initialize the ADC Channel GPIO pin */
  HAL_GPIO_DeInit(DACx_CHANNELa_GPIO_PORT, DACx_CHANNELa_PIN);

  /*##-3- Disable the DMA streams ############################################*/
  /* De-Initialize the DMA associated to transmission process */
  HAL_DMA_DeInit(hdac->DMA_Handle1);
  HAL_DMA_DeInit(hdac->DMA_Handle2);

  /*##-4- Disable the NVIC ###################################################*/
  /* Disable the NVIC configuration for DMA interrupt */
  HAL_NVIC_DisableIRQ(DACx_CHANNELa_DMA_IRQn);

  /* Disable the NVIC configuration for DAC interrupt */
  HAL_NVIC_DisableIRQ(DACx_IRQn);
}
#endif /* WAVEFORM_VOLTAGE_GENERATION_FOR_TEST */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

