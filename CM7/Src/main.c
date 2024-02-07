/**
  ******************************************************************************
  * @file    ADC/ADC_DualModeInterleaved/CM7/Src/main.c
  * @author  MCD Application Team
  * @brief   This example provides a short description of how to use the ADC
  *          peripheral to perform conversions in interleaved dual-mode.
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
#include <stdarg.h>
#include <stdlib.h>
#include <stdio.h>
#include "main.h"
#include "string.h"

/** @addtogroup STM32H7xx_HAL_Examples
  * @{
  */

/** @addtogroup ADC_DualModeInterleaved
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Application general parameters */
#define VDD_APPLI                      ((uint32_t) 3300)    /* Value of analog voltage supply Vdda (unit: mV) */
#define RANGE_8BITS                    ((uint32_t)  255)    /* Max digital value with a full range of 8 bits */
#define RANGE_12BITS                   ((uint32_t) 4095)    /* Max digital value with a full range of 12 bits */
#define RANGE_16BITS                   ((uint32_t)65535)    /* Max digital value with a full range of 16 bits */

/* ADC parameters */
//#define ADCCONVERTEDVALUES_BUFFER_SIZE ((uint32_t)  256)    /* Size of array containing ADC converted values */
#define ADCCONVERTEDVALUES_BUFFER_SIZE ((uint32_t)  4)    /* Size of array containing ADC converted values */

#if defined(ADC_TRIGGER_FROM_TIMER)
/* Timer for ADC trigger parameters */
//#define TIMER_FREQUENCY                ((uint32_t) 1000)    /* Timer frequency (unit: Hz). With a timer 16 bits and time base freq min 1Hz, range is min=1Hz, max=32kHz. */
#define TIMER_FREQUENCY                ((uint32_t) 32000)    /* Timer frequency (unit: Hz). With a timer 16 bits and time base freq min 1Hz, range is min=1Hz, max=32kHz. */
#define TIMER_FREQUENCY_RANGE_MIN      ((uint32_t)    1)    /* Timer minimum frequency (unit: Hz), used to calculate frequency range. With a timer 16 bits, maximum frequency will be 32000 times this value. */
#define TIMER_PRESCALER_MAX_VALUE      (0xFFFF-1)           /* Timer prescaler maximum value (0xFFFF for a timer 16 bits) */
#endif /* ADC_TRIGGER_FROM_TIMER */

#define TIMER4_FREQUENCY                ((uint32_t) 32000)    /* Timer frequency (unit: Hz). With a timer 16 bits and time base freq min 1Hz, range is min=1Hz, max=32kHz. */

#if defined(WAVEFORM_VOLTAGE_GENERATION_FOR_TEST)
/* Timer for DAC trigger parameters */
#define TIMER_FOR_WAVEFORM_TEST_FREQUENCY                ((uint32_t)  500)    /* Timer for DAC trigger to send each sample of the waveform: Timer frequency (unit: Hz). With a timer 16 bits and time base freq min 1Hz, range is min=1Hz, max=32kHz. */
#define TIMER_FOR_WAVEFORM_TEST_FREQUENCY_RANGE_MIN      ((uint32_t)    1)    /* Timer for DAC trigger to send each sample of the waveform: Timer minimum frequency used to calculate frequency range (unit: Hz). With timer 16 bits, maximum frequency possible will be 32000 times this value. */
#define TIMER_FOR_WAVEFORM_TEST_PRESCALER_MAX_VALUE      (0xFFFF-1)           /* Timer prescaler maximum value (0xFFFF for a timer 16 bits) */

/* Waveform voltage generation for test parameters */
#define WAVEFORM_TEST_SAMPLES_NUMBER                     ((uint32_t)    5)   /* Size of array of DAC waveform samples */
#define WAVEFORM_TEST_PERIOD_US                          ((WAVEFORM_TEST_SAMPLES_NUMBER * 1000000) / TIMER_FOR_WAVEFORM_TEST_FREQUENCY_HZ)   /* Waveform voltage generation for test period (unit: us) */
#endif /* WAVEFORM_VOLTAGE_GENERATION_FOR_TEST */
/* Private macro -------------------------------------------------------------*/
/**
  * @brief  Computation of ADC master conversion result
  *         from ADC dual mode conversion result (ADC master and ADC slave
  *         results concatenated on data register of ADC master).
  * @param  DATA: ADC dual mode conversion result
  * @retval None
  */
#define COMPUTATION_DUALMODEINTERLEAVED_ADCMASTER_RESULT(DATA)                 \
  ((DATA) & 0x0000FFFF)

/**
  * @brief  Computation of ADC slave conversion result
  *         from ADC dual mode conversion result (ADC master and ADC slave
  *         results concatenated on data register of ADC master).
  * @param  DATA: ADC dual mode conversion result
  * @retval None
  */
#define COMPUTATION_DUALMODEINTERLEAVED_ADCSLAVE_RESULT(DATA)                  \
  ((DATA) >> 16)

#if defined(WAVEFORM_VOLTAGE_GENERATION_FOR_TEST)
/**
  * @brief  Computation of digital value on range 8 bits from voltage value
  *         (unit: mV).
  *         Calculation depends on settings: digital resolution and power
  *         supply of analog voltage Vdda.
  * @param DATA: Voltage value (unit: mV)
  * @retval None
  */
#define COMPUTATION_VOLTAGE_TO_DIGITAL_8BITS(DATA)                             \
  ((DATA) * RANGE_8BITS / VDD_APPLI)
#endif /* WAVEFORM_VOLTAGE_GENERATION_FOR_TEST */
/* Private variables ---------------------------------------------------------*/
/* Peripherals handlers declaration */
/* ADC handler declaration */
ADC_HandleTypeDef    AdcHandle_master;
ADC_HandleTypeDef    AdcHandle_slave;
/* TIM handler declaration */
TIM_HandleTypeDef    TimHandle;
TIM_HandleTypeDef    Tim4Handle;

#if defined(WAVEFORM_VOLTAGE_GENERATION_FOR_TEST)
/* DAC handler declaration */
DAC_HandleTypeDef    DacForWaveformTestHandle;  /* DAC used for waveform voltage generation for test */
/* TIM handler declaration */
TIM_HandleTypeDef    TimForWaveformTestHandle;  /* TIM used for waveform voltage generation for test */
#endif /* WAVEFORM_VOLTAGE_GENERATION_FOR_TEST */

/* Variable containing ADC conversions results */
ALIGN_32BYTES(__IO uint32_t   aADCDualConvertedValues[ADCCONVERTEDVALUES_BUFFER_SIZE]);    /* ADC dual mode interleaved conversion results (ADC master and ADC slave results concatenated on data register 32 bits of ADC master). */
ALIGN_32BYTES(__IO uint16_t   aADCxConvertedValues[ADCCONVERTEDVALUES_BUFFER_SIZE]);       /* For the purpose of this example, dispatch dual conversion values into arrays corresponding to each ADC conversion values. */
ALIGN_32BYTES(__IO uint16_t   aADCyConvertedValues[ADCCONVERTEDVALUES_BUFFER_SIZE]);       /* For the purpose of this example, dispatch dual conversion values into arrays corresponding to each ADC conversion values. */
uint8_t         ubADCDualConversionComplete = RESET;                        /* Set into ADC conversion complete callback */

#if defined(WAVEFORM_VOLTAGE_GENERATION_FOR_TEST)
/* Waveform sent by DAC channel. With timer frequency 1kHz and size of 5 samples: waveform 200Hz */
const uint8_t Waveform_8bits[WAVEFORM_TEST_SAMPLES_NUMBER] =
  {COMPUTATION_VOLTAGE_TO_DIGITAL_8BITS(             0),    /* Expected voltage: 0V,          corresponding digital values: to   0 on 8 bits and    0 and 12 bits */
   COMPUTATION_VOLTAGE_TO_DIGITAL_8BITS(VDD_APPLI *1/4),    /* Expected voltage: 1/4 of Vdda, corresponding digital values: to  63 on 8 bits and 1023 and 12 bits */
   COMPUTATION_VOLTAGE_TO_DIGITAL_8BITS(VDD_APPLI *2/4),    /* Expected voltage: 1/2 of Vdda, corresponding digital values: to 127 on 8 bits and 2047 and 12 bits */
   COMPUTATION_VOLTAGE_TO_DIGITAL_8BITS(VDD_APPLI *3/4),    /* Expected voltage: 3/4 of Vdda, corresponding digital values: to 191 on 8 bits and 3071 and 12 bits */
   COMPUTATION_VOLTAGE_TO_DIGITAL_8BITS(VDD_APPLI    )};    /* Expected voltage: Vdda,        corresponding digital values: to 255 on 8 bits and 4095 and 12 bits */
#endif /* WAVEFORM_VOLTAGE_GENERATION_FOR_TEST */

/* Private function prototypes -----------------------------------------------*/
static void MPU_Config(void);
static void SystemClock_Config(void);
static void Error_Handler(void);
static void ADC_Config(void);
static void CPU_CACHE_Enable(void);
#if defined(ADC_TRIGGER_FROM_TIMER)
static void TIM_Config(void);
static void TIM4_Config(void);
#endif /* ADC_TRIGGER_FROM_TIMER */
#if defined(WAVEFORM_VOLTAGE_GENERATION_FOR_TEST)
static void WaveformVoltageGenerationForTest(void);
#endif /* WAVEFORM_VOLTAGE_GENERATION_FOR_TEST */
/* Private functions ---------------------------------------------------------*/

uint16_t adraw[2];
uint8_t uart2_raw[10];
UART_HandleTypeDef huart3;
uint8_t uart3_raw[10];
volatile int rx_flagA = 0;
volatile int rx_flagB = 0;
volatile int rx_flagG = 0;

int aShot = 0;
int bShot = 0;
int convrate = 0;

char writeBuf[100];

char strA1[50];
volatile uint16_t ad1_raw[5];
const int adcChannelCount = 2;
volatile int adcConversionComplete = 0;
volatile int lock = 0;
volatile uint32_t millis = 0;
volatile uint32_t conv_rate = 0;

uint32_t ad1 = 0;
uint32_t ad2 = 0;

#define BUFSIZE		1000

int32_t sawtooth_buf1[BUFSIZE];
int32_t sawtooth_buf2[BUFSIZE];
int32_t signal_buf[BUFSIZE];
int32_t signal_buf1[BUFSIZE];
int32_t signal_buf2[BUFSIZE];
int32_t kalman_buf1[BUFSIZE];
int32_t kalman_buf2[BUFSIZE];
int32_t peaks_buff1[BUFSIZE];
int32_t peaks_buff2[BUFSIZE];
volatile int signal_buffer_in_queue = 1;
volatile int gidxB = 0;
volatile int gidxA = 0;

volatile uint32_t relative_sawtooth_voltage = 0;

int FindPeak(uint32_t *sig)
{

	if((sig[0] < sig[1]) && (sig[2] < sig[1]))
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

void insert_new_value(int32_t *buf, int32_t new_value)
{
	for(gidxA=0;gidxA<199;gidxA++)
	{
		buf[gidxA] = buf[gidxA+1];
	}

	buf[199] = new_value;
}

int flag_FallingEdge = 0;
int flag_saving = 0;

int file_name_index = 0;

//FIL log_file;
int log_file_opened = 0;

static float ADC_OLD_Value;
static float P_k1_k1;

static float Q = 0.0001;//Q: Regulation noise, Q increases, dynamic response becomes faster, and convergence stability becomes worse
//static float Q = 0.0005;//Q: Regulation noise, Q increases, dynamic response becomes faster, and convergence stability becomes worse
static float R = 0.005; //R: Test noise, R increases, dynamic response becomes slower, convergence stability becomes better
//static float R = 0.2;
static float Kg = 0;
static float P_k_k1 = 0.5;
static float kalman_adc_old=0;
static int kalman_adc_int = 0;

unsigned long kalman_filter(unsigned long ADC_Value)
{
    float x_k1_k1,x_k_k1;
    //static float ADC_OLD_Value;
    float Z_k;


    float kalman_adc;

    Z_k = ADC_Value;
    x_k1_k1 = kalman_adc_old;

    x_k_k1 = x_k1_k1;
    P_k_k1 = P_k1_k1 + Q;

    Kg = P_k_k1/(P_k_k1 + R);

    kalman_adc = x_k_k1 + Kg * (Z_k - kalman_adc_old);
    P_k1_k1 = (1 - Kg)*P_k_k1;
    P_k_k1 = P_k1_k1;

    ADC_OLD_Value = ADC_Value;
    kalman_adc_old = kalman_adc;
    kalman_adc_int = (int)kalman_adc;
    return kalman_adc;
}

int getSimplifiedSlope(int32_t *buf)
{
	int32_t avg1 = (buf[0] + buf[1] + buf[2] + buf[3] + buf[4]) / 5;
	int32_t avg2 = (buf[5] + buf[6] + buf[7] + buf[8] + buf[9]) / 5;

	if((avg1 - avg2) > 1000)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

int max = 0;
int min = 5000;

int gmaxA = 0;
int gminA = 0;
int midlineA = 0;
int dripOff = 0;

uint32_t crate = 0;

uint32_t __t2_cntr = 0;
volatile uint32_t __dripA = 0;
uint32_t flag_saving_interval = 0;

int32_t GetMidLine(int32_t *gbuff, uint32_t sz)
{
	int lidxA = 0;

	for(lidxA = 1; lidxA<(sz - 1 ); lidxA++)
	{
		if(gbuff[lidxA] > max)
		{
			max = gbuff[lidxA];
		}
	}

	for(lidxA = 1; lidxA<(sz - 1 ); lidxA++)
	{
		if(gbuff[lidxA] < min)
		{
			min = gbuff[lidxA];
		}
	}

	return (((max - min)/2) + min);
}




/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

void myprintf(const char *fmt, ...) {
  static char buffer[100];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);

  int len = strlen(buffer);
//  _write(0, (char*)buffer,len);
  HAL_UART_Transmit(&huart3, (uint8_t*)buffer, len, -1);

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart3)
	{
		if(uart3_raw[0] == 'a')
		{
			rx_flagA = 1;
			//HAL_UART_Receive_IT(&huart2, uart2_raw, 1);
		}

		if(uart3_raw[0] == 'b')
		{
			rx_flagB = 1;
			//HAL_UART_Receive_IT(&huart2, uart2_raw, 1);
		}

		rx_flagG = 1;
	}
	HAL_UART_Receive_IT(&huart3, uart3_raw, 1);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	int loop_A = 0;
	while(1)
	{
		HAL_Delay(1000);
		loop_A++;

		if(loop_A > 10)
		{
			break;
		}
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &Tim4Handle)
	{
		if(__t2_cntr < 3)
		{
			__t2_cntr++;
		}
		else
		{
			__t2_cntr = 0;
		}

		if(__dripA > 0)
		{
			__dripA++;

			//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

			if(__dripA > 16) // 500uS
			{
				flag_FallingEdge = 0;
				__dripA = 0;
				//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
				//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
			}
		}
	}
}

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  int32_t timeout;

  int lidxA = 0;

  /* System Init, System clock, voltage scaling and L1-Cache configuration are done by CPU1 (Cortex-M7)
     in the meantime Domain D2 is put in STOP mode(Cortex-M4 in deep-sleep)
  */

  /* Configure the MPU attributes */
  MPU_Config();

  /* Enable the CPU Cache */
  CPU_CACHE_Enable();

  /* Wait until CPU2 boots and enters in stop mode or timeout*/
  timeout = 0xFFFF;
  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
  if ( timeout < 0 )
  {
    Error_Handler();
  }

 /* STM32H7xx HAL library initialization:
       - Systick timer is configured by default as source of time base, but user
         can eventually implement his proper time base source (a general purpose
         timer for example or other time source), keeping in mind that Time base
         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and
         handled in milliseconds basis.
       - Set NVIC Group Priority to 4
       - Low Level Initialization
     */
  HAL_Init();

  /* Configure the system clock to 400 MHz */
  SystemClock_Config();

  /*## Configure peripherals #################################################*/

  /* Initialize LEDs on board */
  BSP_LED_Init(LED3);
  BSP_LED_Init(LED1);

  //BSP_LED_On(LED1);


  /* Configure the ADCx and ADCy peripherals */
  ADC_Config();

#if defined(ADC_TRIGGER_FROM_TIMER)
  /* Configure the TIM peripheral */
  TIM_Config();
#endif

  TIM4_Config();

  /*## Enable peripherals ####################################################*/
#if defined(ADC_TRIGGER_FROM_TIMER)
  /* Timer enable */
  if (HAL_TIM_Base_Start(&TimHandle) != HAL_OK)
  {
    /* Counter Enable Error */
    Error_Handler();
  }
#endif /* ADC_TRIGGER_FROM_TIMER */

if (HAL_TIM_Base_Start_IT(&Tim4Handle) != HAL_OK)
{
  /* Counter Enable Error */
  Error_Handler();
}

#if defined(WAVEFORM_VOLTAGE_GENERATION_FOR_TEST)
  /* Generate a periodic signal on a spare DAC channel */
  WaveformVoltageGenerationForTest();
#endif /* WAVEFORM_VOLTAGE_GENERATION_FOR_TEST */


  /*## Start ADC conversions #################################################*/

  /* Start ADCx and ADCy multimode conversion on regular group with transfer by DMA */
  if (HAL_ADCEx_MultiModeStart_DMA(&AdcHandle_master,
                                   (uint32_t *)aADCDualConvertedValues,
                                    ADCCONVERTEDVALUES_BUFFER_SIZE
                                  ) != HAL_OK)
  {
    /* Start Error */
    Error_Handler();
  }

  MX_USART3_UART_Init();

  HAL_UART_Receive_IT(&huart3, uart3_raw, 1);

  myprintf("Starting ... \r\n");
  HAL_Delay(500);
  myprintf("After Delay ... \r\n");

  /* Array "aADCDualConvertedValues" contains both ADC results on 32 bits:    */
  /*  - ADC master results in the 16 LSB [15:0]                               */
  /*  - ADC slave results in the 16 MSB [31:16]                               */

  aShot  = HAL_GetTick();

  /* Infinite loop */
  while (1)
  {
    /* Turn-on/off LED1 in function of ADC conversion result */
    /*  - Turn-off if ADC conversions buffer is not complete */
    /*  - Turn-on if ADC conversions buffer is complete */

    /* ADC conversion buffer complete variable is updated into ADC conversions*/
    /* complete callback.                                                     */
//    if (ubADCDualConversionComplete == RESET)
//    {
//      ; //BSP_LED_Off(LED1);
//    }
//    else
//    {
//      ; //BSP_LED_On(LED1);
//    }

    if(HAL_GetTick() > (aShot + 500))
    {
    	aShot  = HAL_GetTick();

//    	myprintf("ADC[1] = %d  ADC[2] = %d \r\n", aADCxConvertedValues[0], aADCyConvertedValues[0]);
//    	myprintf("Rate : %d\r\n", convrate);
    	convrate = 0;

    	if(rx_flagA == 1)
    	{
    		if(adcConversionComplete == 1)
			{
			  adcConversionComplete = 0;
			  for(lidxA=0;lidxA<BUFSIZE;lidxA++)
			  {
				  //myprintf("A0:%d, A1:%d\n", signal_buf[lidxA], sawtooth_buf[lidxA]);
				  if(signal_buffer_in_queue == 2)
				  {
					  //myprintf("A0:%d\n", signal_buf1[lidxA]);
					  myprintf("%d,%d,%d,%d,%d\r\n", signal_buf1[lidxA], sawtooth_buf1[lidxA], kalman_buf1[lidxA], peaks_buff1[lidxA], midlineA); //GetMidLine(kalman_buf1, 200));
				  }
				  else
				  {
					  //myprintf("A0:%d\n", signal_buf2[lidxA]);
					  myprintf("%d,%d,%d,%d,%d\r\n", signal_buf2[lidxA], sawtooth_buf2[lidxA], kalman_buf2[lidxA], peaks_buff2[lidxA], midlineA); // GetMidLine(kalman_buf2, 200));
				  }
			  }
			}
			HAL_Delay(2500);
			gidxB = 0; // Fresh Copy of ADC
			rx_flagA = 0;
			rx_flagB = 0;
    	}

    	//__dripA = 1;
    }

//    if(ubADCDualConversionComplete == SET)
//    {
//    	myprintf("ADC[1] = %d  ADC[2] = %d \r\n", aADCxConvertedValues[0], aADCyConvertedValues[0]);
//    }

    /* For information: ADC conversion results are stored into array          */
    /* "aADCDualConvertedValues" (for debug: check into watch window)         */

    /* For the purpose of this example, dual conversion values are            */
    /* dispatched into 2 arrays corresponding to each ADC conversion values.  */
    /* (aADCxConvertedValues, aADCyConvertedValues)                           */
  }
}
/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSE BYPASS)
  *            SYSCLK(Hz)                     = 400000000 (CPU Clock)
  *            HCLK(Hz)                       = 200000000 (Cortex-M4 CPU, Bus matrix Clocks)
  *            AHB Prescaler                  = 2
  *            D1 APB3 Prescaler              = 2 (APB3 Clock  100MHz)
  *            D2 APB1 Prescaler              = 2 (APB1 Clock  100MHz)
  *            D2 APB2 Prescaler              = 2 (APB2 Clock  100MHz)
  *            D3 APB4 Prescaler              = 2 (APB4 Clock  100MHz)
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 4
  *            PLL_N                          = 400
  *            PLL_P                          = 2
  *            PLL_Q                          = 4
  *            PLL_R                          = 2
  *            VDD(V)                         = 3.3
  *            Flash Latency(WS)              = 4
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  HAL_StatusTypeDef ret = HAL_OK;

  /*!< Supply configuration update enable */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /* The voltage scaling allows optimizing the power consumption when the device is
     clocked below the maximum system frequency, to update the voltage scaling value
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
  RCC_OscInitStruct.CSIState = RCC_CSI_OFF;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;

  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 400;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;

  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_1;
  ret = HAL_RCC_OscConfig(&RCC_OscInitStruct);
  if(ret != HAL_OK)
  {
    Error_Handler();
  }

/* Select PLL as system clock source and configure  bus clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_D1PCLK1 | RCC_CLOCKTYPE_PCLK1 | \
                                 RCC_CLOCKTYPE_PCLK2  | RCC_CLOCKTYPE_D3PCLK1);

  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;
  ret = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4);
  if(ret != HAL_OK)
  {
    Error_Handler();
  }

  /*
  Note : The activation of the I/O Compensation Cell is recommended with communication  interfaces
          (GPIO, SPI, FMC, QSPI ...)  when  operating at  high frequencies(please refer to product datasheet)
          The I/O Compensation Cell activation  procedure requires :
        - The activation of the CSI clock
        - The activation of the SYSCFG clock
        - Enabling the I/O Compensation Cell : setting bit[0] of register SYSCFG_CCCSR

          To do this please uncomment the following code
  */

  /*
  __HAL_RCC_CSI_ENABLE() ;

  __HAL_RCC_SYSCFG_CLK_ENABLE() ;

  HAL_EnableCompensationCell();
  */
}

/**
  * @brief  ADC configuration
  * @param  None
  * @retval None
  */
static void ADC_Config(void)
{
  ADC_ChannelConfTypeDef   sConfig;
  ADC_MultiModeTypeDef     MultiModeInit;

  /* Configuration of ADC (master) init structure: ADC parameters and regular group */
  AdcHandle_master.Instance = ADCx;

  if (HAL_ADC_DeInit(&AdcHandle_master) != HAL_OK)
  {
    /* ADC initialization error */
    Error_Handler();
  }
  AdcHandle_slave.Instance = ADCy;
  if (HAL_ADC_DeInit(&AdcHandle_slave) != HAL_OK)
  {
    /* ADC initialization error */
    Error_Handler();
}

  AdcHandle_master.Init.ClockPrescaler           = ADC_CLOCK_ASYNC_DIV2;            /* Asynchronous clock mode, input ADC clock divided by 2*/
  AdcHandle_master.Init.Resolution               = ADC_RESOLUTION_16B;              /* 16-bit resolution for converted data */
  AdcHandle_master.Init.ScanConvMode             = DISABLE;                         /* Sequencer disabled (ADC conversion on only 1 channel: channel set on rank 1) */
  AdcHandle_master.Init.EOCSelection             = ADC_EOC_SINGLE_CONV;             /* EOC flag picked-up to indicate conversion end */
  AdcHandle_master.Init.LowPowerAutoWait         = DISABLE;                         /* Auto-delayed conversion feature disabled */
#if defined(ADC_TRIGGER_FROM_TIMER)
  AdcHandle_master.Init.ContinuousConvMode       = DISABLE;                         /* Continuous mode disabled to have only 1 conversion at each conversion trig */
#else
  AdcHandle_master.Init.ContinuousConvMode       = ENABLE;                          /* Continuous mode to have maximum conversion speed (no delay between conversions) */
#endif
  AdcHandle_master.Init.NbrOfConversion          = 1;                               /* Parameter discarded because sequencer is disabled */
  AdcHandle_master.Init.DiscontinuousConvMode    = DISABLE;                         /* Parameter discarded because sequencer is disabled */
  AdcHandle_master.Init.NbrOfDiscConversion      = 1;                               /* Parameter discarded because sequencer is disabled */
#if defined(ADC_TRIGGER_FROM_TIMER)
  AdcHandle_master.Init.ExternalTrigConv         = ADC_EXTERNALTRIG_T3_TRGO;        /* Timer 3 external event triggering the conversion */
  AdcHandle_master.Init.ExternalTrigConvEdge     = ADC_EXTERNALTRIGCONVEDGE_RISING;
#else
  AdcHandle_master.Init.ExternalTrigConv         = ADC_SOFTWARE_START;              /* Software start to trigger the 1st conversion manually, without external event */
  AdcHandle_master.Init.ExternalTrigConvEdge     = ADC_EXTERNALTRIGCONVEDGE_NONE;   /* Parameter discarded because trigger of conversion by software start (no external event) */
#endif
  AdcHandle_master.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR; /* DMA circular mode selected */
  AdcHandle_master.Init.Overrun                  = ADC_OVR_DATA_OVERWRITTEN;        /* DR register is overwritten with the last conversion result in case of overrun */
  AdcHandle_master.Init.OversamplingMode         = DISABLE;                         /* No oversampling */

  if (HAL_ADC_Init(&AdcHandle_master) != HAL_OK)
  {
    /* ADC initialization error */
    Error_Handler();
  }

  /* Configuration of ADC (slave) init structure: ADC parameters and regular group */
  AdcHandle_slave.Instance = ADCy;

  /* Same configuration as ADC master, with continuous mode and external      */
  /* trigger disabled since ADC master is triggering the ADC slave            */
  /* conversions                                                              */
  AdcHandle_slave.Init = AdcHandle_master.Init;
  AdcHandle_slave.Init.ContinuousConvMode    = DISABLE;
  AdcHandle_slave.Init.ExternalTrigConv      = ADC_SOFTWARE_START;

  if (HAL_ADC_Init(&AdcHandle_slave) != HAL_OK)
  {
    /* ADC initialization error */
    Error_Handler();
  }


  /* Configuration of channel on ADC (master) regular group on sequencer rank 1 */
  /* Note: Considering IT occurring after each number of                      */
  /*       "ADCCONVERTEDVALUES_BUFFER_SIZE" ADC conversions (IT by DMA end    */
  /*       of transfer), select sampling time and ADC clock with sufficient   */
  /*       duration to not create an overhead situation in IRQHandler.        */
  sConfig.Channel      = ADCx_CHANNELa;                /* Sampled channel number */
  sConfig.Rank         = ADC_REGULAR_RANK_1;          /* Rank of sampled channel number ADCx_CHANNEL */
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;    /* Minimum sampling time */
  sConfig.SingleDiff   = ADC_SINGLE_ENDED;            /* Single-ended input channel */
  sConfig.OffsetNumber = ADC_OFFSET_NONE;             /* No offset subtraction */
  sConfig.Offset = 0;                                 /* Parameter discarded because offset correction is disabled */

  if (HAL_ADC_ConfigChannel(&AdcHandle_master, &sConfig) != HAL_OK)
  {
    /* Channel Configuration Error */
    Error_Handler();
  }

  /* Configuration of channel on ADC (slave) regular group on sequencer rank 1 */
  /* Same channel as ADCx for dual mode interleaved: both ADC are converting   */
  /* the same channel.                                                         */
  sConfig.Channel = ADCy_CHANNELa;

  if (HAL_ADC_ConfigChannel(&AdcHandle_slave, &sConfig) != HAL_OK)
  {
    /* Channel Configuration Error */
    Error_Handler();
  }

  /* Run the ADC calibration in single-ended mode */
  if (HAL_ADCEx_Calibration_Start(&AdcHandle_master, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED) != HAL_OK)
  {
    /* Calibration Error */
    Error_Handler();
  }

  if (HAL_ADCEx_Calibration_Start(&AdcHandle_slave, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED) != HAL_OK)
  {
    /* Calibration Error */
    Error_Handler();
  }

  /* Configuration of multimode */
  /* Multimode parameters settings and set ADCy (slave) under control of      */
  /* ADCx (master).                                                           */
  MultiModeInit.Mode = ADC_DUALMODE_INTERL;
  MultiModeInit.DualModeData = ADC_DUALMODEDATAFORMAT_32_10_BITS;  /* ADC and DMA configured in resolution 32 bits to match with both ADC master and slave resolution */
  MultiModeInit.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_1CYCLE;

  if (HAL_ADCEx_MultiModeConfigChannel(&AdcHandle_master, &MultiModeInit) != HAL_OK)
  {
    /* Multimode Configuration Error */
    Error_Handler();
  }

}

/**
  * @brief  TIM configuration
  * @param  None
  * @retval None
  */
static void TIM4_Config(void)
{
	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef master_timer_config;
  RCC_ClkInitTypeDef clk_init_struct = {0};       /* Temporary variable to retrieve RCC clock configuration */
  uint32_t latency;                               /* Temporary variable to retrieve Flash Latency */

  uint32_t timer_clock_frequency = 0;             /* Timer clock frequency */
  uint32_t timer_prescaler = 0;                   /* Time base prescaler to have timebase aligned on minimum frequency possible */

  /* Configuration of timer as time base:                                     */
  /* Caution: Computation of frequency is done for a timer instance on APB1   */
  /*          (clocked by PCLK1)                                              */
  /* Timer frequency is configured from the following constants:              */
  /* - TIMER_FREQUENCY: timer frequency (unit: Hz).                           */
  /* - TIMER_FREQUENCY_RANGE_MIN: timer minimum frequency possible            */
  /*   (unit: Hz).                                                            */
  /* Note: Refer to comments at these literals definition for more details.   */

  /* Retrieve timer clock source frequency */
  HAL_RCC_GetClockConfig(&clk_init_struct, &latency);
  /* If APB1 prescaler is different of 1, timers have a factor x2 on their    */
  /* clock source.                                                            */
  if (clk_init_struct.APB1CLKDivider == RCC_HCLK_DIV1)
  {
    timer_clock_frequency = HAL_RCC_GetPCLK1Freq();
  }
  else
  {
    timer_clock_frequency = HAL_RCC_GetPCLK1Freq() *2;
  }

  /* Timer prescaler calculation */
  /* (computation for timer 16 bits, additional + 1 to round the prescaler up) */
  timer_prescaler = (timer_clock_frequency / (TIMER_PRESCALER_MAX_VALUE * TIMER_FREQUENCY_RANGE_MIN)) +1;

  /* Set timer instance */
  Tim4Handle.Instance = TIM4;

  /* Configure timer parameters */
  Tim4Handle.Init.Period            = ((timer_clock_frequency / (timer_prescaler * TIMER4_FREQUENCY)) - 1);
  Tim4Handle.Init.Prescaler         = (timer_prescaler - 1);
  Tim4Handle.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  Tim4Handle.Init.CounterMode       = TIM_COUNTERMODE_UP;
  Tim4Handle.Init.RepetitionCounter = 0x0;
  Tim4Handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

  if (HAL_TIM_Base_Init(&Tim4Handle) != HAL_OK)
  {
    /* Timer initialization Error */
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&Tim4Handle, &sClockSourceConfig) != HAL_OK)
	{
	  Error_Handler();
	}

  /* Timer TRGO selection */
  //master_timer_config.MasterOutputTrigger = TIM_TRGO_UPDATE;
  //master_timer_config.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  master_timer_config.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

  if (HAL_TIMEx_MasterConfigSynchronization(&Tim4Handle, &master_timer_config) != HAL_OK)
  {
    /* Timer TRGO selection Error */
    Error_Handler();
  }
}

#if defined(ADC_TRIGGER_FROM_TIMER)
/**
  * @brief  TIM configuration
  * @param  None
  * @retval None
  */
static void TIM_Config(void)
{
  TIM_MasterConfigTypeDef master_timer_config;
  RCC_ClkInitTypeDef clk_init_struct = {0};       /* Temporary variable to retrieve RCC clock configuration */
  uint32_t latency;                               /* Temporary variable to retrieve Flash Latency */

  uint32_t timer_clock_frequency = 0;             /* Timer clock frequency */
  uint32_t timer_prescaler = 0;                   /* Time base prescaler to have timebase aligned on minimum frequency possible */

  /* Configuration of timer as time base:                                     */
  /* Caution: Computation of frequency is done for a timer instance on APB1   */
  /*          (clocked by PCLK1)                                              */
  /* Timer frequency is configured from the following constants:              */
  /* - TIMER_FREQUENCY: timer frequency (unit: Hz).                           */
  /* - TIMER_FREQUENCY_RANGE_MIN: timer minimum frequency possible            */
  /*   (unit: Hz).                                                            */
  /* Note: Refer to comments at these literals definition for more details.   */

  /* Retrieve timer clock source frequency */
  HAL_RCC_GetClockConfig(&clk_init_struct, &latency);
  /* If APB1 prescaler is different of 1, timers have a factor x2 on their    */
  /* clock source.                                                            */
  if (clk_init_struct.APB1CLKDivider == RCC_HCLK_DIV1)
  {
    timer_clock_frequency = HAL_RCC_GetPCLK1Freq();
  }
  else
  {
    timer_clock_frequency = HAL_RCC_GetPCLK1Freq() *2;
  }

  /* Timer prescaler calculation */
  /* (computation for timer 16 bits, additional + 1 to round the prescaler up) */
  timer_prescaler = (timer_clock_frequency / (TIMER_PRESCALER_MAX_VALUE * TIMER_FREQUENCY_RANGE_MIN)) +1;

  /* Set timer instance */
  TimHandle.Instance = TIMx;

  /* Configure timer parameters */
  TimHandle.Init.Period            = ((timer_clock_frequency / (timer_prescaler * TIMER_FREQUENCY)) - 1);
  TimHandle.Init.Prescaler         = 40; //(timer_prescaler - 1);
  TimHandle.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  TimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
  TimHandle.Init.RepetitionCounter = 0x0;

  if (HAL_TIM_Base_Init(&TimHandle) != HAL_OK)
  {
    /* Timer initialization Error */
    Error_Handler();
  }

  /* Timer TRGO selection */
  master_timer_config.MasterOutputTrigger = TIM_TRGO_UPDATE;
  master_timer_config.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  master_timer_config.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

  if (HAL_TIMEx_MasterConfigSynchronization(&TimHandle, &master_timer_config) != HAL_OK)
  {
    /* Timer TRGO selection Error */
    Error_Handler();
  }

}
#endif /* ADC_TRIGGER_FROM_TIMER */

#if defined(WAVEFORM_VOLTAGE_GENERATION_FOR_TEST)
/**
  * @brief  For this example, generate a periodic signal on a spare DAC
  *         channel, so user has just to connect a wire between DAC channel
  *         (pin PA.04) and ADC channel (pin PA.04) to run this example.
  *         (this prevents the user from resorting to an external signal generator)
  * @param  None
  * @retval None
  */
static void WaveformVoltageGenerationForTest(void)
  {
  DAC_ChannelConfTypeDef sConfig;
  TIM_MasterConfigTypeDef master_timer_config;
  RCC_ClkInitTypeDef clk_init_struct = {0};       /* Temporary variable to retrieve RCC clock configuration */
  uint32_t latency;                               /* Temporary variable to retrieve Flash Latency */

  uint32_t timer_clock_frequency = 0;             /* Timer clock frequency */
  uint32_t timer_prescaler = 0;                   /* Time base prescaler to have timebase aligned on minimum frequency possible */

  /* Configuration of timer as time base:                                     */
  /* Caution: Computation of frequency is done for a timer instance on APB1   */
  /*          (clocked by PCLK1)                                              */
  /* - TIMER_FOR_WAVEFORM_TEST_FREQUENCY: timer frequency (unit: Hz).         */
  /* - TIMER_FOR_WAVEFORM_TEST_FREQUENCY_RANGE_MIN: time base minimum         */
  /*   frequency possible (unit: Hz).                                         */
  /* Note: Refer to comments at these literals definition for more details.   */

  /* Retrieve timer clock source frequency */
  HAL_RCC_GetClockConfig(&clk_init_struct, &latency);
  /* If APB1 prescaler is different of 1, timers have a factor x2 on their    */
  /* clock source.                                                            */
  if (clk_init_struct.APB1CLKDivider == RCC_HCLK_DIV1)
  {
    timer_clock_frequency = HAL_RCC_GetPCLK1Freq();
  }
  else
  {
    timer_clock_frequency = HAL_RCC_GetPCLK1Freq() *2;
  }

  /* Timer prescaler calculation */
  /* (computation for timer 16 bits, additional + 1 to round the prescaler up) */
  timer_prescaler = (timer_clock_frequency / (TIMER_FOR_WAVEFORM_TEST_PRESCALER_MAX_VALUE * TIMER_FOR_WAVEFORM_TEST_FREQUENCY_RANGE_MIN)) +1;

  /* Set timer instance */
  TimForWaveformTestHandle.Instance = TIMy;

  /* Configure timer parameters */
  TimForWaveformTestHandle.Init.Period            = ((timer_clock_frequency / (timer_prescaler * TIMER_FOR_WAVEFORM_TEST_FREQUENCY)) - 1);
  TimForWaveformTestHandle.Init.Prescaler         = (timer_prescaler - 1);
  TimForWaveformTestHandle.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  TimForWaveformTestHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
  TimForWaveformTestHandle.Init.RepetitionCounter = 0x0;

  if (HAL_TIM_Base_Init(&TimForWaveformTestHandle) != HAL_OK)
  {
    /* Timer initialization Error */
    Error_Handler();
  }

  /* Timer TRGO selection */
  master_timer_config.MasterOutputTrigger = TIM_TRGO_UPDATE;
  master_timer_config.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  master_timer_config.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

  if (HAL_TIMEx_MasterConfigSynchronization(&TimForWaveformTestHandle, &master_timer_config) != HAL_OK)
  {
    /* Timer TRGO selection Error */
    Error_Handler();
  }


  /* Configuration of DACx peripheral */
  DacForWaveformTestHandle.Instance = DACx;

  /* DeInitialize the DAC peripheral */
  if (HAL_DAC_DeInit(&DacForWaveformTestHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
  /* Initialize the DAC peripheral */
  if (HAL_DAC_Init(&DacForWaveformTestHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
}
  /* Configuration of DAC channel */
  sConfig.DAC_Trigger = DACx_TRIGGER_Tx_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;

  if (HAL_DAC_ConfigChannel(&DacForWaveformTestHandle, &sConfig, DACx_CHANNELa) != HAL_OK)
  {
    /* Channel configuration error */
    Error_Handler();
  }


  /*## Enable peripherals ####################################################*/

  /* Timer counter enable */
  if (HAL_TIM_Base_Start(&TimForWaveformTestHandle) != HAL_OK)
  {
    /* Counter Enable Error */
    Error_Handler();
  }

  /* Enable DAC Channel1 and associated DMA */
  if (HAL_DAC_Start_DMA(&DacForWaveformTestHandle, DACx_CHANNELa, (uint32_t *)Waveform_8bits, WAVEFORM_TEST_SAMPLES_NUMBER, DAC_ALIGN_8B_R) != HAL_OK)
  {
    /* Start DMA Error */
    Error_Handler();
  }

}
#endif /* WAVEFORM_VOLTAGE_GENERATION_FOR_TEST */

/**
  * @brief  Conversion complete callback in non blocking mode
  * @param  AdcHandle : ADC handle
  * @note   This example shows a simple way to report end of conversion
  *         and get conversion result. You can add your own implementation.
  * @note   When ADC_TRIGGER_FROM_TIMER is disabled, conversions are software-triggered
  *         and are too fast for DMA post-processing. Therefore, to reduce the computational
  *         load, the output buffer filled up by the DMA is post-processed only when
  *         ADC_TRIGGER_FROM_TIMER is enabled.
  * @retval None
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *AdcHandle)
{

   /* Invalidate Data Cache to get the updated content of the SRAM on the second half of the ADC converted data buffer: 32 bytes */
  SCB_InvalidateDCache_by_Addr((uint32_t *) &aADCDualConvertedValues[ADCCONVERTEDVALUES_BUFFER_SIZE/2], 4*ADCCONVERTEDVALUES_BUFFER_SIZE/2);

#if defined(ADC_TRIGGER_FROM_TIMER)
  uint32_t tmp_index = 0;

  convrate++;

  /* For the purpose of this example, dispatch dual conversion values         */
  /* into 2 arrays corresponding to each ADC conversion values.               */
  for (tmp_index = (ADCCONVERTEDVALUES_BUFFER_SIZE/2); tmp_index < ADCCONVERTEDVALUES_BUFFER_SIZE; tmp_index++)
  {
    aADCxConvertedValues[tmp_index] = (uint16_t) COMPUTATION_DUALMODEINTERLEAVED_ADCMASTER_RESULT(aADCDualConvertedValues[tmp_index]);
    aADCyConvertedValues[tmp_index] = (uint16_t) COMPUTATION_DUALMODEINTERLEAVED_ADCSLAVE_RESULT(aADCDualConvertedValues[tmp_index]);
  }
#endif /* ADC_TRIGGER_FROM_TIMER */

  adcConversionComplete = 1;

  for (tmp_index = 0; tmp_index < ADCCONVERTEDVALUES_BUFFER_SIZE; tmp_index++)
  {
	  ad1 = aADCxConvertedValues[tmp_index];
	  ad2 = aADCyConvertedValues[tmp_index];

	  	if(rx_flagA == 0)
	  	{
	  		if(gidxB == BUFSIZE)
	  		{
	  			if(signal_buffer_in_queue == 1)
	  			{
	  				signal_buffer_in_queue = 2;
	  			}
	  			else
	  			{
	  				signal_buffer_in_queue = 1;
	  			}
	  			gidxB = 0;
	  		}

	  		if(gidxB > 5)
	  		{
	  			if(gmaxA < kalman_buf1[gidxB])
	  			{
	  				gmaxA = kalman_buf1[gidxB];
	  			}

	  			if(gminA > kalman_buf1[gidxB])
	  			{
	  				gminA = kalman_buf1[gidxB];
	  			}
	  		}

	  		if(gidxB == (BUFSIZE - 5))
	  		{
	  			midlineA = (((gmaxA - gminA)/2) + gminA);
	  		}

	  		if(signal_buffer_in_queue == 1)
	  		{
	  			signal_buf1[gidxB] = ad1;
	  			sawtooth_buf1[gidxB] = ad2;
	  			kalman_buf1[gidxB] = kalman_filter(signal_buf1[gidxB]);

	  			if(gidxB==0)
	  			{
	  				gminA = kalman_buf1[0];
	  				gmaxA = kalman_buf1[0];
	  			}

	  			if((gidxB >= 5) && (gidxB < (BUFSIZE - 10)))
	  			{

	  				if(FindPeak(&kalman_buf1[gidxB-3]) && (kalman_buf1[gidxB-3] > midlineA))
	  				{
	  					if(dripOff == 0)
	  					{
	  						peaks_buff1[gidxB] = 2000;
	  						dripOff = 20;
	  						relative_sawtooth_voltage = (3300000 / 4096) * sawtooth_buf1[gidxB-3]; // sawtooth_buf1[gidxB-3]; //
	  					}
	  					else
	  					{
	  						peaks_buff1[gidxB] = 500;
	  					}
	  				}
	  				else
	  				{
	  					peaks_buff1[gidxB] = 500;
	  				}
	  			}
	  			else
	  			{
	  				peaks_buff1[gidxB] = 500;
	  			}
	  		}
	  		else
	  		{
	  			signal_buf2[gidxB] = ad1;
	  			sawtooth_buf2[gidxB] = ad2;
	  			kalman_buf2[gidxB] = kalman_filter(signal_buf2[gidxB]);

	  			if(gidxB==0)
	  			{
	  				gminA = kalman_buf2[0];
	  				gmaxA = kalman_buf2[0];
	  			}

	  			if((gidxB >= 5) && (gidxB < (BUFSIZE - 10)))
	  			{
	  				if(FindPeak(&kalman_buf2[gidxB-3]) && (kalman_buf2[gidxB-3] > midlineA))
	  				{
	  					if(dripOff == 0)
	  					{
	  						peaks_buff2[gidxB] = 2000;
	  						relative_sawtooth_voltage = (3300000 / 4096) * sawtooth_buf2[gidxB-3]; // sawtooth_buf2[gidxB-3]; //
	  						dripOff = 20;
	  					}
	  					else
	  					{
	  						peaks_buff2[gidxB] = 500;
	  					}
	  				}
	  				else
	  				{
	  					peaks_buff2[gidxB] = 500;
	  				}
	  			}
	  			else
	  			{
	  				peaks_buff2[gidxB] = 500;
	  			}
	  		}
	  		gidxB++;
	  		if(dripOff > 0)
	  		{
	  			dripOff--;
	  		}
	  	}
  }

  /* Set variable to report DMA transfer status to main program */
  ubADCDualConversionComplete = SET;
}

/**
  * @brief  Conversion DMA half-transfer callback in non blocking mode
  * @param  hadc: ADC handle
  * @note   When ADC_TRIGGER_FROM_TIMER is disabled, conversions are software-triggered
  *         and are too fast for DMA post-processing. Therefore, to reduce the computational
  *         load, the output buffer filled up by the DMA is post-processed only when
  *         ADC_TRIGGER_FROM_TIMER is enabled.
  * @retval None
  */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
  /* Invalidate Data Cache to get the updated content of the SRAM on the first half of the ADC converted data buffer: 32 bytes */
  SCB_InvalidateDCache_by_Addr((uint32_t *) &aADCDualConvertedValues[0], 4*ADCCONVERTEDVALUES_BUFFER_SIZE/2);

#if(defined(ADC_TRIGGER_FROM_TIMER))
  uint32_t tmp_index = 0;

  /* For the purpose of this example, dispatch dual conversion values         */
  /* into 2 arrays corresponding to each ADC conversion values.               */
  for (tmp_index = 0; tmp_index < (ADCCONVERTEDVALUES_BUFFER_SIZE/2); tmp_index++)
  {
    aADCxConvertedValues[tmp_index] = (uint16_t) COMPUTATION_DUALMODEINTERLEAVED_ADCMASTER_RESULT(aADCDualConvertedValues[tmp_index]);
    aADCyConvertedValues[tmp_index] = (uint16_t) COMPUTATION_DUALMODEINTERLEAVED_ADCSLAVE_RESULT(aADCDualConvertedValues[tmp_index]);
  }
#endif /* ADC_TRIGGER_FROM_TIMER */

  /* Reset variable to report DMA transfer status to main program */
  ubADCDualConversionComplete = RESET;
}

/**
  * @brief  ADC error callback in non blocking mode
  *        (ADC conversion with interruption or transfer by DMA)
  * @param  hadc: ADC handle
  * @note   When ADC_TRIGGER_FROM_TIMER is disabled, conversions are software-triggered
  *         and are too fast for DMA post-processing. Overrun issues are observed and to
  *         avoid ending up in the infinite loop of Error_Handler(), no call to this
  *         latter is done in case of HAL_ADC_ERROR_OVR error.
  * @retval None
  */
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
#if !defined(ADC_TRIGGER_FROM_TIMER)
  /* In case of ADC error, call main error handler */
  if (HAL_IS_BIT_CLR(hadc->ErrorCode, HAL_ADC_ERROR_OVR))
  {
#endif /* ADC_TRIGGER_FROM_TIMER */
  Error_Handler();
#if !defined(ADC_TRIGGER_FROM_TIMER)
  }
#endif /* ADC_TRIGGER_FROM_TIMER */
}

/**
  * @brief  This function is executed in case of error occurrence.
* @param  None
* @retval None
*/
static void Error_Handler(void)
{
  /* User may add here some code to deal with a potential error */

  /* In case of error, LED3 is toggling at a frequency of 1Hz */
  while(1)
  {
    /* Toggle LED3 */
    BSP_LED_Toggle(LED3);
    HAL_Delay(500);
  }
}

/**
  * @brief  Configure the MPU attributes
  * @param  None
  * @retval None
  */
static void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct;

  /* Disable the MPU */
  HAL_MPU_Disable();

  /* Configure the MPU as Strongly ordered for not defined regions */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.BaseAddress = 0x00;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /* Enable the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @brief  CPU L1-Cache enable.
  * @param  None
  * @retval None
  */
static void CPU_CACHE_Enable(void)
{
  /* Enable I-Cache */
  SCB_EnableICache();

  /* Enable D-Cache */
  SCB_EnableDCache();
}

/**
  * @}
  */

/**
  * @}
  */

