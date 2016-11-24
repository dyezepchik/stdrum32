/**
  ******************************************************************************
  * @file    UART/UART_TwoBoards_ComIT/Src/main.c 
  * @author  MCD Application Team
  * @version V1.4.0
  * @date    29-April-2016
  * @brief   This sample code shows how to use UART HAL API to transmit
  *          and receive a data buffer with a communication process based on
  *          IT transfer. 
  *          The communication is done using 2 Boards.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <math.h>

/** @addtogroup STM32F1xx_HAL_Examples
  * @{
  */

/** @addtogroup UART_TwoBoards_ComIT
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#if defined(ADC_TRIGGER_FROM_TIMER)
#define TIMER_FREQUENCY                ((uint32_t) 1000)    /* Timer frequency (unit: Hz). With a timer 16 bits and time base freq min 1Hz, range is min=1Hz, max=32kHz. */
#define TIMER_FREQUENCY_RANGE_MIN      ((uint32_t)    1)    /* Timer minimum frequency (unit: Hz). With a timer 16 bits, maximum frequency will be 32000 times this value. */
#define TIMER_PRESCALER_MAX_VALUE      (0xFFFF-1)           /* Timer prescaler maximum value (0xFFFF for a timer 16 bits) */
#endif /* ADC_TRIGGER_FROM_TIMER */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* UART handler declaration */
/* Buffer used for transmission of 8 channels to MIDI*/
uint8_t aTxBuffer[ADC_CHANNELS_USED_NUM];

UART_HandleTypeDef UartHandle;
__IO ITStatus UartReady = RESET;
__IO uint32_t UserButtonStatus = 0;  /* set to 1 after User Button interrupt  */

/* ADC handler declaration */
ADC_HandleTypeDef    AdcHandle;

#if defined(ADC_TRIGGER_FROM_TIMER)
/* TIM handler declaration */
TIM_HandleTypeDef	TimHandle;
#endif /* ADC_TRIGGER_FROM_TIMER */

/* Variable containing ADC conversions results */
__IO uint16_t	aADCxConvertedValues[ADC_CHANNELS_USED_NUM];	/* ADC conversion results table of regular group, channel on rank1 */

/* Variable to report ADC sequencer status */
uint8_t         SequenceTransfered = RESET;     /* Set when all ranks of the sequence have been converted */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(uint8_t num);
static void ADC_Config(void);

#if defined(ADC_TRIGGER_FROM_TIMER)
static void TIM_Config(void);
#endif /* ADC_TRIGGER_FROM_TIMER */

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /* STM32F1xx HAL library initialization:
       - Configure the Flash prefetch
       - Systick timer is configured by default as source of time base, but user 
         can eventually implement his proper time base source (a general purpose 
         timer for example or other time source), keeping in mind that Time base 
         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
         handled in milliseconds basis.
       - Set NVIC Group Priority to 4
       - Low Level Initialization
  */
  HAL_Init();

  /* Configure the system clock to 24 MHz */
  SystemClock_Config();
  
  /* Configure LED3 and LED4 */
  BSP_LED_Init(LED3);
  BSP_LED_Init(LED4);

  /*##-1- Configure peripherals ######################################*/
  /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
  /* UART configured as follows:
      - Word Length = 8 Bits
      - Stop Bit = One Stop bit
      - Parity = None
      - BaudRate = 9600 baud
      - Hardware flow control disabled (RTS and CTS signals) */
  UartHandle.Instance        = USARTx;

  UartHandle.Init.BaudRate   = 115200;
  UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
  UartHandle.Init.StopBits   = UART_STOPBITS_1;
  UartHandle.Init.Parity     = UART_PARITY_NONE;
  UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  UartHandle.Init.Mode       = UART_MODE_TX_RX;
  if(HAL_UART_DeInit(&UartHandle) != HAL_OK){
    Error_Handler(1);
  }  
  if(HAL_UART_Init(&UartHandle) != HAL_OK){
    Error_Handler(2);
  }
  
  /* Configure User push-button in Interrupt mode */
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);
  
  /* Configure the ADC peripheral */
  ADC_Config();

  /* Run the ADC calibration */
  if (HAL_ADCEx_Calibration_Start(&AdcHandle) != HAL_OK){
    /* Calibration Error */
    Error_Handler(3);
  }

#if defined(ADC_TRIGGER_FROM_TIMER)
  /* Configure the TIM peripheral */
  TIM_Config();
#endif /* ADC_TRIGGER_FROM_TIMER */

  /*## Enable peripherals ####################################################*/
#if defined(ADC_TRIGGER_FROM_TIMER)
  /* Timer enable */
  if (HAL_TIM_Base_Start(&TimHandle) != HAL_OK){
    /* Counter Enable Error */
    Error_Handler(4);
  }
#endif /* ADC_TRIGGER_FROM_TIMER */

  /* Wait for User push-button press before starting the Communication.
     In the meantime, LED3 is blinking */
  while(UserButtonStatus == 0){
    /* Toggle LED3*/
    BSP_LED_Toggle(LED3);
    HAL_Delay(100);
  }
  
  BSP_LED_Off(LED3); 

  /* Start ADC conversion on regular group with transfer by DMA */
  if (HAL_ADC_Start_DMA(&AdcHandle, (uint32_t *)aADCxConvertedValues, ADC_CHANNELS_USED_NUM) != HAL_OK){
    /* Start Error */
    Error_Handler(5);
  }

  /*##-2- Start main loop: ADC conversion, analyze, transmit #####################################*/
  while (1){
	/*## Continuously analyze the converted signal for peak and sustain #####################################*/
  /* downsample the converted values to 8 bit and send them to UART */
	for (uint8_t n=0; n<ADC_CHANNELS_USED_NUM; n++){
	  aTxBuffer[n] = (uint8_t) (aADCxConvertedValues[n] >> 4);
	}
	/*## If peak detected, transmit to MIDI #####################################*/
	while (SequenceTransfered != SET){
	  BSP_LED_Toggle(LED3);
	  HAL_Delay(100);
	}
	BSP_LED_Off(LED3);
	// wait if transmission is not complete
	while (UartReady != SET){
	  BSP_LED_Toggle(LED4);
	  HAL_Delay(100);
	}
	/* Reset transmission flag */
	UartReady = RESET;
	if(HAL_UART_Transmit_IT(&UartHandle, (uint8_t*)aTxBuffer, TXBUFFERSIZE)!= HAL_OK){
	  Error_Handler(7);
	}
	/* Toggle LED4 to show the interrupt is working*/
	SequenceTransfered = RESET;
	//BSP_LED_Toggle(LED4);

  } /* Infinite loop */
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 24000000
  *            HCLK(Hz)                       = 24000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            HSE Frequency(Hz)              = 8000000
  *            HSE PREDIV1                    = 2
  *            PLLMUL                         = 6
  *            Flash Latency(WS)              = 0
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef clkinitstruct = {0};
  RCC_OscInitTypeDef oscinitstruct = {0};
  
  /* Enable HSE Oscillator and activate PLL with HSE as source */
  oscinitstruct.OscillatorType  = RCC_OSCILLATORTYPE_HSE;
  oscinitstruct.HSEState        = RCC_HSE_ON;
  oscinitstruct.HSEPredivValue  = RCC_HSE_PREDIV_DIV2;
  oscinitstruct.PLL.PLLState    = RCC_PLL_ON;
  oscinitstruct.PLL.PLLSource   = RCC_PLLSOURCE_HSE;
  oscinitstruct.PLL.PLLMUL      = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&oscinitstruct)!= HAL_OK)
  {
    /* Initialization Error */
    while(1); 
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  clkinitstruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  clkinitstruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clkinitstruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clkinitstruct.APB2CLKDivider = RCC_HCLK_DIV1;
  clkinitstruct.APB1CLKDivider = RCC_HCLK_DIV1;  
  if (HAL_RCC_ClockConfig(&clkinitstruct, FLASH_LATENCY_0)!= HAL_OK)
  {
    /* Initialization Error */
    while(1); 
  }
}

/**
  * @brief  Tx Transfer completed callback
  * @param  UartHandle: UART handle. 
  * @note   This example shows a simple way to report end of IT Tx transfer, and 
  *         you can add your own implementation. 
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  /* Set transmission flag: transfer complete */
  UartReady = SET;

  /* Turn LED3 on: Transfer in transmission process is correct */
  BSP_LED_On(LED3); 
  
}

/**
  * @brief  Rx Transfer completed callback
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report end of DMA Rx transfer, and 
  *         you can add your own implementation.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  /* Set transmission flag: transfer complete */
  UartReady = SET;
  
  /* Turn LED3 on: Transfer in reception process is correct */
  BSP_LED_On(LED3);
  
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
    Error_Handler(8);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == USER_BUTTON_PIN)
  {  
    UserButtonStatus = 1;
  }
}

static void ADC_Config(void)
{
  ADC_ChannelConfTypeDef   sConfig;
  ADC_InjectionConfTypeDef sConfigInjected;

  AdcHandle.Instance = ADCx;

  AdcHandle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
  AdcHandle.Init.ScanConvMode          = ADC_SCAN_ENABLE;
#if defined ADC_TRIGGER_FROM_TIMER
  AdcHandle.Init.ContinuousConvMode    = DISABLE;
#else
  AdcHandle.Init.ContinuousConvMode    = ENABLE;
#endif
  AdcHandle.Init.NbrOfConversion       = ADC_CHANNELS_USED_NUM;
  AdcHandle.Init.DiscontinuousConvMode = ENABLE;
  AdcHandle.Init.NbrOfDiscConversion   = 1;
#if defined ADC_TRIGGER_FROM_TIMER
  AdcHandle.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_Tx_TRGO;
#else
  AdcHandle.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
#endif

  if (HAL_ADC_Init(&AdcHandle) != HAL_OK){
    Error_Handler(9);
  }

  /* Configuration of channel on ADCx regular group on sequencer rank x */
  /* Note: Considering IT occurring after each number of                      */
  /*       "ADCCONVERTEDVALUES_BUFFER_SIZE" ADC conversions (IT by DMA end    */
  /*       of transfer), select sampling time and ADC clock with sufficient   */
  /*       duration to not create an overhead situation in IRQHandler.        */
  sConfig.Channel      = ADC_CHANNEL_1;
  sConfig.Rank         = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_41CYCLES_5;
  if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK){
    Error_Handler(10);
  }

  sConfig.Channel      = ADC_CHANNEL_2;
  sConfig.Rank         = ADC_REGULAR_RANK_2;
  sConfig.SamplingTime = ADC_SAMPLETIME_41CYCLES_5;
  if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK){
    Error_Handler(10);
  }

  sConfig.Channel      = ADC_CHANNEL_3;
  sConfig.Rank         = ADC_REGULAR_RANK_3;
  sConfig.SamplingTime = ADC_SAMPLETIME_41CYCLES_5;
  if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK){
    Error_Handler(10);
  }

  sConfig.Channel      = ADC_CHANNEL_4;
  sConfig.Rank         = ADC_REGULAR_RANK_4;
  sConfig.SamplingTime = ADC_SAMPLETIME_41CYCLES_5;
  if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK){
    Error_Handler(10);
  }

  sConfig.Channel      = ADC_CHANNEL_5;
  sConfig.Rank         = ADC_REGULAR_RANK_5;
  sConfig.SamplingTime = ADC_SAMPLETIME_41CYCLES_5;
  if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK){
    Error_Handler(10);
  }

  sConfig.Channel      = ADC_CHANNEL_6;
  sConfig.Rank         = ADC_REGULAR_RANK_6;
  sConfig.SamplingTime = ADC_SAMPLETIME_41CYCLES_5;
  if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK){
    Error_Handler(10);
  }

  sConfig.Channel      = ADC_CHANNEL_7;
  sConfig.Rank         = ADC_REGULAR_RANK_7;
  sConfig.SamplingTime = ADC_SAMPLETIME_41CYCLES_5;
  if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK){
    Error_Handler(10);
  }
}

#if defined(ADC_TRIGGER_FROM_TIMER)
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
  /* Timer period can be adjusted by modifying the following constants:       */
  /* - TIMER_FREQUENCY: timer frequency (unit: Hz).                           */
  /* - TIMER_FREQUENCY_RANGE_MIN: timer minimum frequency (unit: Hz).         */

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
  TimHandle.Init.Prescaler         = (timer_prescaler - 1);
  TimHandle.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  TimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
  TimHandle.Init.RepetitionCounter = 0x0;

  if (HAL_TIM_Base_Init(&TimHandle) != HAL_OK)
  {
    /* Timer initialization Error */
    Error_Handler(12);
  }

  /* Timer TRGO selection */
  master_timer_config.MasterOutputTrigger = TIM_TRGO_UPDATE;
  master_timer_config.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

  if (HAL_TIMEx_MasterConfigSynchronization(&TimHandle, &master_timer_config) != HAL_OK)
  {
    /* Timer TRGO selection Error */
    Error_Handler(13);
  }

}
#endif /* ADC_TRIGGER_FROM_TIMER */

void Error_Handler(uint8_t num)
{
  /* Turn LED3 on */
  BSP_LED_On(LED4);
  while(num--){
    BSP_LED_Toggle(LED3);
    HAL_Delay(1000);
  }
  while(1){
  }
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
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
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
