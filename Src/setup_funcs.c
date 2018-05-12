
#include "main.h"
#include "stm32f0xx_hal.h"
#include "setup_funcs.h"
#include "spi_driver.h"
/*
 * setup_funcs.c
 *
 *  Created on: 16 באפר׳ 2017
 *      Author: maor
 */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */
extern GPIO_TypeDef* pGP_LED ;
extern	GPIO_TypeDef* pGP_BTN ;
extern	GPIO_TypeDef* pGP_I2C ;
extern GPIO_TypeDef* pGP_SPI;
extern I2C_TypeDef* pI2c;
static RCC_TypeDef* pRCC =  RCC;
EXTI_TypeDef* pEXTI = EXTI;
SYSCFG_TypeDef* pSYSCFG = SYSCFG;
/* Private variables ---------------------------------------------------------*/


/** System Clock Configuration
*/
void SystemClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks
    */


  //step one: turn on used clocks.
  RCC->CR |= RCC_CR_HSION_Msk;
  RCC->CR2 |= RCC_CR2_HSI48ON_Msk|RCC_CR2_HSI14ON_Msk;
  RCC->CSR |= RCC_CSR_LSION_Msk;

  //step two: calibrate all clocks
  RCC->CR &= ~RCC_CR_HSITRIM_Msk;
  RCC->CR |= 16<<RCC_CR_HSITRIM_Pos;
  RCC->CR2 &= ~RCC_CR2_HSI14TRIM_Msk;
  RCC->CR2|= 16<<RCC_CR2_HSI14TRIM_Pos;

  //step three: setup pll values
  RCC->CFGR &= ~RCC_CFGR_SWS_Msk|~RCC_CFGR_PLLMUL_Msk;
  RCC->CFGR |=  RCC_CFGR_SWS_HSI|RCC_PLL_MUL4;
  RCC->CFGR2 = RCC_PREDIV_DIV2;

  //step four: turn on the pll and wait for it to be ready.
  RCC->CR |= RCC_CR_PLLON;
  while(!((RCC->CR)&RCC_CR_PLLRDY_Msk));

  //at this point all basic setup was complete, now to setup peripheral clocks.

  //set the system clk to the pll output.
  RCC->CFGR |= RCC_CFGR_SW_PLL;



  //todo: check if RTC needs this function still since the others are set correctly by default.
//  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_I2C1
//                              |RCC_PERIPHCLK_RTC;
//  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
//  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
//  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
//
//  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
//  {
//    Error_Handler();
//  }

    /**Configure the Systick interrupt time
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC init function */
 void MX_ADC_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
    */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure for the selected ADC regular channel to be converted.
    */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* I2C1 init function */
 void MX_I2C1_Init(void)
{
	__I2C1_CLK_ENABLE();

	__I2C1_FORCE_RESET();
	__I2C1_RELEASE_RESET();







	pI2c->CR1 = 0;//disable the peripheral before configuring.

	//set the relevant pins to i2c mode.
	pGP_I2C->AFR9 = GPIO_AF4_I2C1;
	pGP_I2C->AFR10 = GPIO_AF4_I2C1;
	pGP_I2C->MODER9 = 0b10;
	pGP_I2C->MODER10 = 0b10;
	pGP_I2C->OT9 = 1;
	pGP_I2C->OT10 = 1;



	//set the timing for 400khz clk.

	//todo: timing seems to be wrong, clk pulses look bad.
	pI2c->TIMINGR |= 11<<I2C_TIMINGR_SCLL_Pos;
	pI2c->TIMINGR |= 2 <<I2C_TIMINGR_SCLH_Pos;

	pI2c->CR2 &= ~I2C_CR2_ADD10_Msk;

	pI2c->CR1 |= I2C_PECR_PEC_Msk;
}

/* RTC init function */
 void MX_RTC_Init(void)
{

  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef sDate;
  RTC_AlarmTypeDef sAlarm;

    /**Initialize RTC Only
    */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initialize RTC and set the Time and Date
    */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

    /**Enable the Alarm A
    */
  sAlarm.AlarmTime.Hours = 0x0;
  sAlarm.AlarmTime.Minutes = 0x0;
  sAlarm.AlarmTime.Seconds = 0x0;
  sAlarm.AlarmTime.SubSeconds = 0x0;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 0x1;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

}

/* SPI1 init function */
 void MX_SPI1_Init(void)
{
	 __SPI1_CLK_ENABLE();


	 __SPI1_FORCE_RESET();
	 __SPI1_RELEASE_RESET();

	 //configure the spi ios

	 //set afr to spi
	 pGP_SPI-> AFR3 = GPIO_AF0_SPI1;
	 pGP_SPI->AFR4 = GPIO_AF0_SPI1;
	 pGP_SPI->AFR5 = GPIO_AF0_SPI1;

	 //set pins to push pull
	 pGP_SPI->MODER3 = 0b10;
	 pGP_SPI->MODER4=	 0b10;
	 pGP_SPI->MODER5 = 0b10;

	 //set pins to push pull
	 pGP_SPI->OT3 = 0;
	 pGP_SPI->OT4 = 0;
	 pGP_SPI->OT5 = 0;

	 //set pins to high speed
	 pGP_SPI->OSPEEDR3 = GPIO_SPEED_FREQ_HIGH;
	 pGP_SPI->OSPEEDR4 = GPIO_SPEED_FREQ_HIGH;
	 pGP_SPI->OSPEEDR5 = GPIO_SPEED_FREQ_HIGH;


	 pGP_I2C->AFR15 = 0;
	 pGP_I2C->MODER15 = 0b01;
	 pGP_I2C->OT15 = 0;
	 pGP_SPI->OSPEEDR15 = GPIO_SPEED_FREQ_HIGH;

	 spi_init();


}

/* TIM1 init function */
 void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/

 void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : GP_LED_Pin */
  pGP_LED->MODER3 = GPIO_MODE_OUTPUT_PP;
  pGP_LED->PUPDR3 =GPIO_NOPULL;
  pGP_LED->OSPEEDR3 = GPIO_SPEED_FREQ_LOW;

  /*Configure GPIO pin : USR_BTN_Pin */
  pGP_BTN->MODER0 = GPIO_MODE_INPUT;
  pGP_BTN->PUPDR0 =GPIO_NOPULL;
  pGP_BTN->OSPEEDR0 = GPIO_SPEED_FREQ_LOW;


  //Connect usr btn to external interrupt
  pSYSCFG->EXTICR[0] |= 1;


  pEXTI->IMR 	|= 1;
  //pEXTI->RTSR 	|= 1;
  pEXTI->FTSR 	|= 1;

  //turn on exti0 interrupt

  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

  //unmask interrupt


}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler */
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */

/**
  * @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
