/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */
#include <stdint.h>
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define BATT_SAMPLE_Pin GPIO_PIN_2
#define BATT_SAMPLE_GPIO_Port GPIOA
#define GP_LED_Pin GPIO_PIN_3
#define GP_LED_GPIO_Port GPIOA
#define USR_BTN_Pin GPIO_PIN_0
#define USR_BTN_GPIO_Port GPIOB
#define SPI_CS_Pin GPIO_PIN_15
#define SPI_CS_GPIO_Port GPIOA



#define SPI_FLASH_USER_MEMORY_START 0x10000

#define BTN_PRESS_NONE 0
#define BTN_PRESS_SHORT 1
#define BTN_PRESS_LONG 2
#define BTN_PRESS_LONG_TIME 1000 //test this.
/* USER CODE BEGIN Private defines */


#define TICKS_PER_SEC 2000
typedef enum {init_state,main_screen_state,usb_state,config_state,record_setup_state,lcdoff_state} main_state_type;

typedef struct{
	main_state_type eMainstate;


	uint8_t nSecondsBetweenSamplesidx;

	uint16_t nCurrentTemp;

	union
	{
		uint8_t nStatusByte;
		struct{
			unsigned bLog	: 1;
			unsigned bUsbconnected :1;
			unsigned nBtnPressed	:2; //0 no press, 1 short press, 2 long press.
			unsigned bLcdoff		:1;
			unsigned nCurrentmenuitem:2; //0 start,1 settings,2 exit
			unsigned bLedon			: 1;
		};
	};



} system_handler_type;



volatile system_handler_type sSysHandler;

/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
