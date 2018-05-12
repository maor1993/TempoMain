/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "tm_stm32f4_ssd1306.h"
#include "i2c_driver.h"
#include "spi_driver.h"
#include "setup_funcs.h"
#include "globals.h"
#include "lm75.h"
#include "tm_stm32f4_fonts.h"
#include "SST26VF032.h"




/* USER CODE END PV */


/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void update_lcd();
void logTemp();
void checkforusbmessages();
void Checkbatt();
void save_to_flash(uint8_t nData,uint16_t nPtr);





/* USER CODE END 0 */



uint32_t nCurrentseconds;
uint32_t nLastsampletime;
uint16_t nCurrentflashloc=0;

const uint8_t nTimediffs[] = {1,5,10,30,60};


extern I2C_TypeDef* pI2c;
extern EXTI_TypeDef* pEXTI;

GPIO_TypeDef* pGP_LED = GP_LED_GPIO_Port;
GPIO_TypeDef* pGP_BTN = USR_BTN_GPIO_Port;
GPIO_TypeDef* pGP_I2C = GPIOA;
GPIO_TypeDef* pGP_SPI = GPIOB;


extern   volatile sst_flash_handler_type sHandler;

char cWorkingStr[6];
uint8_t res;



volatile uint32_t nBtnPressTick;
volatile uint8_t bBtnPressed=0;
//user btn intterupt
void EXTI0_1_IRQHandler()
{
	uint32_t nCurrenttick = HAL_GetTick();
	uint32_t nDiff=0;

	//clear the interrupt
	HAL_NVIC_ClearPendingIRQ(EXTI0_1_IRQn);
	pEXTI->PR = 1; //clear the pr reg.
	//check if the button was pressed or released.
	if(bBtnPressed)
	{
		//this interrupt is a release, check if this is a long press or short.
		nDiff = nCurrenttick-nBtnPressTick;
		if(nDiff >= BTN_PRESS_LONG_TIME)
		{
			sSysHandler.nBtnPressed = BTN_PRESS_LONG;
		}
		else
		{
			sSysHandler.nBtnPressed = BTN_PRESS_SHORT;
		}
		bBtnPressed = 0;
		pEXTI->FTSR |= 1;
		pEXTI->RTSR &= ~((uint32_t)1);

	}
	else
	{
		//this is a falling edge, save the current press tick and wait for user to release button.
		bBtnPressed =1;
		nBtnPressTick = nCurrenttick;
		pEXTI->RTSR |= 1;
		pEXTI->FTSR &= ~((uint32_t)1);
	}
}












int main(void)
{
/* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
 // MX_ADC_Init();
  MX_I2C1_Init();
//  MX_RTC_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_USB_DEVICE_Init();
  TM_SSD1306_Init();

  res = sst_flash_init();

  sst_flash_write_block_proc();
  sst_flash_read_block_proc();
  sst_flash_write_enable();
  sst_flash_read_status();

  	  init_icd();




  sSysHandler.eMainstate = main_screen_state;
  sSysHandler.bLcdoff = 0;
  sSysHandler.bLog = 0;
  nLastsampletime = 0;
  while (1)
  {

	  sSysHandler.nCurrentTemp = lm75_get_temp_raw();
	  logTemp();
	  update_lcd();
	  nCurrentseconds = HAL_GetTick();
	  HAL_Delay(100);

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

void update_lcd()
{
	uint8_t bColor=0;
	switch(sSysHandler.eMainstate)
	{
	case init_state:
	{
		break;
	}
	case main_screen_state:
	{


		TM_SSD1306_GotoXY(40,0);
		TM_SSD1306_Puts("Tempo",&TM_Font_7x10,SSD1306_COLOR_WHITE);
		TM_SSD1306_DrawLine(0,11,128,11,SSD1306_COLOR_WHITE);

		//if logging is on,

		if(sSysHandler.bLog)
		{
			TM_SSD1306_DrawFilledCircle(90,5,4,SSD1306_COLOR_WHITE);
		}




		itoa((((sSysHandler.nCurrentTemp)&0xff00)>>8),cWorkingStr,10);
		TM_SSD1306_GotoXY(40,30);
		TM_SSD1306_Puts(cWorkingStr,&TM_Font_16x26,SSD1306_COLOR_WHITE);

		//check if long press occured
		if(sSysHandler.nBtnPressed == BTN_PRESS_LONG)
		{
			sSysHandler.eMainstate = config_state;
			TM_SSD1306_Fill(SSD1306_COLOR_BLACK);
			//button press was caught, remove press.
			sSysHandler.nBtnPressed = BTN_PRESS_NONE;
		}
		else if(sSysHandler.nBtnPressed == BTN_PRESS_SHORT)
		{
			//turn on/off the lcd.
			if(sSysHandler.bLcdoff)
			{
				SSD1306_WRITECOMMAND(0xAF);
				sSysHandler.bLcdoff =0;
			}
			else
			{
				TM_SSD1306_Fill(SSD1306_COLOR_BLACK);
				TM_SSD1306_UpdateScreen();
				SSD1306_WRITECOMMAND(0xAE);
				sSysHandler.bLcdoff =1;
			}
			//button press was caught, remove press.
			sSysHandler.nBtnPressed = BTN_PRESS_NONE;
		}



		break;
	}
	case config_state:
	{



		TM_SSD1306_GotoXY(30,0);
		TM_SSD1306_Puts("settings",&TM_Font_7x10,SSD1306_COLOR_WHITE);
		TM_SSD1306_DrawLine(0,11,128,11,SSD1306_COLOR_WHITE);

		//menu items
		TM_SSD1306_GotoXY(14,12);

		if(sSysHandler.nCurrentmenuitem == 0)
		{
			TM_SSD1306_DrawFilledRectangle(0,12,128,10,SSD1306_COLOR_WHITE);
			bColor=0;
		}
		else
		{
			TM_SSD1306_DrawFilledRectangle(0,12,128,10,SSD1306_COLOR_BLACK);
			bColor=1;
		}
		if(sSysHandler.bLog)
		{
			TM_SSD1306_Puts("stop recording",&TM_Font_7x10,bColor);
		}
		else
		{
			TM_SSD1306_Puts("start recording",&TM_Font_7x10,bColor);
		}


		TM_SSD1306_GotoXY(14,22);
		if(sSysHandler.nCurrentmenuitem == 1)
		{
			TM_SSD1306_DrawFilledRectangle(0,22,128,10,SSD1306_COLOR_WHITE);
			bColor=0;
		}
		else
		{
			TM_SSD1306_DrawFilledRectangle(0,22,128,10,SSD1306_COLOR_BLACK);
			bColor=1;
		}
		TM_SSD1306_Puts("recording setup",&TM_Font_7x10,bColor);

		TM_SSD1306_GotoXY(42,32);
		if(sSysHandler.nCurrentmenuitem == 2)
		{
			TM_SSD1306_DrawFilledRectangle(0,32,128,10,SSD1306_COLOR_WHITE);
			bColor=0;
		}
		else
		{
			TM_SSD1306_DrawFilledRectangle(0,32,128,10,SSD1306_COLOR_BLACK);
			bColor=1;
		}
		TM_SSD1306_Puts("exit",&TM_Font_7x10,bColor);

		//check button presses.
		if(sSysHandler.nBtnPressed == BTN_PRESS_LONG)
			{
				//activate the seltected item.
				switch(sSysHandler.nCurrentmenuitem)
				{
				case 0:
					//enable/disable the recording mode.
					sSysHandler.bLog ? sSysHandler.bLog--:sSysHandler.bLog++;
					break;
				case 1:
					sSysHandler.eMainstate = record_setup_state;
					sSysHandler.nCurrentmenuitem = 0;
					break;
				default:
					//exit the settings
					TM_SSD1306_Fill(SSD1306_COLOR_BLACK);
					sSysHandler.eMainstate = main_screen_state;
					break;
				}
				//button press was caught, remove press.
				sSysHandler.nBtnPressed = BTN_PRESS_NONE;
			}
		else if(sSysHandler.nBtnPressed == BTN_PRESS_SHORT)
			{
				//move to the next item
			sSysHandler.nCurrentmenuitem == 2 ? sSysHandler.nCurrentmenuitem=0:sSysHandler.nCurrentmenuitem++;
			//button press was caught, remove press.
			sSysHandler.nBtnPressed = BTN_PRESS_NONE;


			}

		break;
	}
	case record_setup_state:
	{

		TM_SSD1306_GotoXY(30,0);
				TM_SSD1306_Puts("recording setup",&TM_Font_7x10,SSD1306_COLOR_WHITE);
				TM_SSD1306_DrawLine(0,11,128,11,SSD1306_COLOR_WHITE);

				//menu items
				TM_SSD1306_GotoXY(14,12);

				if(sSysHandler.nCurrentmenuitem == 0)
				{
					TM_SSD1306_DrawFilledRectangle(0,12,128,10,SSD1306_COLOR_WHITE);
					bColor=0;
				}
				else
				{
					TM_SSD1306_DrawFilledRectangle(0,12,128,10,SSD1306_COLOR_BLACK);
					bColor=1;
				}
					TM_SSD1306_Puts("record diff ",&TM_Font_7x10,bColor);
					TM_SSD1306_Puts(itoa(nTimediffs[sSysHandler.nSecondsBetweenSamplesidx],cWorkingStr,10),&TM_Font_7x10,bColor);

				TM_SSD1306_GotoXY(14,22);
				if(sSysHandler.nCurrentmenuitem == 1)
				{
					TM_SSD1306_DrawFilledRectangle(0,22,128,10,SSD1306_COLOR_WHITE);
					bColor=0;
				}
				else
				{
					TM_SSD1306_DrawFilledRectangle(0,22,128,10,SSD1306_COLOR_BLACK);
					bColor=1;
				}
				if(sSysHandler.bLedon)
				{
					TM_SSD1306_Puts("Led on",&TM_Font_7x10,bColor);
				}
				else
				{
					TM_SSD1306_Puts("Led off",&TM_Font_7x10,bColor);
				}

				TM_SSD1306_GotoXY(42,32);
				if(sSysHandler.nCurrentmenuitem == 2)
				{
					TM_SSD1306_DrawFilledRectangle(0,32,128,10,SSD1306_COLOR_WHITE);
					bColor=0;
				}
				else
				{
					TM_SSD1306_DrawFilledRectangle(0,32,128,10,SSD1306_COLOR_BLACK);
					bColor=1;
				}
				TM_SSD1306_Puts("exit",&TM_Font_7x10,bColor);





				//check button presses.
				if(sSysHandler.nBtnPressed == BTN_PRESS_LONG)
					{
						//activate the seltected item.
						switch(sSysHandler.nCurrentmenuitem)
						{
						case 0:
							//enable/disable the recording mode.
							sSysHandler.nSecondsBetweenSamplesidx == (sizeof(nTimediffs)-1) ? sSysHandler.nSecondsBetweenSamplesidx=0:sSysHandler.nSecondsBetweenSamplesidx++;
							break;
						case 1:
							sSysHandler.bLedon ? sSysHandler.bLedon--:sSysHandler.bLedon++;

							break;
						default:
							//exit the settings
							TM_SSD1306_Fill(SSD1306_COLOR_BLACK);
							sSysHandler.eMainstate = main_screen_state;
							break;
						}
						//button press was caught, remove press.
						sSysHandler.nBtnPressed = BTN_PRESS_NONE;
					}
				else if(sSysHandler.nBtnPressed == BTN_PRESS_SHORT)
					{
						//move to the next item
					sSysHandler.nCurrentmenuitem == 2 ? sSysHandler.nCurrentmenuitem=0:sSysHandler.nCurrentmenuitem++;
					//button press was caught, remove press.
					sSysHandler.nBtnPressed = BTN_PRESS_NONE;


					}
		break;
	}

	case usb_state:
	{

		CDC_Transmit_FS((uint8_t*)IcdStatus,sizeof(usb_status_msg_type));



	}



	default:
	{

		//what?
	}

	}
	if((sSysHandler.bLcdoff) == 0)
	{
	TM_SSD1306_UpdateScreen();
	}


}






void logTemp()
{
	//check if temp logging is enabled
	if(sSysHandler.bLog)
	{
		//check if required delta between samples has passed.
		if(nLastsampletime  + nTimediffs[sSysHandler.nSecondsBetweenSamplesidx]*TICKS_PER_SEC  <= nCurrentseconds)
		{
			if(sSysHandler.bLedon)
			{
				pGP_LED->ODR |= GPIO_ODR_3;
			}

			//update last sample time.
			nLastsampletime = nCurrentseconds;
			//log current temperature;
			save_to_flash(sSysHandler.nCurrentTemp,nCurrentflashloc);
			nCurrentflashloc++;

			if(sSysHandler.bLedon)
			{
				pGP_LED->ODR &= ~GPIO_ODR_3;
			}
		}
	}
}

void save_to_flash(uint8_t nData,uint16_t nPtr)
{
//	//save the data to flash.
//	spi_flash_write_enable();
//	spi_flash_write_cmd(SPI_FLASH_USER_MEMORY_START+nPtr,2,&nData);
//	do
//	{
//	spi_flash_read_status();
//	HAL_Delay(1);
//	}while(sHandler.BUSY1); //todo: fix this.
}