/*
 * i2c_driver.c
 *
 *  Created on: 24 בפבר׳ 2017
 *      Author: maor
 */

#include "i2c_driver.h"
#include <stdint.h>
#include "stm32f042x6.h"
#include "stm32f0xx_hal.h"

//i2c_handler_type sI2c_1;
volatile I2C_TypeDef* pI2c = I2C1;

static volatile uint8_t WaitForFlag(volatile uint32_t* pRegloc,uint32_t nFlag,uint8_t bFlagStateRequired,uint32_t nTimeoutVal,uint32_t nTickstart)
{

	if(bFlagStateRequired)
	{
		while(((pI2c->ISR)&nFlag) != nFlag)
			{
 				if(nTimeoutVal != HAL_MAX_DELAY)
				    {
				      if((nTimeoutVal == 0U)||((HAL_GetTick() - nTickstart ) > nTimeoutVal))
				      {

				        return 1;
				      }
				    }
			}
	}
	else
	{
		while(((pI2c->ISR)&nFlag) != 0)
					{
					if(nTimeoutVal != HAL_MAX_DELAY)
						{
						  if((nTimeoutVal == 0U)||((HAL_GetTick() - nTickstart ) > nTimeoutVal))
						  {

							return 1;
						  }
						}
					}

	}
	return 0;
}



uint8_t i2c_write_blocking(uint8_t* pData,uint8_t nSize,uint32_t nTimeout,uint8_t nSlave_addr)
{
	uint8_t i=nSize;
	uint32_t tickstart = 0;

	tickstart = HAL_GetTick();

	//ensure that i2c controller is not busy.
	if(WaitForFlag(&(pI2c->ISR),I2C_ISR_BUSY_Msk,0,I2C_TIMEOUT_BUSY,tickstart) != 0)
	{
		//while(1);
		return 1;
	}

	//clear nbytes and set it to new value.
	pI2c->CR2 &= (~I2C_CR2_NBYTES_Msk);
	pI2c->CR2 |= nSize <<  I2C_CR2_NBYTES_Pos;

	//configure the i2c device to Repeated Start.
	pI2c->CR2 |= I2C_CR2_AUTOEND_Msk;
	//pI2c->CR2 &= ~I2C_CR2_AUTOEND_Msk;

	//set the target address.
	pI2c->CR2 &= (~I2C_CR2_SADD_Msk);
	pI2c->CR2 |= (nSlave_addr&I2C_CR2_SADD_Msk)<<I2C_CR2_SADD_Pos;

	//set the controller to write mode.
	pI2c->CR2 &= ~I2C_CR2_RD_WRN_Msk;

	//wait for the controller to send start.
	pI2c->CR2 |=  I2C_CR2_START_Msk;




	while (i > 0)
	{

		//wait for current transfer to complete.
		if (WaitForFlag(&(pI2c->ISR), I2C_ISR_TXIS_Msk, 1,nTimeout, tickstart) != 0)
		{
			//while(1);
			return 1;
		}

		pI2c->TXDR = *(pData + (nSize - i));




		i--;
	}
	if(WaitForFlag(&(pI2c->ISR),I2C_ISR_STOPF_Msk,1,nTimeout,tickstart) != 0)
	{
		//while(1);
		return 1;
	}

	//generate stop condition
	//pI2c->CR2 |= I2C_CR2_STOP_Msk;
	pI2c->ISR &= ~I2C_ISR_STOPF_Msk;

	return 0;
}

uint8_t i2c_read_blocking(uint8_t* pData,uint8_t nSize,uint32_t nTimeout,uint8_t nSlave_addr)
{
	uint8_t i=0;
	uint32_t tickstart=0;

	tickstart = HAL_GetTick();

	//ensure that i2c controller is not busy.
	if(WaitForFlag(&(pI2c->ISR),I2C_ISR_BUSY_Msk,0,I2C_TIMEOUT_BUSY,tickstart))
	{
		return 1;
	}

	//clear nbytes and set it to new value.
	pI2c->CR2 &= (~I2C_CR2_NBYTES_Msk);
	pI2c->CR2 |= nSize <<  I2C_CR2_NBYTES_Pos;

	//configure the i2c device to Repeated Start.
	pI2c->CR2 |= I2C_CR2_AUTOEND_Msk;
	//pI2c->CR2 &= ~I2C_CR2_AUTOEND_Msk;

	//set the target address.
	pI2c->CR2 &= (~I2C_CR2_SADD_Msk);
	pI2c->CR2 |= (nSlave_addr&I2C_CR2_SADD_Msk)<<I2C_CR2_SADD_Pos;

	//set the i2c controller to read mode.
	pI2c->CR2 |= I2C_CR2_RD_WRN_Msk;

	//wait for the controller to send start.
	pI2c->CR2 |=  I2C_CR2_START_Msk;



	i=nSize;
		while (i)
		{
			//wait for current transfer to complete.
			if (WaitForFlag(&(pI2c->ISR), I2C_ISR_RXNE_Msk, 1,nTimeout, tickstart)) {
				return 1;
			}
			*(pData + (nSize - i)) = pI2c->RXDR;
			i--;
		}


		if(WaitForFlag(&(pI2c->ISR),I2C_ISR_NACKF_Msk,0,I2C_TIMEOUT_BUSY,tickstart))
		{
			return 1;
		}


	//generate stop condition
	//pI2c->CR2 |= I2C_CR2_STOP_Msk;

	return 0;
}
