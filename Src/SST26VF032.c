/*
 * SST26VF032.c
 *
 *  Created on: 2 ����� 2018
 *      Author: maor
 */

#include "SST26VF032.h"
#include "spi_driver.h"
#include <stdint.h>
#include "stm32f0xx_hal.h"
#include <string.h>


 static uint8_t nWorking_buffer[FLASH_BUF_MAX_SIZE];
 static volatile sst_flash_handler_type sHandler;

 static void sst_flash_read_id()
 {
 	nWorking_buffer[0] = SST_FLASH_CMD_JEDEC_ID;
 	spi_write_read(nWorking_buffer,1,nWorking_buffer+1,4);
 }


 static uint8_t sst_flash_solve_mem_overlap(uint32_t nAddr,uint8_t nSize)
 {
	 //check if this write will overlap to the next subsector.
	 if(((nAddr&0x000000ff)+(uint32_t)nSize)&0xf00)
	 {
		return ((nAddr&0x000000ff)+nSize)&0xff;
	 }
	 else
	 {
		 return 0;
	 }

 }


 void sst_flash_write_enable()
 {
 	spi_write_single(SST_FLASH_CMD_WREN);
 }
 void sst_flash_write_disable()
 {
 	spi_write_single(SST_FLASH_CMD_WRDI);
 }
 uint8_t sst_flash_read_cmd(uint32_t nAddr,uint8_t nBytes,uint8_t* pData)
 {
		//build the tx msg
		*((uint32_t*)nWorking_buffer) = ((nAddr>>24)&0xff) | // move byte 3 to byte 0
	            ((nAddr<<8)&0xff0000) | // move byte 1 to byte 2
	            ((nAddr>>8)&0xff00) | // move byte 2 to byte 1
	            ((nAddr<<24)&0xff000000); // byte 0 to byte 3;
		nWorking_buffer[0] = SST_FLASH_CMD_READ;

		return spi_write_read(nWorking_buffer,4,pData,nBytes);
 }


 uint8_t sst_flash_write_cmd(uint32_t nAddr,uint8_t nbytes,uint8_t* pData)
 {
		//build the tx msg
		*((uint32_t*)nWorking_buffer) = ((nAddr>>24)&0xff) | // move byte 3 to byte 0
	            ((nAddr<<8)&0xff0000) | // move byte 1 to byte 2
	            ((nAddr>>8)&0xff00) | // move byte 2 to byte 1
	            ((nAddr<<24)&0xff000000); // byte 0 to byte 3;
		nWorking_buffer[0] = SST_FLASH_CMD_PP;

		memcpy(nWorking_buffer+4,pData,nbytes);

		return spi_write_blocking(nWorking_buffer,nbytes+4);

 }

uint8_t sst_flash_write_cmd_blocking(uint32_t nAddr,uint8_t nbytes,uint8_t* pData,uint8_t nRefreshTicks)
{
	uint8_t res=0;
	uint8_t nCurrentTicks=nRefreshTicks;

	uint8_t nExtrabytes;

	sst_flash_write_enable();
	//verify that chip is alive.
	sst_flash_read_status();
	if(!sHandler.WEL)
	{
		return 1;
	}

	//build the tx msg
	*((uint32_t*)nWorking_buffer) = ((nAddr>>24)&0xff) | // move byte 3 to byte 0
			((nAddr<<8)&0xff0000) | // move byte 1 to byte 2
			((nAddr>>8)&0xff00) | // move byte 2 to byte 1
			((nAddr<<24)&0xff000000); // byte 0 to byte 3;
	nWorking_buffer[0] = SST_FLASH_CMD_PP;

	//check if there are bytes that must be written to a different subsector.
	nExtrabytes = sst_flash_solve_mem_overlap(nAddr,nbytes);

	//copy to working buffer the bytes for this section
	memcpy(nWorking_buffer+4,pData,nbytes-nExtrabytes);
	spi_write_blocking(nWorking_buffer,nbytes+4-nExtrabytes);

		do{
			sst_flash_read_status();
			HAL_Delay(10);
		}while((nCurrentTicks--)&&(sHandler.BUSY1));
		//verify that all is good
		if(sHandler.BUSY1)
		{
			return 1; //there is an issue...
		}
	if(nExtrabytes)
	{
		sst_flash_write_enable();
		sst_flash_read_status();
		//write the rest of the bytes.
		nAddr += nbytes-nExtrabytes;
		nCurrentTicks = nRefreshTicks;

			//write the new address.
			*((uint32_t*)nWorking_buffer) = ((nAddr>>24)&0xff) | // move byte 3 to byte 0
						((nAddr<<8)&0xff0000) | // move byte 1 to byte 2
						((nAddr>>8)&0xff00) | // move byte 2 to byte 1
						((nAddr<<24)&0xff000000); // byte 0 to byte 3;
			nWorking_buffer[0] = SST_FLASH_CMD_PP;

		//copy the spare bytes to buffer.
		memcpy(nWorking_buffer+4,pData+nbytes-nExtrabytes,nExtrabytes);
		spi_write_blocking(nWorking_buffer,4+nExtrabytes);


		do{
			sst_flash_read_status();
			HAL_Delay(10);
		}while((nCurrentTicks--)&&(sHandler.BUSY1));
		//verify that all is good
		if(sHandler.BUSY1)
		{
			return 1; //there is an issue...
		}
	}

	return res;

}
uint8_t sst_flash_init()
{

	 uint8_t res;



	 //reset the flash.
	 spi_write_single(SST_FLASH_CMD_RSTEN);
	 HAL_Delay(10);
	 spi_write_single(SST_FLASH_CMD_RST);

	 HAL_Delay(100);

	 //for now, all we need is to check that we can communicate with the flash.
	 sst_flash_read_id();
	 res =nWorking_buffer[1];

	 HAL_Delay(10);

	// verify the block is protected
	 sst_flash_read_status();



	 return 0;


}

void sst_flash_read_status()
{
	nWorking_buffer[0] = SST_FLASH_CMD_RDSR;

	spi_write_read(nWorking_buffer,1,nWorking_buffer+1,1);
	sHandler.nStatus_byte = nWorking_buffer[1];
}


void sst_flash_write_status(uint8_t nStatusbyte)
{
	nWorking_buffer[0] = SST_FLASH_CMD_WRSR;
	nWorking_buffer[1] = nStatusbyte;
	spi_write_blocking(nWorking_buffer,2);

}
void sst_flash_read_block_proc()
{
	nWorking_buffer[0] = SST_FLASH_CMD_RBPR;
	spi_write_read(nWorking_buffer,1,nWorking_buffer+1,18);

}
void sst_flash_write_block_proc()
{
	uint8_t i;
	nWorking_buffer[0] = SST_FLASH_CMD_WBPR;
	for(i=0;i<18;i++)
	{
		nWorking_buffer[1+i] = 0;
	}
	spi_write_blocking(nWorking_buffer,19);

}

void sst_flash_erase_chip()
{
	nWorking_buffer[0] = SST_FLASH_CMD_CE;
	spi_write_single(nWorking_buffer[0]);



}

void sst_flash_erase_sector(uint32_t nAddr)
{


	sst_flash_write_enable();



	*((uint32_t*)nWorking_buffer) = ((nAddr>>24)&0xff) | // move byte 3 to byte 0
			((nAddr<<8)&0xff0000) | // move byte 1 to byte 2
			((nAddr>>8)&0xff00) | // move byte 2 to byte 1
			((nAddr<<24)&0xff000000); // byte 0 to byte 3;
	nWorking_buffer[0] = SST_FLASH_CMD_SE;

	spi_write_blocking(nWorking_buffer,4);

}

void sst_flash_erase_sector_blocking(uint32_t nAddr)
{


	sst_flash_write_enable();



	*((uint32_t*)nWorking_buffer) = ((nAddr>>24)&0xff) | // move byte 3 to byte 0
			((nAddr<<8)&0xff0000) | // move byte 1 to byte 2
			((nAddr>>8)&0xff00) | // move byte 2 to byte 1
			((nAddr<<24)&0xff000000); // byte 0 to byte 3;
	nWorking_buffer[0] = SST_FLASH_CMD_SE;

	spi_write_blocking(nWorking_buffer,4);


	do{
		sst_flash_read_status();
		HAL_Delay(10);
	}while(sHandler.BUSY1);

}
