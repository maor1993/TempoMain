/*
 * SST26VF032.c
 *
 *  Created on: 2 בפבר׳ 2018
 *      Author: maor
 */

#include "SST26VF032.h"
#include "spi_driver.h"
#include <stdint.h>


 static volatile uint8_t nWorking_buffer[FLASH_BUF_MAX_SIZE];
  volatile sst_flash_handler_type sHandler;

 static void sst_flash_read_id()
 {
 	nWorking_buffer[0] = SST_FLASH_CMD_JEDEC_ID;
 	spi_write_read(nWorking_buffer,1,nWorking_buffer+1,4);
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
