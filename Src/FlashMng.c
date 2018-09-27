/*
 * FlashMng.c
 *
 *  Created on: 11 Sep 2018
 *      Author: Maor
 */


#include <stdint.h>
#include "FlashMng.h"
#include "SST26VF032.h"





volatile flash_file_header_type sCurrentHeader=
		{
			.nNumOfSamples=0,
			.nSectorStart=0,
		};
flash_file_header_type sNextHeader;

uint8_t flash_mgmt_init()
{
	uint8_t nCurrentRecNum=0;
	//step 1: get the first header
	sst_flash_read_cmd(FLASH_FILE_HEADERS_ADDR,sizeof(flash_file_header_type),(uint8_t*)&(sNextHeader.Signature));

	//step 2: verify that sig exists.
	while(nCurrentRecNum < FLASH_MAX_HEADERS)
	{
		if(sNextHeader.Signature != FLASH_HEADER_SIG)
		{
			//this is the fresh header, build it up...
			sCurrentHeader.Signature = FLASH_HEADER_SIG;
			sCurrentHeader.bLastRecording = 1;
			sCurrentHeader.nRecordnum = nCurrentRecNum;


			sCurrentHeader.nSectorStart  = sCurrentHeader.nSectorStart +((sCurrentHeader.nNumOfSamples)>>7);

			//calc new blockstart.
			if(sCurrentHeader.nNumOfSamples%4096)
			{
				//add another block
				sCurrentHeader.nSectorStart++;
			}

			sCurrentHeader.nNumOfSamples = 0;

			return 0;
		}
		else
		{
			//signature is okay, find the next header.
			sCurrentHeader=sNextHeader;
			nCurrentRecNum++;
			sst_flash_read_cmd(FLASH_FILE_HEADERS_ADDR+(nCurrentRecNum*0x1000),sizeof(flash_file_header_type),(uint8_t*)&(sNextHeader.Signature));
		}
	}

	//if we're here, we're full...
	return 1;


}

uint8_t flash_start_recording(uint8_t nTimeDiff)
{


		//step 1: update previous header
		if(sCurrentHeader.nRecordnum != 0)
		{
//			sst_flash_read_cmd(FLASH_FILE_HEADERS_ADDR+(sCurrentHeader.nRecordnum-1)*0x1000,sizeof(flash_file_header_type),(uint8_t*)&(sNextHeader.Signature));
//			sNextHeader.bLastRecording = 0;
//
//			sst_flash_erase_sector_blocking(FLASH_FILE_HEADERS_ADDR+(sCurrentHeader.nRecordnum-1));
//			sst_flash_write_cmd_blocking(FLASH_FILE_HEADERS_ADDR+(sCurrentHeader.nRecordnum-1)*0x1000,sizeof(flash_file_header_type),(uint8_t*)&(sNextHeader.Signature),10);
		}


		//step 2: set the time diff in accordance
		sCurrentHeader.eTimeDiff = nTimeDiff;

		//step 3: write the initial header.
		sst_flash_write_cmd_blocking(FLASH_FILE_HEADERS_ADDR+(sCurrentHeader.nRecordnum)*0x1000,sizeof(flash_file_header_type),(uint8_t*)&(sCurrentHeader.Signature),10);
		//we are now allowed to record.
	return 0;
}



uint8_t flash_stop_recording()
{
	//check if we can record any more.
	if(sCurrentHeader.nRecordnum + 1 > FLASH_MAX_HEADERS)
	{
		return 1;
	}
	else
	{

		return 0;
	}
}

uint8_t flash_save_sample(uint16_t nSample)
{

	uint32_t nAddr;

	nAddr = FLASH_FILES_ADDR + (sCurrentHeader.nSectorStart * 0x1000) + (sCurrentHeader.nNumOfSamples * 2);


	//write sample to flash.
	sst_flash_write_cmd_blocking(nAddr,2,(uint8_t*)&nSample,10);


	//update the amount of samples in header.
	sCurrentHeader.nNumOfSamples++;


	//update header.
	sst_flash_erase_sector_blocking(FLASH_FILE_HEADERS_ADDR+(sCurrentHeader.nRecordnum)*0x1000);
	sst_flash_write_cmd_blocking(FLASH_FILE_HEADERS_ADDR+(sCurrentHeader.nRecordnum)*0x1000,sizeof(flash_file_header_type),(uint8_t*)&(sCurrentHeader.Signature),10);
	return 0;
}

uint8_t flash_save_multi_sample(uint16_t* pSamples,uint8_t nSamples)
{



	return 0;
}

uint8_t flash_sync()
{




	return 0;
}

uint8_t flash_invalidate_headers()
{
	//delete all headers.
	uint8_t i = FLASH_MAX_HEADERS;


	while(i--)
	{
		sst_flash_erase_sector_blocking(FLASH_FILE_HEADERS_ADDR+(i*0x1000));
	}

	return 0;
}

uint8_t flash_erase_recordings()
{
	uint16_t i = FLASH_RECORDING_SECTORS;



	while(i--)
	{
		sst_flash_erase_sector_blocking(FLASH_FILES_ADDR+(i*0x1000));
	}

	return 0;
}

