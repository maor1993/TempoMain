/*
 * FlashMng.h
 *
 *  Created on: 11 Sep 2018
 *      Author: Maor
 */

#ifndef FLASHMNG_H_
#define FLASHMNG_H_

#include "stdint.h"
#include "SST26VF032.h"

//max size: 400000
//
//
//some assumptions:
//we need a max recording time for each file?
//
//
//
//form of a file header:
//
//exists_sig - 4 bytes = 0xfeedabba
//name - 7 bytes (rec0001)
//enum_rec_step - (1,5,20,30,60)
//number_of_samples -(max 65535)
//starting block (max 4000)
//
//
//size of header
//7+4+1+2+2
//
//might need to refresh header once every n smaples to allow for "shit I turned off the device"
//
//some math:
//
//size of sample = 2 bytes
//
//starting address = starting_block*256
//max filesize = 65535*2 = 131070 bytes
//
//
//not going to handle segmentation!!!!!
//currently used space =  0x200000 -(last start block * 256 * number_of_samples * 2)
//
//
//
//
//0x000000-0x002000 empty space
//0x002000-0x004000 Font Small
//0x004000-0x006000 Font Medium
//0x006000-0x008000 Font Large
//0x008000-0x100000 File Headers
//0x100000-0x300000 empty space






/*----------defines------*/

#define FLASH_FONT_SMALL_ADDR	0x002000
#define FLASH_FONT_MEDIUM_ADDR	0x004000
#define FLASH_FONT_LARGE_ADDR	0x006000
#define FLASH_FILE_HEADERS_ADDR 0x008000
#define FLASH_FILES_ADDR		0x100000

#define FLASH_MAX_HEADERS 		2

#define FLASH_MAX_BLOCK			4000
#define FLASH_MAX_FILE_SIZE		0x200000

#define FLASH_HEADER_SIG 		0xfeedabba

#define FLASH_RECORDING_SECTORS 0x200000/0x1000

/*------------------------*/


/*--------typedefs--------*/
typedef struct __attribute__ ((packed))
{
	uint32_t Signature;
	uint8_t nRecordnum;
	uint8_t	bLastRecording;
	uint8_t eTimeDiff;
	uint16_t nNumOfSamples;
	uint16_t nSectorStart;
	uint8_t rsvd[5];
}flash_file_header_type;




/*------------------------*/




/*------Prototypes-------*/


uint8_t flash_mgmt_init();
uint8_t flash_mgmt_seek();

uint8_t flash_start_recording(uint8_t nTimeDiff);
uint8_t flash_stop_recording();
uint8_t flash_save_sample(uint16_t nSample);
uint8_t flash_sync();
uint8_t flash_invalidate_headers();
uint8_t flash_erase_recordings();








/*----------------------*/





#endif /* FLASHMNG_H_ */
