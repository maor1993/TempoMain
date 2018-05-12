/*
 * SST26VF032.h
 *
 *  Created on: 2 áôáø× 2018
 *      Author: maor
 */

#ifndef SST26VF032_H_
#define SST26VF032_H_

#include <stdint.h>
//note that currently only supports normal spi i/f, not QUAD.


/*--------------defines----------*/
#define FLASH_BUF_MAX_SIZE 256 + 5 //256bytes plus 5 for largest header.


/*-------------commands----------*/
/*-----------configuration--------*/
#define SST_FLASH_CMD_NOP 0x00
#define SST_FLASH_CMD_RSTEN 0x66
#define SST_FLASH_CMD_RST	0x99
#define SST_FLASH_CMD_EQIO	0x38
#define SST_FLASH_CMD_RSTQIO 0xff
#define SST_FLASH_CMD_RDSR	0x05
#define SST_FLASH_CMD_WRSR	0x01
#define SST_FLASH_CMD_RDCR  0x35
/*----------read commands------*/
#define SST_FLASH_CMD_READ	0x03
#define SST_FLASH_CMD_HSREAD 0x0b
/*-----------identification-----*/
#define SST_FLASH_CMD_JEDEC_ID 	0x9f
#define SST_FLASH_CMD_SFDP		0x5a
/*-----------writes-------------*/
#define SST_FLASH_CMD_WREN		0x06
#define SST_FLASH_CMD_WRDI		0x04
#define SST_FLASH_CMD_SE		0x20
#define SST_FLASH_CMD_BE		0xd8
#define SST_FLASH_CMD_CE		0xc7
#define SST_FLASH_CMD_PP		0x02
#define SST_FLASH_CMD_WRSU		0xb0
#define SST_FLASH_CMD_WRRE		0x30
/*----------protection------------*/
#define SST_FLASH_CMD_RBPR		0x72
#define SST_FLASH_CMD_WBPR		0x42
#define SST_FLASH_CMD_LBPR		0x8d
#define SST_FLASH_CMD_NVWLDR 	0xe8
#define SST_FLASH_CMD_ULBPR		0x98
#define SST_FLASH_CMD_RSID		0x88
#define SST_FLASH_CMD_PSID		0xa5
#define SST_FLASH_CMD_LSID		0x85



/*------------------------------------*/

typedef struct {


	union{
		struct{
		unsigned BUSY1 : 	1;
		unsigned WEL	:	1;
		unsigned WSE 	: 	1;
		unsigned WSP	: 	1;
		unsigned WPLD	: 	1;
		unsigned SEC	: 	1;
		unsigned rsvd	: 	1;
		unsigned BUSY2	: 	1;
		};
		uint8_t nStatus_byte;

	};

	union{
		struct{
		unsigned rsvd1 	: 1;
		unsigned IOC	: 1;
		unsigned rsvd2 	: 2;
		unsigned BPNV	: 1;
		unsigned rsvd3 	: 3;
		unsigned WPEN	: 1;
		};
		uint8_t nCfg_byte;
	};

}sst_flash_handler_type;







/*--------------------------------*/


/*----------------------------------*/


/**.
 *
 */

/*----------------------basic commands----------*/
/*@brief
 *@enables writing to the flash.
 * @this command must be performed before every active work with the flash.
 */
void sst_flash_write_enable();

/*@brief
 *@disables writing to the flash.
 */
void sst_flash_write_disable();



uint8_t sst_flash_read_cmd(uint32_t nAddr,uint8_t nBytes,uint8_t* pData);


uint8_t sst_flash_write_cmd(uint32_t nAddr,uint8_t nbytes,uint8_t* pData);

void sst_flash_read_status();
void sst_flash_write_status(uint8_t nStatusbyte);

void sst_flash_wakeup();


uint8_t sst_flash_init();
/*-----------------------------`-----------------*/



#endif /* SST26VF032_H_ */
