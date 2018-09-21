/*
 * usb_comm_icd.c
 *
 *  Created on: 6 במאי 2018
 *      Author: maor
 */

#include "usb_comm_icd.h"
#include "usbd_cdc_if.h"
#include "FlashMng.h"


/*----------cosntant headers---------*/

const usb_msg_header_type StatusHeader = {USB_MSG_PREMBLE,0,USB_REQ_MASTER_SEND,USB_MSG_TYPE_STATUS,sizeof(usb_status_msg_type)-sizeof(usb_msg_header_type)};
const usb_msg_header_type GetRecHeader =
{
		.nPremble = USB_MSG_PREMBLE,
		.nReq = USB_REQ_MASTER_RECEIVE,
		.nMsglen = sizeof(usb_get_recording_type) - sizeof(usb_msg_header_type),
		.nMsgtype = USB_MSG_TYPE_REPORT_RECORDING,

};

/*----------------------------------*/




static usb_msg_template_type sRxIcdMsg;


static usb_status_msg_type IcdStatus;
usb_get_recording_type IcdGetRec;




extern flash_file_header_type sCurrentHeader;
extern flash_file_header_type sNextHeader;
extern uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];

extern volatile system_handler_type sSysHandler;


void (*SysMemBootJump)(void);


void init_icd()
{
	IcdStatus.sHeader =StatusHeader;
	IcdGetRec.sHeader =GetRecHeader;
}




void BuildAndSendStatusMsg()
{
	IcdStatus.nTemp = sSysHandler.nCurrentTemp;
		CDC_Transmit_FS((uint8_t*)&(IcdStatus.sHeader),sizeof(usb_status_msg_type));
}

void process_rx_msg()
{
	//todo: is it safe to assume that message is always aligned?
	//step 1: verify alignment.

	//todo: might want to change premble to something more exotic.
	if((UserRxBufferFS[0] == 0xa5) && (UserRxBufferFS[1] == 0xa5))
	{
		//buffer is aligned, get size from buffer and copy to template.
		uint8_t nLen = UserRxBufferFS[5];

		memcpy((uint8_t*)&(sRxIcdMsg.sHeader.nPremble),&UserRxBufferFS[0],sizeof(usb_msg_header_type)+nLen);
		ParseIcdMsg();
	}
}


void ParseIcdMsg()
{
	switch(sRxIcdMsg.sHeader.nReq)
	{
		case USB_REQ_MASTER_SEND:
		{
			switch(sRxIcdMsg.sHeader.nMsgtype)
			{
				case USB_MSG_TYPE_ENTER_UPDATE:
				{

				  //erase first sector
				  FLASH->KEYR = 0x45670123; /* (3) */
				  FLASH->KEYR = 0xCDEF89AB;
				  FLASH_PageErase(0x08000000);
				  while (1);
				break;
				}


			}


		break;
		}
		case USB_REQ_MASTER_RECEIVE:
		{

			switch(sRxIcdMsg.sHeader.nMsgtype)
			{
				case USB_MSG_TYPE_REPORT_RECORDING:
				{
				//get header requested from pc.
				IcdGetRec.nTotalRecs = sCurrentHeader.nRecordnum + 1;
				sst_flash_read_cmd(FLASH_FILE_HEADERS_ADDR+(sRxIcdMsg.nMsgdata[0]*0x1000),sizeof(flash_file_header_type),(uint8_t*)&(IcdGetRec.sRecHeader.Signature));

				CDC_Transmit_FS((uint8_t*)&(IcdGetRec.sHeader),sizeof(usb_get_recording_type));
				break;
				}





			}




		break;
		}

		default:
		{



		break;
		}


	}



}


