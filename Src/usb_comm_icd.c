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

const usb_msg_header_type StatusHeader =
{
		.nPremble =USB_MSG_PREMBLE,
		.nReq =USB_REQ_MASTER_SEND,
		.nMsgtype =USB_MSG_TYPE_STATUS,
		.nMsglen= sizeof(usb_status_msg_type)-USB_HEADER_SIZE
};
const usb_msg_header_type GetRecHeader =
{
		.nPremble = USB_MSG_PREMBLE,
		.nReq = USB_REQ_MASTER_RECEIVE,
		.nMsglen = sizeof(usb_get_recording_type) - USB_HEADER_SIZE,
		.nMsgtype = USB_MSG_TYPE_REPORT_RECORDING,

};

const usb_msg_header_type GetSamplesHeader=
{
		.nPremble = USB_MSG_PREMBLE,
		.nReq = USB_REQ_MASTER_RECEIVE,
		.nMsgtype = USB_MSG_TYPE_REQUST_SAMPLES,
		.nMsglen = sizeof(usb_requested_samples_type) - USB_HEADER_SIZE,
};

/*----------------------------------*/




static usb_msg_template_type sRxIcdMsg;


static usb_status_msg_type IcdStatus;
static usb_get_recording_type IcdGetRec;
static usb_requested_samples_type IcdGetSam;



static uint8_t nTxSeq=0;

extern flash_file_header_type sCurrentHeader;
extern flash_file_header_type sNextHeader;
extern uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];

extern volatile system_handler_type sSysHandler;


void init_icd()
{
	IcdStatus.sHeader =StatusHeader;
	IcdGetRec.sHeader =GetRecHeader;
	IcdGetSam.sHeader =GetSamplesHeader;
}




void BuildAndSendStatusMsg()
{
	IcdStatus.nTemp = sSysHandler.nCurrentTemp;
	IcdStatus.sHeader.nSequence = nTxSeq++;
	CDC_Transmit_FS((uint8_t*)&(IcdStatus.sHeader),sizeof(usb_status_msg_type));
}

void process_rx_msg()
{
	//step 1: verify alignment.

	if((UserRxBufferFS[3] == 0x9a) && (UserRxBufferFS[2] == 0x5b) && (UserRxBufferFS[1]== 0xc0) && (UserRxBufferFS[0]==0xdd))
	{
		//buffer is aligned, get size from buffer and copy to template.
		uint8_t nLen = UserRxBufferFS[7];

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
				  FLASH->KEYR = 0x45670123;
				  FLASH->KEYR = 0xCDEF89AB;
				  FLASH_PageErase(0x08000000);
				  while (1);
				break;
				}
				case USB_MSG_TYPE_START_COMM:
				{

				sSysHandler.eMainstate = usb_connected_state;
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

				IcdGetRec.sHeader.nSequence = nTxSeq++;
				CDC_Transmit_FS((uint8_t*)&(IcdGetRec.sHeader),sizeof(usb_get_recording_type));
				break;
				}


				case USB_MSG_TYPE_REQUST_SAMPLES:
				{
				//cast to message
				usb_req_samples_type* msg = (usb_req_samples_type*)(&sRxIcdMsg);


				//copy 16 samples to message.

				//note that lcd comm needs to be stopped since this occurs in an interrupt.
				sst_flash_read_cmd(msg->nFlashOffset,32,&(IcdGetSam.nSamples[0]));

				IcdGetSam.nRequestedSamples = msg->nFlashOffset;
				//send to device.
				IcdGetSam.sHeader.nSequence = nTxSeq++;

				CDC_Transmit_FS((uint8_t*)&(IcdGetSam.sHeader),sizeof(usb_requested_samples_type));
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


