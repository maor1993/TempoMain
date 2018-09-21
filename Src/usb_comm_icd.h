/*
 * usb_comm_icd.h
 *
 *  Created on: 28 срхїз 2017
 *      Author: maor
 */

#ifndef USB_COMM_ICD_H_
#define USB_COMM_ICD_H_
#include <stdint.h>
#include "FlashMng.h"




//---------defines----------//
#define USB_MSG_PREMBLE 0xa5a5
#define USB_MAX_MSG_SIZE 64


#define USB_REQ_MASTER_SEND 0
#define USB_REQ_MASTER_RECEIVE 1

#define USB_MSG_TYPE_STATUS 				0x01
#define USB_MSG_TYPE_REPORT_RECORDING 		0x02
#define USB_MSG_TYPE_GET_RECORDING			0x03
#define USB_MSG_TYPE_ENTER_UPDATE			0x04



typedef struct __attribute__((packed)){
	uint16_t nPremble;
	uint8_t nSequence;
	uint8_t nReq;
	uint8_t nMsgtype;
	uint8_t nMsglen;


}usb_msg_header_type;


typedef struct{
	usb_msg_header_type sHeader;
	uint8_t nMsgdata[USB_MAX_MSG_SIZE];
}usb_msg_template_type;



typedef struct{
	usb_msg_header_type sHeader;
	uint16_t nBattPercent;
	uint16_t nTemp;


}usb_status_msg_type;

typedef struct{
	usb_msg_header_type sHeader;
	uint8_t nTotalRecs;
	flash_file_header_type sRecHeader;


}usb_get_recording_type;




/*prototypes*/


void BuildAndSendStatusMsg();
void ParseIcdMsg();


/*----------*/




#endif /* USB_COMM_ICD_H_ */
