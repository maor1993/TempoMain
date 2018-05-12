/*
 * usb_comm_icd.h
 *
 *  Created on: 28 срхїз 2017
 *      Author: maor
 */

#ifndef USB_COMM_ICD_H_
#define USB_COMM_ICD_H_
#include <stdint.h>





//---------defines----------//
#define USB_MSG_PREMBLE 0xa5a5
#define USB_MAX_MSG_SIZE 64


#define USB_REQ_MASTER_SEND 0
#define USB_REQ_MASTER_RECEIVE 1

#define USB_MSG_TYPE_STATUS 				0x01
#define USB_MSG_TYPE_REPORT_RECORDING 		0x02
#define USB_MSG_TYPE_GET_RECORDING			0x03




typedef struct __attribute__((packed)){
	uint16_t nPremble;
	uint8_t nSequence;
	uint8_t nReq;
	uint8_t nMsglen;
	uint8_t nMsgtype;


}usb_msg_header_type;


typedef struct{
	usb_msg_header_type sHeader;
	uint8_t nMsgdata[USB_MAX_MSG_SIZE - sizeof(usb_msg_header_type)];
}usb_msg_template_type;



typedef struct{
	usb_msg_header_type sHeader;
	uint16_t nBattPercent;


}usb_status_msg_type;






/*prototypes*/


void BuildAndSendStatusMsg();



/*----------*/




#endif /* USB_COMM_ICD_H_ */
