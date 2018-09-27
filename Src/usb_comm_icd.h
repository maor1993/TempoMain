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
#define USB_MSG_PREMBLE 0x9a5bc0dd
#define USB_MAX_MSG_SIZE 64
#define USB_HEADER_SIZE 8


#define USB_REQ_MASTER_SEND 0
#define USB_REQ_MASTER_RECEIVE 1

#define USB_MSG_TYPE_STATUS 				0x01
#define USB_MSG_TYPE_REPORT_RECORDING 		0x02
#define USB_MSG_TYPE_GET_RECORDING			0x03
#define USB_MSG_TYPE_ENTER_UPDATE			0x04
#define USB_MSG_TYPE_START_COMM				0x05
#define USB_MSG_TYPE_REQUST_SAMPLES			0x06



typedef struct __attribute__((packed)){
	uint32_t nPremble;
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
	flash_file_header_type sRecHeader;
	uint8_t nTotalRecs;


}usb_get_recording_type;


typedef struct{
	usb_msg_header_type sHeader;
	uint32_t nFlashOffset;
}usb_req_samples_type;


typedef struct{
	usb_msg_header_type sHeader;
	uint16_t nSamples[16];
}usb_requested_samples_type;



/*---prototypes---*/


void BuildAndSendStatusMsg();
void ParseIcdMsg();


/*----------------*/




#endif /* USB_COMM_ICD_H_ */
