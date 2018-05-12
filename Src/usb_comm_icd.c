/*
 * usb_comm_icd.c
 *
 *  Created on: 6 במאי 2018
 *      Author: maor
 */

#include "usb_comm_icd.h"
#include "usbd_cdc_if.h"

static usb_status_msg_type IcdStatus;


const usb_msg_header_type StatusHeader = {USB_MSG_PREMBLE,0,USB_REQ_MASTER_SEND,sizeof(usb_status_msg_type),USB_MSG_TYPE_STATUS};





void init_icd()
{
	IcdStatus.sHeader =StatusHeader;
}




void BuildAndSendStatusMsg()
{
	CDC_Transmit_FS((uint8_t*)IcdStatus,sizeof(usb_status_msg_type));
}


