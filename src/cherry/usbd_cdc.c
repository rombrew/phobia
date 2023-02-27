/*
 * Copyright (c) 2022, sakumisu
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "usbd_core.h"
#include "usbd_cdc.h"

static int cdc_acm_class_interface_request_handler(struct usb_setup_packet *setup, uint8_t **data, uint32_t *len)
{
    struct cdc_line_coding line_coding;
    bool dtr, rts;
    uint8_t iface_num = LO_BYTE(setup->wIndex);

    switch (setup->bRequest) {
        case CDC_REQUEST_SET_LINE_CODING:
            /*******************************************************************************/
            /* Line Coding Structure                                                       */
            /*-----------------------------------------------------------------------------*/
            /* Offset | Field       | Size | Value  | Description                          */
            /* 0      | dwDTERate   |   4  | Number |Data terminal rate, in bits per second*/
            /* 4      | bCharFormat |   1  | Number | Stop bits                            */
            /*                                        0 - 1 Stop bit                       */
            /*                                        1 - 1.5 Stop bits                    */
            /*                                        2 - 2 Stop bits                      */
            /* 5      | bParityType |  1   | Number | Parity                               */
            /*                                        0 - None                             */
            /*                                        1 - Odd                              */
            /*                                        2 - Even                             */
            /*                                        3 - Mark                             */
            /*                                        4 - Space                            */
            /* 6      | bDataBits  |   1   | Number Data bits (5, 6, 7, 8 or 16).          */
            /*******************************************************************************/
            memcpy(&line_coding, *data, setup->wLength);
            usbd_cdc_acm_set_line_coding(iface_num, &line_coding);
            break;

	case CDC_REQUEST_SET_CONTROL_LINE_STATE:
	    dtr = (setup->wValue & 0x0001);
	    rts = (setup->wValue & 0x0002);
	    usbd_cdc_acm_set_dtr(iface_num, dtr);
	    usbd_cdc_acm_set_rts(iface_num, rts);
	    break;

        case CDC_REQUEST_GET_LINE_CODING:
            usbd_cdc_acm_get_line_coding(iface_num, &line_coding);
            memcpy(*data, &line_coding, 7);
            *len = 7;
            break;

        default:
            return -1;
    }

    return 0;
}

static void cdc_notify_handler(uint8_t event, void *arg)
{
    switch (event) {
        case USBD_EVENT_RESET:
            break;
        default:
            break;
    }
}

struct usbd_interface *usbd_cdc_acm_init_iface(struct usbd_interface *iface)
{
    iface->class_interface_handler = cdc_acm_class_interface_request_handler;
    iface->class_endpoint_handler = NULL;
    iface->vendor_handler = NULL;
    iface->notify_handler = cdc_notify_handler;

    return iface;
}

__WEAK void usbd_cdc_acm_set_line_coding(uint8_t iface, struct cdc_line_coding *line_coding)
{
}

__WEAK void usbd_cdc_acm_get_line_coding(uint8_t iface, struct cdc_line_coding *line_coding)
{
    line_coding->dwDTERate = 57600;
    line_coding->bDataBits = 8;
    line_coding->bParityType = 2;
    line_coding->bCharFormat = 0;
}

__WEAK void usbd_cdc_acm_set_dtr(uint8_t iface, bool dtr)
{
}

__WEAK void usbd_cdc_acm_set_rts(uint8_t iface, bool rts)
{
}

