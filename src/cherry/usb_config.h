/*
 * Copyright (c) 2022, sakumisu
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef CHERRYUSB_CONFIG_H
#define CHERRYUSB_CONFIG_H

extern void log_TRACE(const char *fmt, ...);

/* ================ USB common Configuration ================ */

#ifndef CONFIG_USB_DBG_LEVEL
#define CONFIG_USB_DBG_LEVEL		-1
#endif

#ifndef CONFIG_USB_PRINTF
#define CONFIG_USB_PRINTF		log_TRACE
#endif

#ifndef CONFIG_USB_ALIGN_SIZE
#define CONFIG_USB_ALIGN_SIZE		4
#endif

#define USB_NOCACHE_RAM_SECTION		/* TODO */
#define USB_MEM_ALIGN			__attribute__ ((aligned(CONFIG_USB_ALIGN_SIZE)))

/* ================= USB Device Stack Configuration ================ */

/* Ep0 max transfer buffer, specially for receiving data from ep0 out
 * */
#define CONFIG_USBDEV_REQUEST_BUFFER_LEN	256

/* ================ USB Device Port Configuration ================*/

#define CONFIG_USB_DWC2_PORT		FS_PORT
#define USBD_IRQHandler			irq_OTG_FS

#include "hal/hwdefs.h"

#if defined(HW_MCU_STM32F722) && !defined(STM32F7)
#define STM32F7
#endif /* HW_MCU_STM32F722 */

#endif /* CHERRYUSB_CONFIG_H */

