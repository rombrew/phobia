/*
 * Copyright (c) 2022, sakumisu
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef CHERRYUSB_CONFIG_H
#define CHERRYUSB_CONFIG_H

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

#define USB_NOCACHE_RAM_SECTION
#define USB_MEM_ALIGNX	__attribute__ ((aligned(CONFIG_USB_ALIGN_SIZE)))

/* ================= USB Device Stack Configuration ================ */

/* Ep0 max transfer buffer, specially for receiving data from ep0 out
 * */
#define CONFIG_USBDEV_REQUEST_BUFFER_LEN	256

/* ================ USB Device Port Configuration ================*/

#define CONFIG_USB_DWC2_PORT		0	/* OTF_FS */
/*#define CONFIG_USB_DWC2_DMA_ENABLE*/

#define USBD_IRQHandler			irq_OTG_FS

#endif /* CHERRYUSB_CONFIG_H */

