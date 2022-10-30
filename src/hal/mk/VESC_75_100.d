HWMCU = STM32F405
INCLUDE_HAL_USB = hal/usb.o
INCLUDE_CHERRY = cherry/usb_dc_dwc2.o \
                 cherry/usbd_cdc.o \
                 cherry/usbd_core.o
INCLUDE_HAL_CAN = hal/can.o
INCLUDE_EPCAN = epcan.o
