HWREV	?= REV5A

include hal/mk/$(HWREV).d

BUILD	?= /tmp/pmc-$(HWREV)
LOG	?= /tmp/phobia.log

TARGET	= $(BUILD)/pmc-$(HWREV)
IMAGE	= $(TARGET).bin

TTY	?= /dev/ttyUSB0
BAUD	?= 57600

CROSS	?= arm-none-eabi
CC	= $(CROSS)-gcc
OC	= $(CROSS)-objcopy
GDB	= $(CROSS)-gdb
MK	= mkdir -p
RM	= rm -rf

#CROSS	= armv7m-none-eabi
#CC	= clang -target $(CROSS)

CFLAGS	= -O3 -flto=auto -g3 -pipe
CFLAGS	+= -Wno-stringop-overflow

ifeq ($(HWMCU), STM32F405)
CFLAGS	+= -mcpu=cortex-m4 -mthumb
CFLAGS	+= -mfloat-abi=hard -mfpu=fpv4-sp-d16
endif

ifeq ($(HWMCU), STM32F722)
CFLAGS	+= -mcpu=cortex-m7 -mthumb
CFLAGS	+= -mfloat-abi=hard -mfpu=fpv5-sp-d16
endif

CFLAGS	+= -Wall -Wdouble-promotion
CFLAGS	+= -ffreestanding
CFLAGS	+= -fno-stack-protector
CFLAGS	+= -fno-math-errno
CFLAGS	+= -ffinite-math-only
CFLAGS	+= -fno-signed-zeros
CFLAGS	+= -fno-trapping-math
CFLAGS	+= -fno-associative-math
CFLAGS	+= -fno-reciprocal-math
CFLAGS	+= -ffp-contract=fast
CFLAGS  += -I.
CFLAGS	+= -D_HW_REV=\"$(HWREV)\" \
	   -D_HW_INCLUDE=\"hal/hw/$(HWREV).h\"

LDFLAGS	= -nostdlib
LDFLAGS	+= -Wl,--no-warn-rwx-segment
LDFLAGS += -Wl,--print-memory-usage

ifeq ($(HWMCU), STM32F405)
LDFLAGS	+= -Wl,-T,hal/link_f405.ld
endif

ifeq ($(HWMCU), STM32F722)
LDFLAGS	+= -Wl,-T,hal/link_f722.ld
endif

OBJS	=  app/as5047.o \
	   app/hx711.o \
	   app/mpu6050.o \
	   app/pushbutton.o

OBJS	+= freertos/heap_4.o \
	   freertos/list.o \
	   freertos/port.o \
	   freertos/queue.o \
	   freertos/tasks.o

ifeq ($(OBJ_HAL_CAN), INCLUDE)
OBJS	+= hal/can.o
endif

ifeq ($(OBJ_HAL_DRV), INCLUDE)
OBJS	+= hal/drv.o
endif

OBJS	+= hal/adc.o \
	   hal/dac.o \
	   hal/dps.o \
	   hal/entry.o \
	   hal/flash.o \
	   hal/gpio.o \
	   hal/hal.o \
	   hal/ppm.o \
	   hal/pwm.o \
	   hal/rng.o \
	   hal/spi.o \
	   hal/tim.o \
	   hal/usart.o \
	   hal/wd.o

ifeq ($(OBJ_HAL_USB), INCLUDE)
OBJS	+= hal/usb.o
endif

ifeq ($(OBJ_LIB_CHERRY), INCLUDE)
OBJS	+= cherry/usb_dc_dwc2.o \
	   cherry/usbd_cdc.o \
	   cherry/usbd_core.o
endif

OBJS	+= phobia/libm.o \
	   phobia/lse.o \
	   phobia/pm.o \
	   phobia/pm_fsm.o

ifeq ($(OBJ_EPCAN), INCLUDE)
OBJS	+= epcan.o
endif

OBJS	+= flash.o \
	   libc.o \
	   main.o \
	   ntc.o \
	   pmfunc.o \
	   pmtest.o \
	   regfile.o \
	   shell.o \
	   tlm.o

PMC_OBJS = $(addprefix $(BUILD)/, $(OBJS))

all: $(TARGET)

$(BUILD)/%.o: %.c
	@ echo "  CC    " $<
	@ $(MK) $(dir $@)
	@ $(CC) -c $(CFLAGS) -MMD -o $@ $<

$(TARGET): $(PMC_OBJS)
	@ echo "  LD    " $(notdir $@)
	@ $(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

$(IMAGE): $(TARGET)
	@ echo "  OC    " $(notdir $@)
	@ $(OC) -j .text -j .data -O binary -S $< $@

gdb: $(TARGET)
	@ echo "  GDB   " $(notdir $<)
	@ $(GDB) $<

zip: $(IMAGE)
	@ echo "  ZIP   " $(notdir $<)
	@ zip -j9 $(BUILD)/pmc-$(HWREV)-$(shell hg id -n).zip $<

flash: $(IMAGE)
	@ echo "  FLASH " $(notdir $<)
	@ stm32flash -w $< -v -S 0x08000000:245760 -b $(BAUD) -R $(TTY)

stlink: $(IMAGE)
	@ echo "  FLASH " $(notdir $<)
	@ st-flash write $< 0x08000000

dfu: $(IMAGE)
	@ echo "  FLASH " $(notdir $<)
	@ dfu-util -d 0483:df11 -a0 --dfuse-address 0x08000000 -D $<

connect:
	@ picocom -l -b $(BAUD) -ye $(TTY) -g $(LOG)

clean:
	@ echo "  CLEAN "
	@ $(RM) $(BUILD)

include $(wildcard $(BUILD)/*.d) $(wildcard $(BUILD)/*/*.d)

