HWREV	?= REV5A

include hal/mk/$(HWREV).d

BUILD	?= /tmp/pmcfw-$(HWREV)
TARGET	= $(BUILD)/pmcfw-$(HWREV)
IMAGE	= $(TARGET).bin

CROSS	?= armv7m-none-eabi
CC	= $(CROSS)-gcc
#CC	= clang -target $(CROSS)
OC	= $(CROSS)-objcopy
#GDB	= $(CROSS)-gdb
GDB	= /home/amaora/src/blackmagic/armv7m-none-eabi-gdb
MK	= mkdir -p
RM	= rm -rf

CFLAGS	= -Os -flto -g3 -pipe

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

ifeq ($(HWMCU), STM32F405)
LDFLAGS	+= -Wl,-T,hal/link_f405.ld
endif

ifeq ($(HWMCU), STM32F722)
LDFLAGS	+= -Wl,-T,hal/link_f722.ld
endif

#TTY	= /dev/rfcomm0
TTY	= /dev/ttyUSB0
BAUD	= 57600

LOGFILE	= /tmp/phobia.log

OBJS    = freertos/heap_4.o \
	  freertos/list.o \
	  freertos/port.o \
	  freertos/queue.o \
	  freertos/tasks.o \
	  hal/adc.o \
	  $(INCLUDE_HAL_CAN) \
	  hal/dps.o \
	  $(INCLUDE_HAL_DRV) \
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
	  hal/wd.o \
	  phobia/libm.o \
	  phobia/lse.o \
	  phobia/pm.o \
	  phobia/pm_fsm.o \
	  flash.o \
	  $(INCLUDE_IFCAN) \
	  libc.o \
	  main.o \
	  ntc.o \
	  pmfunc.o \
	  pmtest.o \
	  regfile.o \
	  servo.o \
	  shell.o \
	  tlm.o

OBJS	+= apps/hx711.o \
	   apps/push.o

TARGET_OBJS = $(addprefix $(BUILD)/, $(OBJS))

all: $(TARGET)

$(BUILD)/%.o: %.c
	@ echo "  CC    " $<
	@ $(MK) $(dir $@)
	@ $(CC) -c $(CFLAGS) -MMD -o $@ $<

$(TARGET): $(TARGET_OBJS)
	@ echo "  LD    " $(notdir $@)
	@ $(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

$(IMAGE): $(TARGET)
	@ echo "  OC    " $(notdir $@)
	@ $(OC) -j .text -j .data -O binary -S $< $@

zip: $(IMAGE)
	@ echo "  ZIP   " $(notdir $<)
	@ zip -j9 $(BUILD)/pmcfw-$(HWREV)-$(shell hg id -n).zip $<

flash: $(IMAGE)
	@ echo "  FLASH " $(notdir $<)
	@ stm32flash -w $< -v -S 0x08000000:131072 -b $(BAUD) -R $(TTY)

stlink: $(IMAGE)
	@ echo "  FLASH " $(notdir $<)
	@ st-flash write $< 0x08000000

gdb: $(TARGET)
	@ echo "  GDB   " $(notdir $<)
	@ $(GDB) $<

connect:
	@ picocom -l -b $(BAUD) -ye $(TTY) -g $(LOGFILE)

clean:
	@ echo "  CLEAN "
	@ $(RM) $(BUILD)

include $(wildcard $(BUILD)/*.d) $(wildcard $(BUILD)/**/*.d)

