
BUILD	?= /tmp/pmc
SCRIPT	?= flash
TARGET	= $(BUILD)/pmc
IMAGE	= $(TARGET).b

CROSS	?= armv7m-none-eabi-
CC	= $(CROSS)gcc
OC	= $(CROSS)objcopy
GDB     = $(CROSS)gdb
MK	= mkdir -p
RM	= rm -rf

CFLAGS	= -std=gnu99 -pipe
CFLAGS	+= -mcpu=cortex-m4 -mthumb
CFLAGS	+= -mhard-float -mfpu=fpv4-sp-d16
CFLAGS	+= -Wall -Wdouble-promotion
CFLAGS	+= -O3 -flto
#CFLAGS	+= -static -fno-pic -fno-PIC -fno-pie -fno-PIE
CFLAGS	+= -fno-hosted
CFLAGS	+= -fno-stack-protector
CFLAGS	+= -fno-math-errno
CFLAGS	+= -ffinite-math-only
CFLAGS	+= -fno-signed-zeros
CFLAGS	+= -fno-trapping-math
CFLAGS  += -I.

LDFLAGS	= -nostdlib
LDFLAGS	+= -Wl,-T,hal/$(SCRIPT).ld

TTY	= /dev/rfcomm0
BAUD	= 57600

OBJS    = freertos/heap_4.o \
        freertos/list.o \
        freertos/port.o \
        freertos/queue.o \
        freertos/tasks.o \
        freertos/timers.o

OBJS	+= hal/entry.o \
	hal/adc.o \
	hal/can.o \
	hal/flash.o \
	hal/gpio.o \
	hal/hal.o \
	hal/pwm.o \
	hal/usart.o

OBJS	+= flash.o \
	lib.o \
	main.o \
	ntc.o \
	pm_control.o \
	pm_fsm.o \
	pm_math.o \
	regfile.o \
	shell.o \
	telinfo.o

LIST	= $(addprefix $(BUILD)/, $(OBJS))

all: $(TARGET)

$(BUILD)/%.o: %.s
	@ echo "  AS    " $<
	@ $(MK) $(dir $@)
	@ $(CC) -c $(CFLAGS) -o $@ $<

$(BUILD)/%.o: %.c
	@ echo "  CC    " $<
	@ $(MK) $(dir $@)
	@ $(CC) -c $(CFLAGS) -MMD -o $@ $<

$(TARGET): $(LIST)
	@ echo "  LD    " $(notdir $@)
	@ $(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^

$(IMAGE): $(TARGET)
	@ echo "  OC    " $(notdir $@)
	@ $(OC) -j .text -j .data -O binary $< $@

flash: $(IMAGE)
	@ echo "  FLASH " $(notdir $<)
	@ stmflasher -p $(TTY) -b $(BAUD) -w $< -v -g +0

ram: $(IMAGE)
	@ echo "  RAM   " $(notdir $<)
	@ stmflasher -p $(TTY) -b $(BAUD) -w $< -Mr -v -g +0

gdb: $(TARGET)
	@ echo "  GDB   " $(notdir $<)
	@ $(GDB) $<

connect:
	@ picocom -l -b $(BAUD) -ye $(TTY)

clean:
	@ echo "  CLEAN "
	@ $(RM) $(BUILD)

include $(wildcard $(BUILD)/*.d) $(wildcard $(BUILD)/**/*.d)

