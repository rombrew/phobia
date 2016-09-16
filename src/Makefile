
BUILD	?= /tmp/pmc
TARGET	= $(BUILD)/pmc

CROSS	?= arm-none-eabi-
CC	= $(CROSS)gcc
OC	= $(CROSS)objcopy
MK	= mkdir -p
RM	= rm -rf

CFLAGS	= -std=gnu99 -pipe
CFLAGS	+= -mcpu=cortex-m4 -mthumb
CFLAGS	+= -mhard-float -mfpu=fpv4-sp-d16
CFLAGS	+= -Wall -Wdouble-promotion
CFLAGS	+= -O3 -flto
CFLAGS	+= -fno-math-errno
CFLAGS	+= -ffinite-math-only
CFLAGS	+= -fno-signed-zeros
CFLAGS	+= -fno-trapping-math
CFLAGS	+= -fno-hosted
CFLAGS	+= -fno-stack-protector
CFLAGS  += -I.

LDFLAGS	= -nostdlib
LDFLAGS	+= -Wl,-T,hal/ram.ld

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
	hal/hal.o \
	hal/pwm.o \
	hal/usart.o

OBJS	+= ap.o \
	conf.o \
	lib.o \
	m.o \
	main.o \
	pm.o \
	pmc.o \
	sh.o \
	tel.o

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
	@ $(OC) -j .text -j .data -O binary $@

flash: $(TARGET)
	@ echo "  FLASH " $(notdir $^)
	@ stmflasher -p $(TTY) -b $(BAUD) -w $(TARGET) -Mr -v -g +0
#	@ stmflasher -p $(TTY) -b $(BAUD) -w $(TARGET) -v -g +0
	@ picocom -l -b $(BAUD) -ye $(TTY)

resume:
	@ picocom -l -b $(BAUD) -ye $(TTY)

clean:
	@ echo "  CLEAN "
	@ $(RM) $(BUILD)

include $(wildcard $(BUILD)/*.d) $(wildcard $(BUILD)/**/*.d)

