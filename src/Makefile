
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
CFLAGS	+= -fno-math-errno
CFLAGS	+= -ffinite-math-only
CFLAGS	+= -fno-signed-zeros
CFLAGS	+= -fno-trapping-math
CFLAGS	+= -fno-hosted
CFLAGS	+= -fno-stack-protector
CFLAGS	+= -O3 -flto

LDFLAGS	= -nostdlib
LDFLAGS	+= -Wl,-T,hal/ram.ld

TTY	= /dev/rfcomm0
BAUD	= 57600

OBJS	= hal/entry.o \
	hal/adc.o \
	hal/can.o \
	hal/hal.o \
	hal/pwm.o \
	hal/usart.o

OBJS	+= ap.o \
	cml.o \
	lib.o \
	m.o \
	pmc.o \
	sh.o \
	task.o \
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
	@ $(CC) -c $(CFLAGS) -I $(BUILD) -MMD -o $@ $<

$(TARGET): $(LIST)
	@ echo "  LD    " $(notdir $@)
	@ $(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^
	@ $(OC) -j .text -j .data -O binary $@

flash: $(TARGET)
	@ echo "  FLASH " $(notdir $^)
	@ stmflasher -p $(TTY) -b $(BAUD) -w $(TARGET) -Mr -v -g +0
#	@ stmflasher -p $(TTY) -b $(BAUD) -w $(TARGET) -v -g +0
	@ picocom -l -b $(BAUD) -pe $(TTY)

resume:
	@ picocom -l -b $(BAUD) -pe $(TTY)

clean:
	@ echo "  CLEAN "
	@ $(RM) $(BUILD)

include $(wildcard $(BUILD)/*.d)

