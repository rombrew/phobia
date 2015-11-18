
BUILD	?= /tmp/pmc
TARGET	= $(BUILD)/pmc

CROSS	?= arm-none-eabi-
CC      = $(CROSS)gcc
AS      = $(CROSS)as
LD      = $(CROSS)ld
OC      = $(CROSS)objcopy
OD      = $(CROSS)objdump
MK	= mkdir -p
RM      = rm -rf

CFLAGS  = -std=gnu99 -pipe -Wall -O3
CFLAGS	+= -mcpu=cortex-m4 -mthumb
CFLAGS	+= -mhard-float -mfpu=fpv4-sp-d16
CFLAGS  += -fno-hosted
CFLAGS  += -fno-stack-protector
AFLAGS	= -mcpu=cortex-m4 -mthumb
LFLAGS  = -T hal/ram.ld

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
	@ $(AS) $(AFLAGS) -o $@ $<

$(BUILD)/%.o: %.c
	@ echo "  CC    " $<
	@ $(MK) $(dir $@)
	@ $(CC) -c $(CFLAGS) -MMD -o $@ $<

$(TARGET): $(LIST)
	@ echo "  LD    " $(notdir $@)
	@ $(LD) $(LFLAGS) -o $@ $^
	@ $(OC) -j .text -j .data -O binary $@

flash: $(TARGET)
	@ echo "  FLASH " $(notdir $^)
	@ stmflasher -p $(TTY) -b $(BAUD) -w $(TARGET) -Mr -v -g +0
#	@ stmflasher -p $(TTY) -b $(BAUD) -w $(TARGET) -v -g +0
	@ picocom -l -b $(BAUD) -pe $(TTY)

clean:
	@ echo "  CLEAN "
	@ $(RM) $(BUILD)

include $(wildcard $(BUILD)/*.d)

