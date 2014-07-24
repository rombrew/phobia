
TARGET	= fw

CROSS	?= arm-none-eabi-
CC      = $(CROSS)gcc
AS      = $(CROSS)as
LD      = $(CROSS)ld
OC      = $(CROSS)objcopy
OD      = $(CROSS)objdump
MKDIR	= mkdir -p
RM      = rm -rf

CFLAGS  = -std=gnu99 -pipe -Wall -O2
CFLAGS	+= -mcpu=cortex-m4 -mhard-float -mthumb
CFLAGS  += -fno-hosted
CFLAGS  += -fno-stack-protector
AFLAGS	= -mcpu=cortex-m4 -mthumb
LFLAGS  = -T hal/ram.ld

TTY	= /dev/rfcomm0
BAUD	= 57600

OBJS	= hal/entry.o \
	hal/hal.o \
	hal/pwm.o \
	hal/uart.o

OBJS	+= fw.o \
	lib.o \
	shell.o

all: $(TARGET)

%.o: %.s
	@ echo "  AS    " $<
	@ $(AS) $(AFLAGS) -o $@ $<

%.o: %.c
	@ echo "  CC    " $<
	@ $(CC) -c $(CFLAGS) -MMD -o $@ $<

$(TARGET): $(OBJS)
	@ echo "  LD    " $@
	@ $(LD) $(LFLAGS) -o $@ $^
	@ $(OC) -j .text -j .data -O binary $@

flash: $(TARGET)
	@ echo "  FLASH " $^
	@ stmflasher -p $(TTY) -b $(BAUD) -w $(TARGET) -Mr -v -g +0
	@ picocom -l -b $(BAUD) -pe $(TTY)

ctags:
	@ echo "  CTAGS "
	@ ctags --c-kinds=+px --fields=+Sn -R .
	@ cscope -b -k -f scope -R -s .

clean:
	@ echo "  CLEAN "
	@ $(RM) $(TARGET) $(OBJS) $(patsubst %.o, %.d, $(OBJS))
	@ $(RM) tags scope

include $(wildcard *.d)

