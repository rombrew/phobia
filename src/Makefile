
BUILD	?= /tmp/pmc
TARGET	= $(BUILD)/pmc
IMAGE	= $(TARGET).b

CROSS	?= armv7m-none-eabi-
CC	= $(CROSS)gcc
OC	= $(CROSS)objcopy
GDB     = $(CROSS)gdb
MK	= mkdir -p
RM	= rm -rf

CFLAGS	= -pipe
CFLAGS	+= -mcpu=cortex-m4 -mthumb
CFLAGS	+= -mhard-float -mfpu=fpv4-sp-d16
CFLAGS	+= -Wall -Wdouble-promotion
CFLAGS	+= -Os -flto -g3
CFLAGS	+= -fno-hosted
CFLAGS	+= -fno-stack-protector
CFLAGS	+= -fno-math-errno
CFLAGS	+= -ffinite-math-only
CFLAGS	+= -fno-signed-zeros
CFLAGS	+= -fno-trapping-math
CFLAGS  += -I.

LDFLAGS	= -nostdlib
LDFLAGS	+= -Wl,-T,hal/link.ld

TTY	= /dev/rfcomm0
BAUD	= 57600

OBJS    = freertos/heap_4.o \
	  freertos/list.o \
	  freertos/port.o \
	  freertos/queue.o \
	  freertos/tasks.o \
	  hal/entry.o \
	  hal/adc.o \
	  hal/can.o \
	  hal/flash.o \
	  hal/gpio.o \
	  hal/hal.o \
	  hal/ppm.o \
	  hal/pwm.o \
	  hal/usart.o \
	  phobia/libm.o \
	  phobia/pm.o \
	  phobia/pm_fsm.o \
	  flash.o \
	  ifcan.o \
	  libc.o \
	  main.o \
	  ntc.o \
	  pmfunc.o \
	  pmtest.o \
	  regfile.o \
	  shell.o \
	  tel.o

OBJS	+= apps/hx711.o \
	   apps/push2.o \
	   apps/volt.o

LIST	= $(addprefix $(BUILD)/, $(OBJS))

all: $(TARGET)

$(BUILD)/%.o: %.c
	@ echo "  CC    " $<
	@ $(MK) $(dir $@)
	@ $(CC) -c $(CFLAGS) -MMD -o $@ $<

$(TARGET): $(LIST)
	@ echo "  LD    " $(notdir $@)
	@ $(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

$(IMAGE): $(TARGET)
	@ echo "  OC    " $(notdir $@)
	@ $(OC) -j .text -j .data -O binary -S $< $@

flash: $(IMAGE)
	@ echo "  FLASH " $(notdir $<)
	@ stmflasher -p $(TTY) -b $(BAUD) -w $< -v -g +0

gdb: $(TARGET)
	@ echo "  GDB   " $(notdir $<)
	@ $(GDB) $<

connect:
	@ picocom -l -b $(BAUD) -ye $(TTY)

clean:
	@ echo "  CLEAN "
	@ $(RM) $(BUILD)

include $(wildcard $(BUILD)/*.d) $(wildcard $(BUILD)/**/*.d)

