BUILD	?= /tmp/pmc
TARGET	= $(BUILD)/pmc
IMAGE	= $(TARGET).b

CROSS	?= armv7m-none-eabi
#CC	= $(CROSS)-gcc
CC	= clang -target $(CROSS)
OC	= $(CROSS)-objcopy
GDB     = $(CROSS)-gdb
MK	= mkdir -p
RM	= rm -rf

CFLAGS	= -Os -flto -g3 -pipe
CFLAGS	+= -mcpu=cortex-m4 -mthumb
CFLAGS	+= -mfloat-abi=hard -mfpu=fpv4-sp-d16
CFLAGS	+= -Wall -Wdouble-promotion
CFLAGS	+= -ffreestanding
CFLAGS	+= -fno-stack-protector
CFLAGS	+= -fno-math-errno
CFLAGS	+= -ffinite-math-only
CFLAGS	+= -fno-signed-zeros
CFLAGS	+= -fno-trapping-math
CFLAGS	+= -ffp-contract=fast
CFLAGS  += -I.

#CFLAGS  += -D_HW_REV2
#CFLAGS  += -D_HW_KLEN
#CFLAGS  += -D_HW_REV4B
CFLAGS  += -D_HW_REV4C

LDFLAGS	= -nostdlib
LDFLAGS	+= -Wl,-T,hal/link.ld

TTY	= /dev/rfcomm0
#TTY	= /dev/ttyUSB0
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
	  hal/tim.o \
	  hal/usart.o \
	  hal/wd.o \
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
	  servo.o \
	  shell.o \
	  tlm.o

OBJS	+= apps/hx711.o \
	   apps/mice.o \
	   apps/pushb.o

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
	@ stm32flash -w $< -v -S 0x08000000:262144 -b $(BAUD) -R $(TTY)

zip: $(IMAGE)
	@ echo "  ZIP   " $(notdir $<)
	@ zip -j9 $(BUILD)/pmc-$(shell hg id -b)-$(shell hg id -n).zip $<

gdb: $(TARGET)
	@ echo "  GDB   " $(notdir $<)
	@ $(GDB) $<

connect:
	@ picocom -l -b $(BAUD) -ye $(TTY)

clean:
	@ echo "  CLEAN "
	@ $(RM) $(BUILD)

include $(wildcard $(BUILD)/*.d) $(wildcard $(BUILD)/**/*.d)

