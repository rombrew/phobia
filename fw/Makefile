
TARGET	= phobia

HAL	?= stm32
REV	?= 1
LDS	?= ram
FW	?= bl
SIM	?= sim_bldc

CROSS	?= arm-none-eabi-

CC      = $(CROSS)gcc
AS      = $(CROSS)as
LD      = $(CROSS)ld
OC      = $(CROSS)objcopy
OD      = $(CROSS)objdump
MKDIR	= mkdir -p
RM      = rm -rf
HOST_CC	= gcc
HOST_LD	= gcc

CFLAGS  = -std=gnu99 -pipe -Wall -O2
CFLAGS  += -I. -DHWREV=$(REV) -DHWINL=\"hal/$(HAL)/hwinl.h\"
CFLAGS	+= -mcpu=cortex-m4 -mhard-float -mthumb
CFLAGS  += -fno-builtin
AFLAGS	= -mcpu=cortex-m4 -mthumb
LFLAGS  = -T ld/$(LDS).ld
HOST_CFLAGS	= -std=gnu99 -pipe -Wall -O2
HOST_CFLAGS	+= -I. -DHWINL=\"hal/$(SIM)/hwinl.h\"
HOST_LFLAGS	= -lm -pthread

SRCDIR	= $(FW) cm lib
SEARCH	= $(sort $(addprefix $(2)/,		\
		$(patsubst %.s, %.o, $(wildcard		\
			$(addsuffix /*.s, $(1))))	\
		$(patsubst %.c, %.o, $(wildcard		\
			$(addsuffix /*.c, $(1))))))

BUILD	= ../build/$(HAL)
DIRS	= hal/$(HAL) $(SRCDIR)
OBJS	= $(call SEARCH, $(DIRS), $(BUILD))

HOST_BUILD	= ../build/$(SIM)
HOST_DIRS	= hal/$(SIM) $(SRCDIR)
HOST_OBJS	= $(call SEARCH, $(HOST_DIRS), $(HOST_BUILD))

TTY	= /dev/ttyUSB0

all: $(HOST_BUILD)/$(TARGET) $(BUILD)/$(TARGET)

hal: $(BUILD)/$(TARGET)
sim: $(HOST_BUILD)/$(TARGET)

$(BUILD)/%.o: %.s
	@ echo "  AS    " $<
	@ $(MKDIR) $(dir $@)
	@ $(AS) $(AFLAGS) -o $@ $<

$(BUILD)/%.o: %.c
	@ echo "  CC    " $<
	@ $(MKDIR) $(dir $@)
	@ $(CC) -c $(CFLAGS) -MMD -o $@ $<

$(HOST_BUILD)/%.o: %.c
	@ echo "  HOSTCC" $<
	@ $(MKDIR) $(dir $@)
	@ $(HOST_CC) -c $(HOST_CFLAGS) -MMD -o $@ $<

$(BUILD)/$(TARGET): $(OBJS)
	@ echo "  LD    " $@
	@ $(LD) $(LFLAGS) -o $@ $^
	@ $(OC) -j .text -j .data -O binary $@

$(HOST_BUILD)/$(TARGET): $(HOST_OBJS)
	@ echo "  HOSTLD" $@
	@ $(HOST_LD) $(HOST_LFLAGS) -o $@ $^

flash: $(BUILD)/$(TARGET)
	@ echo "  FLASH " $^
#	@ stmflasher -p $(TTY) -w $(BUILD)/$(TARGET) -v -g +0
	@ stmflasher -p $(TTY) -w $(BUILD)/$(TARGET) -Mr -S +0x1000 -v -g +0
	@ picocom -l -b 9600 $(TTY)

tags:
	@ echo "  CTAGS "
	@ ctags --c-kinds=+px --fields=+Sn -R .
	@ cscope -b -k -f scope -R -s .

clean:
	@ echo "  CLEAN "
	@ $(RM) $(BUILD) $(HOST_BUILD)
	@ $(RM) tags scope

include $(wildcard *.d)

