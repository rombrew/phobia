BUILD	?= /tmp/pmcfe
TARGET	= $(BUILD)/pmcfe

CC	= gcc
LD	= gcc
GDB	= gdb
MK	= mkdir -p
RM      = rm -rf

CFLAGS  = -std=gnu99 -pipe -Wall -O3 -flto -g3
CFLAGS  += -I/usr/include/SDL2 -D_REENTRANT

LFLAGS	= -lm -lSDL2 -lSDL2_ttf

OBJS	= blob_ttf.o \
	  config.o \
	  link.o \
	  nksdl.o \
	  pmcfe.o \
	  serial.o

PMCFE_OBJS = $(addprefix $(BUILD)/, $(OBJS))

all: $(TARGET)

$(BUILD)/%.o: %.c
	@ echo "  CC    " $<
	@ $(MK) $(dir $@)
	@ $(CC) -c $(CFLAGS) -MMD -o $@ $<

$(TARGET): $(PMCFE_OBJS)
	@ echo "  LD    " $(notdir $@)
	@ $(LD) $(CFLAGS) -o $@ $^ $(LFLAGS)

run: $(TARGET)
	@ echo "  RUN	" $(notdir $<)
	@ $<

debug: $(TARGET)
	@ echo "  GDB	" $(notdir $<)
	@ $(GDB) $<

clean:
	@ echo "  CLEAN "
	@ $(RM) $(BUILD)

include $(wildcard $(BUILD)/*.d)

