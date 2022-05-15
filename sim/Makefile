BUILD	?= /tmp/pmcsim
TARGET	= $(BUILD)/pmcsim

CC	= gcc
LD	= gcc
GDB	= gdb
MK	= mkdir -p
RM      = rm -rf

CFLAGS	= -std=gnu99 -pipe -Wall -O3 -flto -g3
LFLAGS	= -lm

OBJS	= blm.o lfg.o pm.o sim.o tsfunc.o

SIM_OBJS = $(addprefix $(BUILD)/, $(OBJS))

all: $(TARGET)

$(BUILD)/%.o: %.c
	@ echo "  CC    " $<
	@ $(MK) $(dir $@)
	@ $(CC) -c $(CFLAGS) -MMD -o $@ $<

$(TARGET): $(SIM_OBJS)
	@ echo "  LD    " $(notdir $@)
	@ $(LD) $(CFLAGS) -o $@ $^ $(LFLAGS)

run: $(TARGET)
	@ echo "  RUN	" $(notdir $<)
	@ $<

test: $(TARGET)
	@ echo "  TEST	" $(notdir $<)
	@ $< test

debug: $(TARGET)
	@ echo "  GDB	" $(notdir $<)
	@ $(GDB) $<

clean:
	@ echo "  CLEAN "
	@ $(RM) $(BUILD)

include $(wildcard $(BUILD)/*.d)

