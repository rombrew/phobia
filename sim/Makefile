
BUILD	?= /tmp/sim
TARGET	= $(BUILD)/sim

CC	= gcc
LD	= gcc
MK	= mkdir -p
RM      = rm -rf

CFLAGS	= -std=gnu99 -pipe -Wall -O3 -flto -g3
LFLAGS	= -lm -g3

OBJS	= lib.o blm.o sim.o
OBJS	+= pmc.o

LIST	= $(addprefix $(BUILD)/, $(OBJS))

all: $(TARGET)

$(BUILD)/%.o: %.c
	@ echo "  CC    " $<
	@ $(MK) $(dir $@)
	@ $(CC) -c $(CFLAGS) -MMD -o $@ $<

$(TARGET): $(LIST)
	@ echo "  LD    " $(notdir $@)
	@ $(LD) $(CFLAGS) $(LFLAGS) -o $@ $^

run: $(TARGET)
	@ echo "  RUN	" $(notdir $^)
	@ $(TARGET)

clean:
	@ echo "  CLEAN "
	@ $(RM) $(BUILD)

include $(wildcard $(BUILD)/*.d)

