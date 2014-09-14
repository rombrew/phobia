
BUILD	?= /tmp/build
TARGET	= $(BUILD)/sim

CC	= gcc
LD	= gcc
MK	= mkdir -p
RM      = rm -rf

CFLAGS	= -std=gnu99 -pipe -Wall -O2 -g3
CFLAGS  += -D__MATH_C
LFLAGS	= -lm -g3

OBJS	= lib.o blm.o sim.o
OBJS	+= math.o
OBJS	+= blc.o

LIST	= $(addprefix $(BUILD)/, $(OBJS))

all: $(TARGET)

$(BUILD)/%.o: %.c
	@ echo "  CC    " $<
	@ $(MK) $(dir $@)
	@ $(CC) -c $(CFLAGS) -MMD -o $@ $<

$(TARGET): $(LIST)
	@ echo "  LD    " $(notdir $@)
	@ $(LD) $(LFLAGS) -o $@ $^

run: $(TARGET)
	@ echo "  RUN	" $(notdir $^)
	@ $(TARGET)

tags:
	@ echo "  TAGS "
	@ $(MK) $(BUILD)
	@ ctags --c-kinds=+px --fields=+Sn -R -f $(BUILD)/tags .
	@ cscope -b -k -R -f $(BUILD)/scope -s .

clean:
	@ echo "  CLEAN "
	@ $(RM) $(BUILD)

include $(wildcard $(BUILD)/*.d)

