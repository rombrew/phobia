
BUILD	?= /tmp/vis
TARGET	= $(BUILD)/vis

CC	= gcc
LD	= gcc
MK	= mkdir -p
RM      = rm -rf

CFLAGS	= -std=gnu99 -pipe -Wall -O2 -g3
LFLAGS	= -lm -lSDL2 -lpthread -lGL -g3

OBJS	= vis.o

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

clean:
	@ echo "  CLEAN "
	@ $(RM) $(BUILD)

include $(wildcard $(BUILD)/*.d)

