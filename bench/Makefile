BUILD	?= /tmp/bench
TARGET	= $(BUILD)/bench

CC	= gcc
LD	= gcc
GDB	= gdb
MK	= mkdir -p
RM      = rm -rf

CFLAGS	= -std=gnu99 -Wall -O3 -flto=auto -g3 -pipe

CFLAGS	+= -fno-math-errno \
	   -ffinite-math-only \
	   -fno-signed-zeros \
	   -fno-trapping-math \
	   -fno-associative-math \
	   -fno-reciprocal-math \
	   -ffp-contract=fast

LFLAGS	= -lm

OBJS	= blm.o lfg.o pm.o bench.o tsfunc.o

SIM_OBJS = $(addprefix $(BUILD)/, $(OBJS))

all: $(TARGET)

$(BUILD)/%.o: %.c
	@ echo "  CC    " $<
	@ $(MK) $(dir $@)
	@ $(CC) -c $(CFLAGS) -MMD -o $@ $<

$(TARGET): $(SIM_OBJS)
	@ echo "  LD    " $(notdir $@)
	@ $(LD) $(CFLAGS) -o $@ $^ $(LFLAGS)

test: $(TARGET)
	@ echo "  TEST	" $(notdir $<)
	@ $< test

run: $(TARGET)
	@ echo "  RUN	" $(notdir $<)
	@ $< bench

data: $(TARGET)
	@ echo "  DATA	" $(notdir $<)
	@ $< data

debug: $(TARGET)
	@ echo "  GDB	" $(notdir $<)
	@ $(GDB) $<

clean:
	@ echo "  CLEAN "
	@ $(RM) $(BUILD)

include $(wildcard $(BUILD)/*.d) $(wildcard $(BUILD)/*/*.d)

