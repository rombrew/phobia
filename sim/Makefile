
TARGET	= sim

CC	= gcc
LD	= gcc
RM      = rm -f

CFLAGS	= -std=gnu99 -pipe -Wall -O2 -g3
LFLAGS	= -lm -g3

OBJS	= lib.o blm.o sim.o
OBJS	+= blc.o

all: $(TARGET)

%.o: %.c
	@ echo "  CC    " $<
	@ $(CC) -c $(CFLAGS) -MMD -o $@ $<

$(TARGET): $(OBJS)
	@ echo "  LD    " $@
	@ $(LD) $(LFLAGS) -o $@ $^

ctags:
	@ echo "  CTAGS "
	@ ctags --c-kinds=+px --fields=+Sn -R .
	@ cscope -b -k -f scope -R -s .

clean:
	@ echo "  CLEAN "
	@ $(RM) $(TARGET) $(OBJS) $(patsubst %.o, %.d, $(OBJS))
	@ $(RM) tags scope

include $(wildcard *.d)

