BUILD	?= /tmp/pgui
TARGET	= $(BUILD)/pgui

CC	= gcc
LD	= gcc
STRIP	= strip -s
GDB	= gdb
MK	= mkdir -p
RM	= rm -rf

CFLAGS	= -std=gnu99 -Wall -O3 -flto=auto -g3 -pipe

CFLAGS	+= -fno-math-errno \
	   -ffinite-math-only \
	   -fno-signed-zeros \
	   -fno-trapping-math \
	   -fno-associative-math \
	   -fno-reciprocal-math \
	   -ffp-contract=fast

CFLAGS	+= -fno-stack-protector
CFLAGS	+= -I/usr/include -D_REENTRANT
CFLAGS	+= -D_EMBED_GP

LFLAGS	= -lm -lSDL2 -lSDL2_ttf -lSDL2_image

OBJS	= config.o \
	  font.o \
	  link.o \
	  nksdl.o \
	  phobia.o \
	  serial.o

OBJS	+= gp/async.o \
	   gp/dirent.o \
	   gp/draw.o \
	   gp/edit.o \
	   gp/font.o \
	   gp/gp.o \
	   gp/lang.o \
	   gp/lse.o \
	   gp/lz4.o \
	   gp/menu.o \
	   gp/plot.o \
	   gp/read.o \
	   gp/scheme.o \
	   gp/svg.o

BUILD_OBJS = $(addprefix $(BUILD)/, $(OBJS))

all: $(TARGET)

$(BUILD)/%.o: %.c
	@ echo "  CC    " $<
	@ $(MK) $(dir $@)
	@ $(CC) -c $(CFLAGS) -MMD -o $@ $<

$(TARGET): $(BUILD_OBJS)
	@ echo "  LD    " $(notdir $@)
	@ $(LD) $(CFLAGS) -o $@ $^ $(LFLAGS)

run: $(TARGET)
	@ echo "  RUN	" $(notdir $<)
	@ $<

debug: $(TARGET)
	@ echo "  GDB	" $(notdir $<)
	@ $(GDB) $<

strip: $(TARGET)
	@ echo "  STRIP " $(notdir $<)
	@ $(STRIP) $<

clean:
	@ echo "  CLEAN "
	@ $(RM) $(BUILD)

include $(wildcard $(BUILD)/*.d) $(wildcard $(BUILD)/*/*.d)

