BUILD	?= /tmp/phobia
TARGET	= $(BUILD)/phobia

CC	= gcc
LD	= gcc
GDB	= gdb
MK	= mkdir -p
RM	= rm -rf

CFLAGS	= -std=gnu99 -pipe -Wall -O3 -flto=auto -g3
CFLAGS	+= -fno-stack-protector
CFLAGS	+= -I/usr/include/SDL2 -D_REENTRANT
CFLAGS	+= -D_EMBED_GP

LFLAGS	= -lm -lSDL2 -lSDL2_ttf -lSDL2_image

OBJS	= blob_ttf.o \
	  config.o \
	  dirent.o \
	  link.o \
	  nksdl.o \
	  phobia.o \
	  serial.o

OBJS	+= gp/async.o \
	   gp/blob_ttf.o \
	   gp/dirent.o \
	   gp/draw.o \
	   gp/edit.o \
	   gp/gp.o \
	   gp/lang.o \
	   gp/lse.o \
	   gp/lz4.o \
	   gp/menu.o \
	   gp/plot.o \
	   gp/read.o \
	   gp/scheme.o \
	   gp/svg.o

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

include $(wildcard $(BUILD)/*.d) $(wildcard $(BUILD)/*/*.d)

