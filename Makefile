# Makefile by Dan Fay
# Designed for use with STM32F4 and Libopencm3

DEVICE          = STM32F401RCT6
OPENCM3_DIR     = libopencm3
OBJS            += $(wildcard src/*.c)

INC             += src/

CFLAGS          += -O0 -ggdb3 -Wall -Werror 
CPPFLAGS	    += -DSTM32F4
LDFLAGS         += -static -nostartfiles
LDLIBS          += -Wl,--start-group -lc -lgcc -lnosys -Wl,--end-group

include $(OPENCM3_DIR)/mk/genlink-config.mk
include $(OPENCM3_DIR)/mk/gcc-config.mk

.PHONY: clean all lib

all: lib main.elf main.bin

# Build libopencm3
#lib:
#	make -C $(OPENCM3_DIR)/ all

clean:
	$(Q)$(RM) -rf *.elf *.bin *.o generated.*

include $(OPENCM3_DIR)/mk/genlink-rules.mk
include $(OPENCM3_DIR)/mk/gcc-rules.mk