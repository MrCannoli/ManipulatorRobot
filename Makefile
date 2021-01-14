# Makefile by Dan Fay
# Designed for use with STM32F401

DEVICE           = stm32f401rct6
OPENCM3_DIR      = libopencm3
OBJS            += $(patsubst %.c,%.o,$(wildcard src/*.c))

# Compiler Flags
CFLAGS          += -std=c11
CFLAGS          += -O0 -ggdb3 -flto
CFLAGS          += -Wall -Wextra -Wshadow -Wconversion -Werror
CFLAGS          += -Isrc/
CPPFLAGS        += -MD -DSTM32F4

# Linker Flags
LDFLAGS         += -static -nostartfiles -lm -Xlinker -Map=main.map $(CFLAGS)
LDLIBS          += -Wl,--start-group -lc -lgcc -lnosys -Wl,--end-group -Wl,--gc-sections

# Note: Most device specific compiler & linker flags are added by libopencm3

include $(OPENCM3_DIR)/mk/genlink-config.mk
include $(OPENCM3_DIR)/mk/gcc-config.mk
include $(OPENCM3_DIR)/mk/genlink-rules.mk
include $(OPENCM3_DIR)/mk/gcc-rules.mk

.PHONY: clean format all lib load gdb

# Clean commands
clean:
	$(Q)$(RM) -rf main.* $(OBJS) generated*.ld $(wildcard src/*.d)

# Formatting Commands
format:
	clang-format -i src/*.c src/*.h

# Building Commands
all: lib main.elf main.bin

lib:
	$(Q)$(MAKE) -C $(OPENCM3_DIR)

# Black Magic Probe (BMP) load & debug commands
load: lib main.elf
	arm-none-eabi-gdb main.elf -x bmp.gdb -ex load

gdb: lib main.elf
	arm-none-eabi-gdb -x bmp.gdb
