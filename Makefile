##
## This file is part of the libopenstm32 project.
##
## Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
##
## This program is free software: you can redistribute it and/or modify
## it under the terms of the GNU General Public License as published by
## the Free Software Foundation, either version 3 of the License, or
## (at your option) any later version.
##
## This program is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
##
## You should have received a copy of the GNU General Public License
## along with this program.  If not, see <http://www.gnu.org/licenses/>.
##

OUT_DIR      = obj
PREFIX		?= arm-none-eabi
CONTROL     ?= CHARGER
CONTROLLC   := $(shell echo $(CONTROL) | tr A-Z a-z)
BINARY		= stm32_$(CONTROLLC)
SIZE        = $(PREFIX)-size
CC		      = $(PREFIX)-gcc
CPP	      = $(PREFIX)-g++
LD		      = $(PREFIX)-gcc
OBJCOPY		= $(PREFIX)-objcopy
OBJDUMP		= $(PREFIX)-objdump
MKDIR_P     = mkdir -p
CFLAGS		= -Os -Wall -Wextra -Iinclude/ -Ilibopeninv/include -Ilibopencm3/include \
             -fno-common -fno-builtin -pedantic -DSTM32F1 \
             -mcpu=cortex-m3 -mthumb -std=gnu99 -ffunction-sections -fdata-sections
CPPFLAGS    = -Os -Wall -Wextra -Iinclude/ -Ilibopeninv/include -Ilibopencm3/include \
            -fno-common -std=c++11 -pedantic -DSTM32F1 -DMAX_ITEMS=70 -DMAX_MESSAGES=15 \
            -ffunction-sections -fdata-sections -fno-builtin -fno-rtti -fno-exceptions -fno-unwind-tables -mcpu=cortex-m3 -mthumb
LDSCRIPT	  = linker.ld
LDFLAGS    = -Llibopencm3/lib -T$(LDSCRIPT) -march=armv7 -nostartfiles -Wl,--gc-sections,-Map,linker.map
OBJSL		  = main.o hwinit.o stm32scheduler.o params.o terminal.o terminal_prj.o \
             my_string.o digio.o sine_core.o my_fp.o printf.o anain.o \
             param_save.o errormessage.o canhardware.o stm32_can.o canmap.o cansdo.o \
             picontroller.o terminalcommands.o chargercan.o charger.o

OBJS     = $(patsubst %.o,obj/%.o, $(OBJSL))
DEPENDS  = $(patsubst %.o,obj/%.d, $(OBJSL))
vpath %.c src/ libopeninv/src
vpath %.cpp src/ libopeninv/src

OPENOCD_BASE	= /usr
OPENOCD		= $(OPENOCD_BASE)/bin/openocd
OPENOCD_SCRIPTS	= $(OPENOCD_BASE)/share/openocd/scripts
OPENOCD_FLASHER	= $(OPENOCD_SCRIPTS)/interface/parport.cfg
OPENOCD_BOARD	= $(OPENOCD_SCRIPTS)/board/olimex_stm32_h103.cfg

# Be silent per default, but 'make V=1' will show all compiler calls.
ifneq ($(V),1)
Q := @
NULL := 2>/dev/null
endif

all: directories images Test
Debug:images
Release: images
cleanDebug:clean
images: $(BINARY)
	@printf "  OBJCOPY $(BINARY).bin\n"
	$(Q)$(OBJCOPY) -Obinary $(BINARY) $(BINARY).bin
	@printf "  OBJCOPY $(BINARY).hex\n"
	$(Q)$(OBJCOPY) -Oihex $(BINARY) $(BINARY).hex
	$(Q)$(SIZE) $(BINARY)

directories: ${OUT_DIR}

${OUT_DIR}:
	$(Q)${MKDIR_P} ${OUT_DIR}

$(BINARY): $(OBJS) $(LDSCRIPT)
	@printf "  LD      $(subst $(shell pwd)/,,$(@))\n"
	$(Q)$(LD) $(LDFLAGS) -o $(BINARY) $(OBJS) -lopencm3_stm32f1

-include $(DEPENDS)

$(OUT_DIR)/%.o: %.c Makefile
	@printf "  CC      $(subst $(shell pwd)/,,$(@))\n"
	$(Q)$(CC) $(CFLAGS) -MMD -MP -o $@ -c $<

$(OUT_DIR)/%.o: %.cpp Makefile
	@printf "  CPP     $(subst $(shell pwd)/,,$(@))\n"
	$(Q)$(CPP) $(CPPFLAGS) -MMD -MP -o $@ -c $<

clean:
	@printf "  CLEAN   ${OUT_DIR}\n"
	$(Q)rm -rf ${OUT_DIR}
	@printf "  CLEAN   $(BINARY)\n"
	$(Q)rm -f $(BINARY)
	@printf "  CLEAN   $(BINARY).bin\n"
	$(Q)rm -f $(BINARY).bin
	@printf "  CLEAN   $(BINARY).hex\n"
	$(Q)rm -f $(BINARY).hex
	@printf "  CLEAN   $(BINARY).srec\n"
	$(Q)rm -f $(BINARY).srec
	@printf "  CLEAN   $(BINARY).list\n"
	$(Q)rm -f $(BINARY).list

flash: images
	@printf "  FLASH   $(BINARY).bin\n"
	@# IMPORTANT: Don't use "resume", only "reset" will work correctly!
	$(Q)$(OPENOCD) -s $(OPENOCD_SCRIPTS) \
		       -f $(OPENOCD_FLASHER) \
		       -f $(OPENOCD_BOARD) \
		       -c "init" -c "reset halt" \
		       -c "flash write_image erase $(BINARY).hex" \
		       -c "reset" \
		       -c "shutdown" $(NULL)

.PHONY: directories images clean

get-deps:
	@printf "  GIT SUBMODULE\n"
	$(Q)git submodule update --init
	@printf "  MAKE libopencm3\n"
	$(Q)${MAKE} -C libopencm3

Test:
	g++ -c -Ilibopeninv/include/ -o my_fp.o libopeninv/src/my_fp.c
	g++ -c -Ilibopeninv/include/ -o my_string.o libopeninv/src/my_string.c
	g++ -c -Ilibopeninv/include/ -Iinclude -o params.o libopeninv/src/params.cpp
	g++ -c -Ilibopeninv/include/ -Iinclude -o errormessage.o libopeninv/src/errormessage.cpp
	g++ -c -Ilibopeninv/include/ -Iinclude -Wno-builtin-declaration-mismatch -Wno-int-to-pointer-cast -o printf.o libopeninv/src/printf.cpp
	g++ -c -Ilibopeninv/include/ -Iinclude -o picontroller.o libopeninv/src/picontroller.cpp
	g++ -c -Ilibopeninv/include/ -Iinclude -o charger.o src/charger.cpp -D TEST_COMMON_H
	g++ -c -Ilibopeninv/include/ -Iinclude -o digio_mock.o test/digio_mock.cpp
	g++ -c -Ilibopeninv/include/ -Iinclude -o timer_mock.o test/timer_mock.cpp
	ar rcs libopeninv/libopeninv.a *.o
	rm -f my_fp.o my_string.o params.o errormessage.o printf.o digio_mock.o timer_mock.o terminal.o
	g++ -gdwarf-2 -g test/test.c -Llibopeninv -Iinclude -lopeninv -o test/test.out && gdb -ex='set confirm on' -ex=run -ex=quit --args ./test/test.out
cleanTest:
	cd test && $(MAKE) clean
