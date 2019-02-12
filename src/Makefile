TARGET = main
DEVICE = attiny84
CLOCK  = 4000000

CC     = avr-gcc
CFLAGS  = -ggdb -DF_CPU=$(CLOCK) -mmcu=$(DEVICE) -Os -Wall -Werror -Wextra -I $(INC)/
OBJCOPY = avr-objcopy
AVRDUDE = avrdude -c $(PROGRAMMER) -p $(DEVICE)

# directories
SRC = .
# INC = ../lib/uart
INC = .

# device parameters
PROGRAMMER = usbasp
LFUSE      = 0xfc
HFUSE      = 0x9f
EFUSE      = 0xff

#--------------------------------------
# No need to edit the below code
#--------------------------------------

# source files and object directory
SOURCES = $(wildcard $(SRC)/*.c) $(wildcard $(INC)/*.c)
OBJECTS = $(patsubst %.c, %.o, $(SOURCES))

# get file dependencies then build hex
all: Makefile.deps $(TARGET).hex

# include object dependencies
-include Makefile.deps

$(TARGET).elf: $(OBJECTS)
	$(CC) $(CFLAGS) -o $@ $^

$(TARGET).hex: $(TARGET).elf
	$(OBJCOPY) -j .text -j .data -O ihex $< $@

# generate object dependencies
Makefile.deps: $(SOURCES)
	$(CC) $(CFLAGS) -MM $^ > Makefile.deps

# remove build files
clean:
	-@rm -rf $(SRC)/*.o $(INC)/*.o $(TARGET).elf $(TARGET).hex Makefile.deps 2>/dev/null || true

# flash to micro-controller
flash: $(TARGET).hex
	$(AVRDUDE) -U flash:w:$<

# set fuses
fuses:
	$(AVRDUDE) -U lfuse:w:$(LFUSE):m -U hfuse:w:$(HFUSE):m -U efuse:w:$(EFUSE):m

# show fuses already set on micro-controller
show_fuses:
	@$(AVRDUDE) -nv 2>&1 |grep Fuses

# temporarily switch to ISP mode (if dwen = 1)
isp:
	@(../debug/dwire-debug/dwdebug u0 1,qi 2>&1 >/dev/null && echo "temporarily switched to ISP mode. You may flash or update fuses now") || true

# run cmd "reset" to connect to device
dwdebug:
	@../debug/dwire-debug/dwdebug
