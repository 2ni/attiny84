.PHONY: compile upload check fuse

compile:
	pio run

flash:
	pio run -t program

# see https://stackoverflow.com/questions/25591406/how-to-make-mac-detect-avr-board-using-usbasp-and-burn-program-to-it
check:
	avrdude -c usbasp -p t84 -P usb -v

show_fuses:
	@avrdude -c usbasp -p t84 -P usb 2>&1 |grep Fuses

# No div8, clk to portb2, ceramic resonatr fast rising, 3-8MHz
# to use for moisture sensor
fuseckout:
	avrdude -c usbasp -p t84 -P usb -U lfuse:w:0xbc:m

# No div8, ceramic resonatr fast rising, 3-8MHz
# to use for RFM69 module
fuse:
	avrdude -c usbasp -p t84 -P usb -U lfuse:w:0xfc:m -U hfuse:w:0xdf:m -U efuse:w:0xff:m

# same as fuse but with debugwire (dewen) enabled
fusedwen:
	avrdude -c usbasp -p t84 -P usb -U lfuse:w:0xfc:m -U hfuse:w:0x9f:m -U efuse:w:0xff:m

# Connect PA2 to RXD of UART. GND programmer to GND UART.
# Connect UART to computer via USB
# Connect SPI programmer to computer via additional USB
debug:
	@echo "quit with: ctrl-a; ctrl-$$ "
	@sleep 1
	@screen /dev/cu.usbserial-AH06TDUZ
