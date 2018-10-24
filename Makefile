.PHONY: compile upload check fuse

compile:
	pio run

flash:
	pio run -t program

# see https://stackoverflow.com/questions/25591406/how-to-make-mac-detect-avr-board-using-usbasp-and-burn-program-to-it
check:
	avrdude -c usbasp -p t84 -P usb -v

# No div8, clk to portb2, ceramic resonatr fast rising, 3-8MHz
fuseckout:
	avrdude -c usbasp -p t84 -P usb -U lfuse:w:0xbc:m

# No div8, ceramic resonatr fast rising, 3-8MHz
fuse:
	avrdude -c usbasp -p t84 -P usb -U lfuse:w:0xfc:m

debug:
	@echo "quit with: ctrl-a; ctrl-$$ "
	@sleep 1
	@screen /dev/cu.usbserial-AH06TDUZ
