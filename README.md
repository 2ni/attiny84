# What to find here
Some tryout with ATtiny84
- TWI with USI (Slave)
- ADC measurements
- based on https://github.com/Miceuz/i2c-moisture-sensor

# Installation
```
pio init --board attiny84
```

# Usage
http://docs.platformio.org/en/latest/platforms/atmelavr.html#upload-using-programmer

```
pio run -t program
```

# Fuses
```
CKDIV8    : 0 (no division)
CKOUT     : 0 (clk on portb2)
CKSEL[3:1]: 110 (3-8Mhz)
CKSEL0    : 0
SUT[1:0]  : 11 (ceramic resonator, fast rising power)
L: 0xBC, H: 0xDF, E: 0xFF -> avrdude -c usbasp -p t84 -P usb -U lfuse:w:0xbc:m
(Defaults L: 0x62, H: 0xDF, E: 0xFF)
```

# Pins
```
		DDR	PORT
tri-state	0	0
pullup in	0	1
out		1	0/1
```

# Background information
- https://wemakethings.net/2012/09/26/capacitance_measurement/
- https://wemakethings.net/chirp/
- https://github.com/Miceuz/i2c-moisture-sensor/
- https://nathan.chantrell.net/20120225/an-attiny-based-wireless-temperature-sensor/

# I2C
- (doesn't work) https://www.instructables.com/id/ATTiny-USI-I2C-The-detailed-in-depth-and-infor/
- tinywires: https://github.com/DzikuVx/attiny_photoresistor_i2c, https://github.com/rambo/TinyWire/tree/rollback/TinyWireS/examples
- avr slave: http://pc.gameactive.org/i2cslavetutorial/I2C_SlaveTutorial.html



# AVR libs
/usr/local/Cellar/avr-gcc/8.2.0/avr/include/avr

# Install avr-gcc
https://github.com/osx-cross/homebrew-avr
```
brew tap osx-cross/avr
brew install avr-gcc
```

# Fix the USBAsp
Solution: For some reasons attiny84 needs 5V power to flash
(communication won't work with 3.3v)


### More background reasearch
- http://irq5.io/2017/07/25/making-usbasp-chinese-clones-usable/
- https://stackoverflow.com/questions/25591406/how-to-make-mac-detect-avr-board-using-usbasp-and-burn-program-to-it

Use slower clock on blank AVRs as it uses slow internal clock 1MHz: Set JP3 on programmer!
- https://tosiek.pl/usbasp-v2-0-warning-cannot-set-sck-period/
- https://forum.arduino.cc/index.php?topic=149668.0

```
Once the AVR has been programmed to use a faster external crystal, (which requires changing fuses)
the USBasp slow SCK jumper can be removed.
Future re-programming of that AVR  can now be done faster
and the SCK error/warning can be ignored since the AVR is now running with a faster
external crystal and the faster SCK will not create any issues since the AVR can keep up
with it.
```
