Following https://github.com/dcwbrown/dwire-debug/blob/master/Manual.md#running-the-program

### Find device
```
ls -al /dev/tty.*
```

### Run
```
./dwire-debug/dwdebug ls                            - list available devices
./dwire-debug/dwdebug device tty.usbserial-AH06TDUZ - connect to device
```

### Load programm
? use -gstabs parameter on the avr-as command

```
cd src; make
./dwire-debug/dwdebug device tty.usbserial-AH06TDUZ
> l src/main.elf
```
fails so far...

### Fallback to ISP
> qi (quits in ISP mode. Push code set fuse via isp connector)

### show debug symbols
```
avr-nm blink.o
```

### gdbserver
```
./dwire-debug/dwdebug device tty.usbserial-AH06TDUZ
> gdbserver

avr-gdb src/main.elf
target remote :4444
```
