Following https://github.com/dcwbrown/dwire-debug/blob/master/Manual.md#running-the-program

### Find device
```
ls -al /dev/tty.*
```

### Load and run programm in debugwire
Ensure we have a working elf file!

```
cd src; make
cd ../debug

./dwire-debug/dwdebug device tty.usbserial-AH06TDUZ
or
./dwire-debug/dwdebug
> reset

> l ../src/main.elf
```

or from terminal:
```
./dwire-debug/dwdebug l../src/main.elf,qr
```

### debugwire commands
```
> help (show all commands)
> ls   (show all available devices)
> qi   (quits in ISP mode as a fallback. We can now push code or re-set fuse via isp connector)
```

### show debug symbols
```
avr-nm blink.o
```

### start gdbserver
```
./dwire-debug/dwdebug device tty.usbserial-AH06TDUZ
> gdbserver

avr-gdb src/main.elf
target remote :4444
(gdb)
```

### gdbserver commands
(https://www.emse.fr/~lalevee/ismin/programmation_2/supports/gdb.pdf)
(https://linux.die.net/man/1/avr-gdb)

```
(gdb) break <file>:<line>      (eg verify_board.c:85)
(gdb) del <breakpoint num>     (remove breakpoint num)
(gdb) clear <file>:<line>      (remove breakpoint at file line)
(gdb) run                      (use continue if run is not working, ctrl-c to stop)
(gdb) bt                       (backtrace)
(gdb) print <expr>
(gdb) c                        (continue running programm)
(gdb) next                     (execute next line, step over any function call)
(gdb) edit [<file>:<line>]     (look at the line where it stopped)
(gdb) list 1,40                (type the text of the program where it stopped)
(gdb) step                     (execute next line, step into function call)
(gdb) jump <line>              (jump to line and run)
(gdb) info line                (info about current line)
(gdb) info source              (info about current file)
(gdb) info break               (list all breakpoints)
