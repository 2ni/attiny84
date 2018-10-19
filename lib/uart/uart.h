/*
 * from http://www.justgeek.de/a-simple-simplex-uart-for-debugging-avrs/
 */
#ifndef __UART_H
#define __UART_H
#include <stdint.h>
#include <avr/io.h>

// Baud rate
#define UART_BPS 9600
#define UART_TX_DDR  DDRA
#define UART_TX_PORT PORTA
#define UART_TX_PIN  PA2

#define DINIT() uart_setup()
#define DCRLF() uart_puts("\r\n")
#define DL(str) { uart_puts(str); DCRLF(); }
#define DL2(name, value) { uart_puts(name); uart_puts(": "); uart_puts(value); }


extern uint8_t uart_setup(void);
extern void uart_putc(char c);
extern void uart_puts(char* s);

#endif
