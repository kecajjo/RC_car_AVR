#include <avr/io.h>

#define MANCH_ERR 0 // existance of pair of bits 11 or 00

volatile uint8_t flags = 0;