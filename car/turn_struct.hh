#include <avr/io.h>

typedef struct turn_factor{
    uint8_t value:7;
    uint8_t left:1;
};