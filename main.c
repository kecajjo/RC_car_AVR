#include "manchester.hh"
#include <avr/io.h>

int main(){
    uint8_t cos = 0b10001001;
    uint16_t cos_zakodowane;

    cos_zakodowane = manchester_encode_byte(cos);
    cos = manchester_decode_byte(cos_zakodowane);

    return 0;
}