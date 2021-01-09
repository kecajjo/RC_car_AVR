#include "flags.hh"

// function encodes 1byte variable (2bytes result cose of manchester encoding)
uint16_t manchester_encode_byte(uint8_t byte){
    uint16_t encoded = 0x00;
    uint8_t i;
    for(i=0;i<8;i++){
        // changes 0 to 01 and 1 to 10
        encoded |= ( (byte & (1 << i)) ? (1 << 2*i+1) : (1 << 2*i) );
    }
    return encoded;
}

void manchester_encode(uint8_t data[], uint8_t size, uint8_t encoded[]){
    uint8_t i;
    for(i=0;i<size;i++){
        encoded[i] = manchester_encode_byte(data[i]);
    }
}

/* slower
uint8_t manchester_decode_byte(uint16_t encoded){
    uint8_t decoded = 0;
    uint8_t tmp;
    uint8_t i;
    for (i=0;i<8;i++){
        tmp = (encoded & (1 << 2*i | 1 << 2*i+1)) >> 2*i;
        switch(tmp){
        case 1: // 01 means 0 so dont do anything
            break;
        case 2: // 10 means 1
            decoded |= (1<<i);
            break;
        default:
            flags |= (1<<MANCH_ERR); // other symbols shouldnt exist
        }
    }
    return decoded;
}*/

uint8_t manchester_decode_byte(uint16_t encoded){
    uint16_t en_shifted = encoded << 1;
    en_shifted &= 0xAAAA; //every 2nd bit is important
    encoded &= 0xAAAA; //every 2nd bit is important
    // checking if each pair of bits was 01 or 10
    en_shifted ^= encoded;
    if(en_shifted != 0xAAAA){
        flags |= 1<<MANCH_ERR;
        return 0xFF;
    }
    //reading every 2nd bit by shifting them left
    encoded += encoded & 0x2AAA;
    encoded += encoded & 0x1554;
    encoded += encoded & 0x0AA8;
    encoded += encoded & 0x0550;
    encoded += encoded & 0x02A0;
    encoded += encoded & 0x0140;
    encoded += encoded & 0x0080;

    return (uint8_t)(encoded >> 8);
}

void manchester_decode(uint16_t encoded[], uint8_t size, uint8_t data[]){
    uint8_t i;
    for(i=0;i<size;i++){
        data[i] = manchester_decode_byte(encoded[i]);
    }
}