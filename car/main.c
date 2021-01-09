#define __AVR_ATmega328P__
#define F_CPU 8000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "manchester.hh"

//set desired baud rate
#define BAUDRATE 1200
//calculate UBRR value
#define UBRRVAL ((F_CPU/(BAUDRATE*16UL))-1)
//define receive parameters
#define SYNC 0X55// synchro signal
#define RADDR 0x44

//frame structure(in bytes, manchester encoded): 1 sync signal, 2 addr, 2 pos_x, 2 pos_y
#define FRAME_SIZE 6//frame size in bytes without sync byte

#define MAX_LOOPS 3//how many times program can loop without new information before PWM duty cycle will be set to 0

volatile uint8_t data[FRAME_SIZE];
volatile uint8_t i = 0;
volatile uint8_t sync_byte = 0;
volatile uint8_t dat_buf = 0;//data buffer
volatile uint32_t no_new_data = 0;

// PWM on PD6 and PD5
// USART on PD0 

void init_USART(){
    //set baud rate
    UBRR0L = (uint8_t)UBRRVAL;//low byte
    UBRR0H = (UBRRVAL>>8);//high byte
    //frame format: asynchronous mode,no parity, 2 stop bits, 8 bit size
    UCSR0C = (1<<UCSZ01)|(1<<UCSZ00)|(1<<USBS0); 
    //receiver and receiver interrupt
    UCSR0B=(1<<RXEN0)|(1<<RXCIE0);
}

ISR(USART_RX_vect){
    while((UCSR0A & (1<<RXC0)) == 0);
    dat_buf = UDR0;
    if(sync_byte == 0){
        if(dat_buf == SYNC){
            sync_byte = 1;
        }
    } else{
        data[i] = dat_buf;
        ++i;
        // if frame ended waiting for sync byte
        if(i == FRAME_SIZE){
            i = 0;
            sync_byte = 0;
            no_new_data = 0;
        }
    }
}

void init_PWM(){
    //PWM on OC0A and OC0B
    //non-inverting mode on OC0A
    TCCR0A |= (1<<COM0A1);
    TCCR0A &= ~(1<COM0A0);
    //non-inverting mode on OC0A
    TCCR0A |= (1<<COM0B1);
    TCCR0A &= ~(1<COM0B0);
    //Fast PWM
    TCCR0A |= (1<<WGM01) | (1<<WGM00);
    TCCR0B &= ~(1<<WGM02);
    //prescaler 1/64
    TCCR0B |= (1<<CS00)|(1<<CS01);
    TCCR0B &= ~(1<<CS02);
    // duty cycle
    OCR0A = 0; //duty cycle 0/256
    OCR0B = 0; //duty cycle 0/256
}

int main(){
    volatile uint8_t encoded_data[FRAME_SIZE/2]; //due to manchester encoding
    uint16_t buf[FRAME_SIZE/2];
    uint8_t j = 0;

    DDRD = 0xFE; // ports 1-7 as output

    cli();
    init_PWM();
    init_USART();
    sei();

    while(1){
        while(sync_byte);// waiting for data to be read

        // preventing data races
        cli();
        sync_byte = 0; // just in case between while loop and cli(); interrupt happened
        for(j=0;j<FRAME_SIZE/2;j++){
            buf[j] = (data[2*j]<<8) + data[2*j+1];
        }
        sei();

        //decoding
        manchester_decode(buf, FRAME_SIZE/2, encoded_data);
        // if all decoded symbols were knows
        if(no_new_data<MAX_LOOPS){
            if(!(flags & (1<<MANCH_ERR))){
                if(encoded_data[0] == RADDR){
                    if(encoded_data[1] > 131){
                        OCR0A = (encoded_data[1]-128)*2;
                    } else if(encoded_data[1] < 124){
                        OCR0A = (128-encoded_data[1])*2;
                    }else{
                        OCR0A = 0;
                    }
                    if(encoded_data[2] > 131){
                        OCR0B = (encoded_data[2]-128)*2;
                    } else if(encoded_data[2] < 124){
                        OCR0B = (128-encoded_data[2])*2;
                    }else{
                        OCR0B = 0;
                    }
                }
            }
        } else{
            OCR0A = 0;
            OCR0B = 0;
        }
        flags &= ~(1<<MANCH_ERR); //clear manchester error flag
        ++no_new_data;
        _delay_ms(63);//different delay than in joystick
    }

    return 0;
}