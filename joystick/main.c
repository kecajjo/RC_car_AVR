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

volatile uint8_t x_pos_raw = 0;
volatile uint8_t y_pos_raw = 0;

//joystick on PC0 and PC1
//USART on PD1

void init_USART(){
    //set baud rate
    UBRR0L = (uint8_t)UBRRVAL;//low byte
    UBRR0H = (UBRRVAL>>8);//high byte
    //frame format: asynchronous mode,no parity, 2 stop bits, 8 bit size
    UCSR0C = (1<<UCSZ01)|(1<<UCSZ00)|(1<<USBS0); 
    //only transmitter
    UCSR0B = (1<<TXEN0);
}

void init_ADC(){
    //referance voltage as AVCC with capacitor to AREF
    //8bit precision is enough, reading data from ADCH
    ADMUX &= ~(1<<REFS1);
    ADMUX |= (1<<REFS0) | (1<<ADLAR);
    //enable ADC
    ADCSRA |= (1<<ADEN);
}

//channel must be between 0 and 8
uint8_t read_ADC(uint8_t channel){
    ADMUX = (ADMUX & 0xf0) | channel;
    ADCSRA |= (1<<ADSC); // start conversion
    while(ADCSRA & (1<<ADSC)); // wait for conversion to end
    return(ADCH); // return ADC with 8bit resolution
}

void send_byte(uint8_t data){
    // wait if a byte is being transmitted
    while((UCSR0A & (1<<UDRE0)) == 0);
    // transmit data
    UDR0 = data;  
}

void send_packet(uint8_t addr, uint8_t b1, uint8_t b2){
    uint16_t addr_en, b1_en, b2_en;
    addr_en = manchester_encode_byte(addr);
    b1_en = manchester_encode_byte(b1);
    b2_en = manchester_encode_byte(b2);
    send_byte(SYNC); //synchronisation signal
    send_byte(addr_en>>8);
    send_byte(addr_en);
    send_byte(b1_en>>8);
    send_byte(b1_en);
    send_byte(b2_en>>8);
    send_byte(b2_en);
    //send_byte(0xFF-(uint8_t)(addr+b1+b2)); //checksum
}

int main(){
    DDRC = 0x00; // full C as input
    DDRD = 0xff; // full D as output

    cli();
    init_ADC();
    init_USART();
    sei();

    while(1){
        //read joystick pos
        x_pos_raw = read_ADC(0);
        y_pos_raw = read_ADC(1);
        //send position via USART
        send_packet(RADDR, x_pos_raw, y_pos_raw);
        _delay_ms(50);
    }

    return 0;
}