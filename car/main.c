#define __AVR_ATmega328P__
#define F_CPU 8000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "manchester.hh"
#include "turn_struct.hh"

//set desired baud rate
#define BAUDRATE 1200
//calculate UBRR value
#define UBRRVAL ((F_CPU/(BAUDRATE*16UL))-1)
//define receive parameters
#define SYNC 0X55// synchro signal
#define RADDR 0x44

//frame structure(in bytes, manchester encoded): 1 sync signal, 2 addr, 2 pos_x, 2 pos_y
#define FRAME_SIZE 6//frame size in bytes without sync byte

#define MAX_LOOPS 20//how many times program can loop without new information before PWM duty cycle will be set to 0
#define TURN_DIV 2//bigger number==slower turning speed

volatile uint8_t data[FRAME_SIZE];
volatile uint8_t i = 0;
volatile uint8_t sync_byte = 0;
volatile uint8_t dat_buf = 0;//data buffer
volatile uint32_t no_new_data = 0;
volatile uint8_t motor_command[2];
volatile struct turn_factor turn;

// AIN1 PB1 - AIN2 PB2 - BIN1 PB3 - BIN2 PB4
// PWMA PD6 - PWMB PD5
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
    DDRB = 0xFF;
    PORTB = 0x00; // low everywhere, no short circuit on H bridge

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
        // if all decoded symbols were known
        if(no_new_data<MAX_LOOPS){
            if(!(flags & (1<<MANCH_ERR))){
                if(encoded_data[0] == RADDR){
                    no_new_data = 0;

                    // getting joystick y_pos to each motor command
                    motor_command[0] = encoded_data[2];
                    motor_command[1] = encoded_data[2];

                    if(encoded_data[1] > 131){// if joystick x_pos is on the left
                        turn.value = encoded_data[1] - 128;
                        turn.left = 1;
                    } else if(encoded_data[1] < 124){// if joystick x_pos is on the right
                        turn.value = 127 - encoded_data[1];
                        turn.left = 0;
                    }

                    // if turn.left is 1 we decrease command for motor 0 and increase for motor 1
                    // if turn is 0 we do opposite (to turn right)
                    if(encoded_data[2] > turn.value/TURN_DIV){//subtraction cant be under 0
                        motor_command[(turn.left+1)%2] -= turn.value/TURN_DIV;
                    } else{
                        motor_command[(turn.left+1)%2] = 0;
                    }
                    if(encoded_data[2] < encoded_data[2] + turn.value/TURN_DIV){//sum cant be over 255
                        motor_command[turn.left] += turn.value/TURN_DIV;
                    } else{
                        motor_command[turn.left] = 255;
                    }

                    if(motor_command[0] > 145){
                        OCR0A = (motor_command[0]-128)*2;
                        PORTB &= ~(1<<PB2);
                        PORTB |= (1<<PB1);
                    } else if(motor_command[0] < 115){
                        OCR0A = (127-motor_command[0])*2;
                        PORTB &= ~(1<<PB1);
                        PORTB |= (1<<PB2);
                    }else{
                        PORTB &= ~((1<<PB2)|(1<<PB1));
                        OCR0A = 0;
                    }
                    if(motor_command[1] > 145){
                        OCR0B = (motor_command[1]-128)*2;
                        PORTB &= ~(1<<PB4);
                        PORTB |= (1<<PB3);
                    } else if(motor_command[1] < 115){
                        OCR0B = (127-motor_command[1])*2;
                        PORTB &= ~(1<<PB3);
                        PORTB |= (1<<PB4);
                    }else{
                        PORTB &= ~((1<<PB3)|(1<<PB4));
                        OCR0B = 0;
                    }
                }
            }
        } else{
            OCR0A = 0;
            PORTB &= ~((1<<PB3)|(1<<PB4));
            PORTB &= ~((1<<PB2)|(1<<PB1));
            OCR0B = 0;
        }
        flags &= ~(1<<MANCH_ERR); //clear manchester error flag
        ++no_new_data;
        _delay_ms(63);//different delay than in joystick
    }

    return 0;
}