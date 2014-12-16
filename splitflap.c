/*
  SplitFlap Display Controller

ATMEL ATTINY2313/4313
                    +-\/-+
  RST  (D 17) PA2  1|    |20 VCC
  RX   (D  0) PD0  2|    |19 PB7 (D 16)  SCL
  TX   (D  1) PD1  3|    |18 PB6 (D 15)  MISO
       (D  2) PA1  4|    |17 PB5 (D 14)  SDA
       (D  3) PA0  5|    |16 PB4 (D 13)* 3/4 EN
  M1B  (D  4) PD2  6|    |15 PB3 (D 12)* M2B
  IO2  (D  5) PD3  7|    |14 PB2 (D 11)* IO1
  M2A  (D  6) PD4  8|    |13 PB1 (D 10)  M1A
1/2EN *(D  7) PD5  9|    |12 PB0 (D  9)
              GND 10|    |11 PD6 (D  8)  LED
                    +----+
					
	PD  000*0*00
	PB  0000*0*0
 */

#define F_CPU 16000000UL // 16 MHz
 
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/sfr_defs.h>
#include <compat/twi.h>

#include <util/delay.h>
#include <string.h>

#include "usiTwiSlave.h"

#define output(directions,pin) (directions |= pin) // set port direction for output
#define input(directions,pin) (directions &= (~pin)) // set port direction for input

#define set(port,pin) (port |= pin) // set port pin
#define clear(port,pin) (port &= (~pin)) // clear port pin
#define pin_test(pins,pin) (pins & pin) // test for port pin
#define bit_test(byte,bit) (byte & (1 << bit)) // test for bit set

#define A1 (1 << PB1)
#define A2 (1 << PD4)
#define B1 (1 << PD2)
#define B2 (1 << PB3)

#define EN1 (1 << PD5)
#define EN2 (1 << PB4)

#define LED (1 << PD6)
#define MAG (1 << PD3)

#define on_delay() _delay_us(50) // PWM on time was 25 Chris 50
#define off_delay() _delay_us(10) // PWM off time was 5 Chris 10
#define PWM_count 125 // number of PWM cycles was 100 Chris 200

static uint8_t count;

void pulse_A1A2() {
clear(PORTD, B1);
clear(PORTB, B2);
set(PORTB, A1);
set(PORTD, A2);
	for (count = 0; count < PWM_count; ++count) {
		set(PORTB, A1);
		set(PORTD, A2);
		on_delay();
		clear(PORTB, A1);
		clear(PORTD, A2);
		off_delay();
	}
}

void pulse_A2B1() {
clear(PORTB, A1);
clear(PORTB, B2);
set(PORTD, A2);
set(PORTD, B1);
	for (count = 0; count < PWM_count; ++count) {
		set(PORTD, A2);
		set(PORTD, B1);
		on_delay();
		clear(PORTD, A2);
		clear(PORTD, B1);
		off_delay();
	}
}

void pulse_B1B2() {
	clear(PORTB, A1);
	clear(PORTD, A2);
	set(PORTD, B1);
	set(PORTB, B2);
	for (count = 0; count < PWM_count; ++count) {
		set(PORTD, B1);
		set(PORTB, B2);
		on_delay();
		clear(PORTD, B1);
		clear(PORTB, B2);
		off_delay();
	}
}

void pulse_B2A1() {
	clear(PORTD, A2);
	clear(PORTD, B1);
	set(PORTB, B2);
	set(PORTB, A1);
	for (count = 0; count < PWM_count; ++count) {
		set(PORTB, B2);
		set(PORTB, A1);
		on_delay();
		clear(PORTB, B2);
		clear(PORTB, A1);
		off_delay();
	}
}

void step_cw(int step) {
	set(PORTD, EN1);
	set(PORTB, EN2);
	if (step == 0)
		pulse_A1A2();
	if (step == 1)
		pulse_A2B1();
	if (step == 2)
		pulse_B1B2();
	if (step == 3)
		pulse_B2A1();
	clear(PORTD, EN1);
	clear(PORTB, EN2);
}

//void step_ccw() {
//	pulse_B2A1();
//	pulse_B1B2();
//	pulse_A2B1();
//	pulse_A1A2();
//}
   
int main(void)
{
	//set led high
	output(DDRD, LED);
	set(PORTD, LED);

	//set motor enable pins high
	clear(PORTD, EN1);
	output(DDRD, EN1);
	clear(PORTB, EN2);
	output(DDRB, EN2);
	clear(PORTB, A1);
	output(DDRB, A1);
	clear(PORTD, A2);
	output(DDRD, A2);
	clear(PORTD, B1);
	output(DDRD, B1);
	clear(PORTB, B2);
	output(DDRB, B2);

	const uint8_t slaveAddress = 0x04;
	usiTwiSlaveInit(slaveAddress);	
	
	//enable interrupts
	sei();
	
	//setup magnet input
	input(DDRD, MAG);
	
	int currStep = 0;
	uint8_t temp_var = 0;
	while(1)
	{
		if(usiTwiDataInReceiveBuffer()) {
			temp_var = usiTwiReceiveByte();
		}
		int i = 0;
		set(PORTD, LED);
		
		//do homing routine
		if (temp_var > 50) {
			clear(PORTD, LED);
			while(bit_test(PIND, MAG) {
				for(i = 0; i < 4; i++) {
					step_cw(i);
				}
			}
			set(PORTD, LED);
		} 
		//otherwise move that number of steps
		else {
			for(i = 0; i < temp_var; i++) {
				currStep = (++currStep) % 4;
				step_cw(currStep);
			}
		}
		temp_var = 0;
	
		//if (PIND & (1<<PD3)) {
		//	PORTD |= (1<<PD6);
		//	step_cw();
		//}
		//else {
		//	PORTD &= ~(1<<PD6);
		//	step_ccw();
		//}
	}
	return 0;
}

