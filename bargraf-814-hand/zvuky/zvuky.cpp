#include <avr/io.h>
#include <math.h>

#define F_CPU	(20000000UL/6UL)
#include <util/delay.h>

#define REPRACEK_port PORTA
#define REPRACEK_bm   8

#define KOEF_pulton	pow(2, 1.0/12.0)
// A 440 Hz

void pip(char ton, uint8_t cas) { // ton, cas [0,1 s]
	float fr;
	switch (ton) {
		case 'a':
			fr = 440.0;
			break;
		case 'A':
			fr = 440.0 * KOEF_pulton;
			break;
		case 'h':
			fr = 440.0 * pow(KOEF_pulton, 2.0);
			break;
		case 'c':
			fr = 440.0 * pow(KOEF_pulton, 3.0);
			break;
		case 'C':
			fr = 440.0 * pow(KOEF_pulton, 4.0);
			break;		
		case 'd':
			fr = 440.0 * pow(KOEF_pulton, 5.0);
			break;
		case 'D':
			fr = 440.0 * pow(KOEF_pulton, 6.0);
			break;
		case 'e':
			fr = 440.0 * pow(KOEF_pulton, 7.0);
			break;
		case 'f':
			fr = 440.0 * pow(KOEF_pulton, 8.0);
			break;
		case 'F':
			fr = 440.0 * pow(KOEF_pulton, 9.0);
			break;
		case 'g':
			fr = 440.0 * pow(KOEF_pulton, 10.0);
			break;
		case 'G':
			fr = 440.0 * pow(KOEF_pulton, 11.0);
			break;
	}
	uint8_t t = static_cast<uint8_t>(100000.0/fr);
	
	
	
		REPRACEK_port.OUTSET = REPRACEK_bm;
		for(uint8_t n=0; n<t ; n++) _delay_us(5);
		REPRACEK_port.OUTCLR = REPRACEK_bm;
		for(uint8_t n=0; n<t ; n++) _delay_us(5);
	
	
}

int main(void) {
	PORTA.DIR = 0b11111010;
	PORTA_PIN0CTRL = PORT_PULLUPEN_bm;
	PORTB.DIR = 7;
	PORTB_PIN3CTRL = PORT_ISC_INTDISABLE_gc;
	
	while (1) {
		
		//_delay_ms(1);
		_delay_us(275);
		REPRACEK_port.OUTTGL = REPRACEK_bm;
		
	/*	// blikat LEDkama
		PORTA.OUTTGL = 255-15;
		PORTB.OUTTGL = 7;
		_delay_ms(100);
		// cvakat s reprackem
		PORTA.OUTSET = 8;
		_delay_ms(10);
		PORTA.OUTCLR = 8; */
	}
}

