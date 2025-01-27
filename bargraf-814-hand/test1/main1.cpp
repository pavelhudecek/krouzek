#include <avr/io.h>

#define F_CPU	(20000000UL/6UL)
#include <util/delay.h>

int main(void) {
	PORTA.DIR = 0b11111010;
	PORTA_PIN0CTRL = PORT_PULLUPEN_bm;
	PORTB.DIR = 7;
	PORTB_PIN3CTRL = PORT_ISC_INTDISABLE_gc;
	
    while (1) {
		// blikat LEDkama
		PORTA.OUTTGL = 255-15;
		PORTB.OUTTGL = 7;
		_delay_ms(100);
		// cvakat s reprackem
		PORTA.OUTSET = 8;
		_delay_ms(10);
		PORTA.OUTCLR = 8;
    }
}

