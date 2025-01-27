#include <avr/io.h>

#define F_CPU	(20000000UL/6UL)
#include <util/delay.h>

int main(void) {
	PORTA.DIR = 0b11111010;
	PORTA_PIN0CTRL = PORT_PULLUPEN_bm;
	PORTB.DIR = 7;
	PORTB_PIN3CTRL = PORT_ISC_INTDISABLE_gc;
	
	int pip = 0;
	
    while (1) {
		// blikat s LEDkama
		PORTA.OUTTGL = 255-15;
		PORTB.OUTTGL = 7;
		_delay_ms(100);
		// pipani, ale jen 10 sekund
		if (pip<100*50) for (uint8_t n=0; n<50; n++) {
			PORTA.OUTTGL = 8;
			_delay_us(5000/27);
			pip++;
		}
		PORTA.OUTCLR = 8;
    }
}

