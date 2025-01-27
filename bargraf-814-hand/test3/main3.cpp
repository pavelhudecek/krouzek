#include <avr/io.h>
#include <avr/interrupt.h>

#define F_CPU	(20000000UL)
#include <util/delay.h>

#define PIP_port	PORTA
#define PIP_bm		(1<<3)


int main(void) {
	_PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, 0);				 // nedelit
	
	PORTA.DIR = 0b11111010;
	PORTA_PIN0CTRL = PORT_PULLUPEN_bm;
	PORTB.DIR = 7;
	PORTB_PIN3CTRL = PORT_ISC_INTDISABLE_gc;
	
	TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV1_gc;
	TCA0.SINGLE.CTRLB = TCA_SINGLE_WGMODE_NORMAL_gc; // FRQ by bylo lepsi, ale nemame WO<3
	TCA0.SINGLE.INTCTRL = TCA_SINGLE_OVF_bm;
	TCA0.SINGLE.PER = F_CPU / 3000UL;
	TCA0.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm;
	TCA0.SINGLE.CTRLA |= TCA_SINGLE_ENABLE_bm;
	
	PIP_port.OUTSET = PIP_bm;
	
	__asm("sei");
	
    while (1) {
		// blikat s LEDkama
		PORTA.OUTTGL = 255-15;
		PORTB.OUTTGL = 7;
		_delay_ms(5);

		// plynule ladeni zvuku a konec az budou MHz
		if (TCA0.SINGLE.PER>2) TCA0.SINGLE.PER--;
		else PIP_port.OUTCLR = PIP_bm;
    }
}

ISR (TCA0_OVF_vect) {
	PIP_port.DIRTGL = PIP_bm;
	TCA0.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm;
}