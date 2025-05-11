#include <avr/io.h>
#include <avr/interrupt.h>

#define F_CPU	(20000000UL)
#include <util/delay.h>

#define PIP_port	PORTB
#define PIP_bm		(1<<2)
#define OUT_port	PORTA
#define OUT_bp		(4)

int main(void) {
	_PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, 0);		// nedìlit f CPU, bude 20 MHz
	
	PORTA.DIR = 0b11110010;
	PORTA_PIN0CTRL = PORT_PULLUPEN_bm;
	PORTA_PIN2CTRL = PORT_PULLUPEN_bm;
	PORTA_PIN3CTRL = PORT_PULLUPEN_bm;
	
	PORTB.DIR = 0 + PIP_bm;
	PORTB_PIN0CTRL = PORT_ISC_INTDISABLE_gc;
	PORTB_PIN1CTRL = PORT_ISC_INTDISABLE_gc;
	PORTB_PIN3CTRL = PORT_PULLUPEN_bm;

	TCA0_SINGLE_CTRLA = TCA_SINGLE_CLKSEL_DIV1_gc;		// použít pøímo frekvenci procesoru (dìleno 1)
	TCA0_SINGLE_CTRLB = TCA_SINGLE_WGMODE_NORMAL_gc;	// NORMAL - jen èítat
	TCA0_SINGLE_PER = F_CPU / 1000UL;					// perioda èítání bude tisícina F_CPU
	TCA0_SINGLE_INTCTRL = TCA_SINGLE_OVF_bm;			// pøerušení od pøeteèení periody
	TCA0_SINGLE_INTFLAGS = TCA_SINGLE_OVF_bm;			// smazat pøípadné pøedchozí visící pøerušení
	TCA0_SINGLE_CTRLA |= TCA_SINGLE_ENABLE_bm;			// povolit èítaè

	__asm__("sei");

    while (1) {
		_delay_ms(200);
		OUT_port.OUTTGL = 1<<OUT_bp;
		
    }
}

ISR (TCA0_OVF_vect) { // 1 ms =============================
	OUT_port.OUTSET = 2<<OUT_bp;
	OUT_port.OUTTGL = 4<<OUT_bp;
	PIP_port.OUTTGL = PIP_bm;
	
	TCA0_SINGLE_INTFLAGS = 1;
}