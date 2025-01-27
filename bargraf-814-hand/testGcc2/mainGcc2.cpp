// https://github.com/ZakKemble/avr-gcc-build/releases/tag/v14.1.0-1
// C:\Program Files (x86)\Atmel\GCC\avr-gcc-14.1.0-x64-windows\bin\avr-g++.exe
// <ToolchainFlavour>gcc1410</ToolchainFlavour>
// -fexceptions
#include <avr/io.h>
#include <avr/interrupt.h>

#define F_CPU	(20000000UL)
#include <util/delay.h>

#define PIP_port	PORTA
#define PIP_bm		(1<<3)

#define LED_03port	PORTA
#define LED_46port	PORTB
#define LED_0bp		4 // 4567
#define LED_4bp		2 // 210

uint8_t ledky[7] = {1, 2, 3, 4, 5, 6, 7};
bool	pipat = false;
constexpr auto cLedValsCnt = 8;
uint8_t ledVals[cLedValsCnt] = {0, 1, 3, 9, 27, 54, 108, 216};
//uint8_t ledVals[cLedValsCnt] = {0, 1, 2, 4, 8, 16, 32, 64};


void pip(uint16_t t, int rep) { // ===========================================
	PIP_port.OUTSET = PIP_bm;
	for (int r=0; r<rep; r++) {
		pipat = true;
		for (uint16_t n=0; n<t; n++) _delay_ms(1);
		pipat = false;
		for (uint16_t n=0; n<t; n++) _delay_ms(1);
	}
	PIP_port.OUTCLR = PIP_bm;
}

template <typename T> void ledSet1(uint8_t idx, T arg) { // ========
	if (idx<4) {
		if (arg==0) LED_03port.OUTCLR = 1<<(LED_0bp+idx);
		else		LED_03port.OUTSET = 1<<(LED_0bp+idx);
	} else {
		if (arg==0) LED_46port.OUTCLR = 1<<(LED_4bp-(idx-4));
		else		LED_46port.OUTSET = 1<<(LED_4bp-(idx-4));
	}
}
/*template <class ...C> void test(C ... args) { // ====================
	//try {
	uint8_t err = 0;
	do {
		uint8_t siz = sizeof...(C);
		//if (siz>7) throw(1);
		if (siz>7) {err=1; break;}
		uint8_t res[siz]={args...};
	
		for (uint8_t n=0; n<7; n++) {
			if (n>=siz) {
				ledky[n] = 0;
			} else {
				//if (res[n]>=cLedValsCnt) throw(2);
				if (res[n]>=cLedValsCnt) {err=2; break;}
				ledky[n] = ledVals[res[n]];
			}
		}
		if (err) break;
	//} catch (int err) {
	} while(0);
	pip(50, err);
	//}
}*/

template <class ...C> void test(C ... args) { // ====================
	uint8_t siz = sizeof...(C);
	uint8_t res[siz]={args...};
		
	uint8_t err = [&]() {
		if (siz>7) return(1);
		
		for (uint8_t n=0; n<7; n++) {
			if (n>=siz) {
				ledky[n] = 0;
			} else {
				if (res[n]>=cLedValsCnt) return(2);
				ledky[n] = ledVals[res[n]];
			}
		}
		return 0;
	}();
	
	pip(50, err);
}

int main(void) { // #################################################
	_PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, 0);	// nedelit
	
	PORTA.DIR = 0b11110000;				// LED0-3
	PORTA_PIN0CTRL = PORT_PULLUPEN_bm;	// A0/UPDI
	PORTB.DIR = 7;						// LED6-4
	
	TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV1_gc;	 // nedelit
	TCA0.SINGLE.CTRLB = TCA_SINGLE_WGMODE_NORMAL_gc; // FRQ by bylo lepsi, ale nemame WO<3
	TCA0.SINGLE.INTCTRL = TCA_SINGLE_OVF_bm;		 // pøerušení od pøeteèení periody
	TCA0.SINGLE.PER = F_CPU / 12000UL;				 // nastavení periody 12 kHz
	TCA0.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm;		 // ukonèit pøerušení, pokud nìjaké už bylo
	TCA0.SINGLE.CTRLA |= TCA_SINGLE_ENABLE_bm;		 // povolit timer
	
	__asm("sei");
	
	while (1) {
		_delay_ms(1000);
		test(1, 2, 3, 4, 5, 6, 7);
		_delay_ms(1000);
		test(7, 6, 5);
		_delay_ms(1000);
		test(7, 6, 5, 4, 3, 2, 1);
		pip(100,100);
		//_delay_ms(1000);
		//test(7, 6, 5, 4, 3, 2, 1, 5);
		//_delay_ms(1000);
		//test(7, 6, 5, 4, 3, 2, 15);
		//int i = static_cast<int>([]() {return 5;});
		
		//pip(50, []() { return 4; }());
	}
}

ISR (TCA0_OVF_vect) { // ============================================
	static uint8_t jcnt = 0;
		
	if (++jcnt>ledVals[cLedValsCnt-1]) jcnt=0;
	for (uint8_t n=0; n<7; n++) ledSet1(n, (jcnt<ledky[n]) ? 1 : 0);
	
	if (pipat) {
		if (!(jcnt & 4)) PIP_port.DIRTGL = PIP_bm;
	} else {
		PIP_port.DIRCLR = PIP_bm;
	}

	TCA0.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm;
}