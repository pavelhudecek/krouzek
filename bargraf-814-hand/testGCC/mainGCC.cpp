// https://github.com/ZakKemble/avr-gcc-build/releases/tag/v14.1.0-1
// C:\Program Files (x86)\Atmel\GCC\avr-gcc-14.1.0-x64-windows\bin\avr-g++.exe
// <ToolchainFlavour>gcc1410</ToolchainFlavour>

#include <avr/io.h>

#define F_CPU	(20000000UL)
#include <util/delay.h>

#define LED_03port	PORTA
#define LED_46port	PORTB
#define LED_0bp		4 // 4567
#define LED_4bp		2 // 210

template <typename T> void ledSet1(uint8_t idx, T arg) {
    if (idx<4) {
        if (arg==0) LED_03port.OUTCLR = 1<<(LED_0bp+idx);
        else        LED_03port.OUTSET = 1<<(LED_0bp+idx);
    } else {
        if (arg==0) LED_46port.OUTCLR = 1<<(LED_4bp-(idx-4));
        else        LED_46port.OUTSET = 1<<(LED_4bp-(idx-4));
    }
}
template <class ...C> void test(C ... args) {
    uint8_t siz = sizeof...(C);
    uint8_t res[siz]={args...};

    for (uint8_t n=0; n<7; n++) {
        if (n>=siz) ledSet1(n, 0);
        else        ledSet1(n, res[n]);
    }
} 

int main(void) {
	_PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, 0);	// nedelit
	
	PORTA.DIR = 0b11110000;				// LED0-3
	PORTA_PIN0CTRL = PORT_PULLUPEN_bm;	// A0/UPDI
	PORTB.DIR = 7;						// LED6-4
	
	while (1) {
		test(1, 0, 1, 0, 1, 0, 1);
		_delay_ms(1000);
		test(0, 1, 0, 1, 0, 1);
		_delay_ms(1000);
		test(1, 1, 1);
		_delay_ms(1000);
	}
}