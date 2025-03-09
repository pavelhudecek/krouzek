#include <avr/io.h>
#include <math.h>

#define F_CPU	(20000000UL/6UL)
#include <util/delay.h>

#define REPRACEK_port PORTA
#define REPRACEK_bm   8

// index tónu (0: 440 Hz), kor k odeètení [µs] -> pùlperioda [µs]
constexpr uint16_t idxCas(const float idx, const uint8_t okt, const uint16_t kor) {
	const float cPulton = pow(2, 1.0/12.0);
	const float cCPUkor = 1.0;
	return static_cast<uint16_t>(0.5e6/(440.0 * static_cast<float>(okt) * cCPUkor * pow(cPulton, idx)));
}

void pip(char ton, uint8_t cas) { // ton, oktava 1 2 4, cas [1/16 s]: 8 = 120 BPM
	uint32_t cnt = 1000000UL * static_cast<uint32_t>(cas) / 8;
	const uint16_t cKorH = 0;
	const uint16_t cKorL = 4;
	
	switch (ton) {
		case 'a':
			cnt /= static_cast<uint32_t>(idxCas(0.0, 2, cKorH + cKorL));
			for (uint32_t n=0; n<cnt; n++) {
				REPRACEK_port.OUTSET = REPRACEK_bm;
				_delay_us(idxCas(0.0, 2, cKorH));
				REPRACEK_port.OUTCLR = REPRACEK_bm;
				_delay_us(idxCas(0.0, 2, cKorL));
			}
			break;
		case 'A':
			cnt /= static_cast<uint32_t>(idxCas(1.0, 2, cKorH + cKorL));
			for (uint32_t n=0; n<cnt; n++) {
				REPRACEK_port.OUTSET = REPRACEK_bm;
				_delay_us(idxCas(1.0, 2, cKorH));
				REPRACEK_port.OUTCLR = REPRACEK_bm;
				_delay_us(idxCas(1.0, 2, cKorL));
			}
			break;
		case 'h':
			cnt /= static_cast<uint32_t>(idxCas(2.0, 2, cKorH + cKorL));
			for (uint32_t n=0; n<cnt; n++) {
				REPRACEK_port.OUTSET = REPRACEK_bm;
				_delay_us(idxCas(2.0, 2, cKorH));
				REPRACEK_port.OUTCLR = REPRACEK_bm;
				_delay_us(idxCas(2.0, 2, cKorL));
			}
			break;
		case 'c':
			cnt /= static_cast<uint32_t>(idxCas(3.0, 1, cKorH + cKorL));
			for (uint32_t n=0; n<cnt; n++) {
				REPRACEK_port.OUTSET = REPRACEK_bm;
				_delay_us(idxCas(3.0, 1, cKorH));
				REPRACEK_port.OUTCLR = REPRACEK_bm;
				_delay_us(idxCas(3.0, 1, cKorL));
			}
			break;
		case 'C':
			cnt /= static_cast<uint32_t>(idxCas(4.0, 1, cKorH + cKorL));
			for (uint32_t n=0; n<cnt; n++) {
				REPRACEK_port.OUTSET = REPRACEK_bm;
				_delay_us(idxCas(4.0, 1, cKorH));
				REPRACEK_port.OUTCLR = REPRACEK_bm;
				_delay_us(idxCas(4.0, 1, cKorL));
			}
			break;
		case 'd':
			cnt /= static_cast<uint32_t>(idxCas(5.0, 1, cKorH + cKorL));
			for (uint32_t n=0; n<cnt; n++) {
				REPRACEK_port.OUTSET = REPRACEK_bm;
				_delay_us(idxCas(5.0, 1, cKorH));
				REPRACEK_port.OUTCLR = REPRACEK_bm;
				_delay_us(idxCas(5.0, 1, cKorL));
			}
			break;
		case 'D':
			cnt /= static_cast<uint32_t>(idxCas(6.0, 1, cKorH + cKorL));
			for (uint32_t n=0; n<cnt; n++) {
				REPRACEK_port.OUTSET = REPRACEK_bm;
				_delay_us(idxCas(6.0, 1, cKorH));
				REPRACEK_port.OUTCLR = REPRACEK_bm;
				_delay_us(idxCas(6.0, 1, cKorL));
			}
			break;
		case 'e':
			cnt /= static_cast<uint32_t>(idxCas(7.0, 1, cKorH + cKorL));
			for (uint32_t n=0; n<cnt; n++) {
				REPRACEK_port.OUTSET = REPRACEK_bm;
				_delay_us(idxCas(7.0, 1, cKorH));
				REPRACEK_port.OUTCLR = REPRACEK_bm;
				_delay_us(idxCas(7.0, 1, cKorL));
			}
			break;
		case 'f':
			cnt /= static_cast<uint32_t>(idxCas(8.0, 1, cKorH + cKorL));
			for (uint32_t n=0; n<cnt; n++) {
				REPRACEK_port.OUTSET = REPRACEK_bm;
				_delay_us(idxCas(8.0, 1, cKorH));
				REPRACEK_port.OUTCLR = REPRACEK_bm;
				_delay_us(idxCas(8.0, 1, cKorL));
			}
			break;
		case 'F':
			cnt /= static_cast<uint32_t>(idxCas(9.0, 1, cKorH + cKorL));
			for (uint32_t n=0; n<cnt; n++) {
				REPRACEK_port.OUTSET = REPRACEK_bm;
				_delay_us(idxCas(9.0, 1, cKorH));
				REPRACEK_port.OUTCLR = REPRACEK_bm;
				_delay_us(idxCas(9.0, 1, cKorL));
			}
			break;
		case 'g':
			cnt /= static_cast<uint32_t>(idxCas(10.0, 1, cKorH + cKorL));
			for (uint32_t n=0; n<cnt; n++) {
				REPRACEK_port.OUTSET = REPRACEK_bm;
				_delay_us(idxCas(10.0, 1, cKorH));
				REPRACEK_port.OUTCLR = REPRACEK_bm;
				_delay_us(idxCas(10.0, 1, cKorL));
			}
			break;
		case 'G':
			cnt /= static_cast<uint32_t>(idxCas(11.0, 1, cKorH + cKorL));
			for (uint32_t n=0; n<cnt; n++) {
				REPRACEK_port.OUTSET = REPRACEK_bm;
				_delay_us(idxCas(11.0, 1, cKorH));
				REPRACEK_port.OUTCLR = REPRACEK_bm;
				_delay_us(idxCas(11.0, 1, cKorL));
			}
			break;
		case ' ': // pauza
			cnt /= static_cast<uint32_t>(1000 - cKorH - cKorL + 2);
			for (uint32_t n=0; n<cnt; n++) {
				_delay_us(500 - cKorH + 1);
				_delay_us(500 - cKorL + 1);
			}
			break;
		case '1': // kalibrace 1 kHz
			cnt /= static_cast<uint32_t>(1000 - cKorH - cKorL);
			for (uint32_t n=0; n<cnt; n++) {
				REPRACEK_port.OUTSET = REPRACEK_bm;
				_delay_us(500 - cKorH);
				REPRACEK_port.OUTCLR = REPRACEK_bm;
				_delay_us(500 - cKorL);
			}
			break;
		case '5': // kalibrace 5 kHz
			cnt /= static_cast<uint32_t>(200 - cKorH - cKorL);
			for (uint32_t n=0; n<cnt; n++) {
				REPRACEK_port.OUTSET = REPRACEK_bm;
				_delay_us(100 - cKorH);
				REPRACEK_port.OUTCLR = REPRACEK_bm;
				_delay_us(100 - cKorL);
			}
	}
}

int main(void) { // ===========================================================
	PORTA.DIR = 0b11111010;
	PORTA_PIN0CTRL = PORT_PULLUPEN_bm;
	PORTB.DIR = 7;
	PORTB_PIN3CTRL = PORT_ISC_INTDISABLE_gc;
	
	while (1) {
		/*pip('1', 16);
		pip('5', 16);
		pip(' ', 16);*/
		
		pip('c', 8);
		pip('d', 8);
		pip('e', 8);
		pip('f', 8);
		pip('g', 8);
		pip('a', 8);
		pip('h', 8);

		for (uint8_t n=0; n<20 ; n++) { // blikat LEDkama
			PORTA.OUTTGL = 255-15;
			PORTB.OUTTGL = 7;
			_delay_ms(100);
		}
	}
}

/*const uint16_t tonCas(const char ton, const uint16_t kor) {
	switch (ton) {
		case 'a': return idxCas(0.0, kor);
		case 'A': return idxCas(1.0, kor);
		case 'h': return idxCas(2.0, kor);
		case 'c': return idxCas(3.0, kor);
		case 'C': return idxCas(4.0, kor);		
		case 'd': return idxCas(5.0, kor);
		case 'D': return idxCas(6.0, kor);
		case 'e': return idxCas(7.0, kor);
		case 'f': return idxCas(8.0, kor);
		case 'F': return idxCas(9.0, kor);
		case 'g': return idxCas(10.0, kor);
		case 'G': return idxCas(11.0, kor);
		case else: return 1;
	}
}*/