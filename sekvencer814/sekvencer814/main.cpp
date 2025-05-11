#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <math.h>

#define F_CPU	(20000000UL)
#include <util/delay.h>

#define F_CPUkor	(F_CPU + ((F_CPU * (int8_t)SIGROW.OSC20ERR5V) / 1024L))
#define uartBaud(br) ((float) (F_CPUkor*64 / (16*(float)br)) + 0.5)
//#define UART_baud2(br,f) ((float) (f*64/(16*(float)br))+0.5)
#define UART_bd		19200

#define PIP_port	PORTB
#define PIP_bm		(1<<2)
#define OUT_port	PORTA
#define OUT_bp		(4)
#define TL1_port	PORTB
#define TL1_bm		(1<<3)
#define TL2_port	PORTA
#define TL2_bm		(1<<3)
#define TL3_port	PORTA
#define TL3_bm		(1<<0)

const uint8_t TLAC_none = 0;
const uint8_t TLAC_tl1  = 1<<1;
const uint8_t TLAC_tl2  = 1<<2;
const uint8_t TLAC_tl3  = 1<<3;
const uint8_t TLAC_all  = 7<<1;

volatile uint32_t	ms = 0;
volatile uint8_t	msSync = 0, sekSync = 0, adSync = 0;
uint8_t				tlacitka = TLAC_none;

const uint16_t	KROK_min = 10;
const uint16_t	KROK_max = 65535;
const uint16_t	KROK_default = 5000;
uint16_t		krokCas = KROK_default;
bool			progPrepnout = 0;
bool			skipCekani=0; // skipnout èekání pokud má skipEn

const uint8_t	SEQ_end = 255;
const uint8_t	SEQ_pip0 = SEQ_end-1;
const uint8_t	SEQ_pip1 = SEQ_end-2;
const uint8_t	SEQ_pip2 = SEQ_end-3;
const uint8_t	SEQ_pip3 = SEQ_end-4;
const uint16_t	PIP_frq0 = 1200;
const uint16_t	PIP_frq1 = 1600;
const uint16_t	PIP_frq2 = 2500;
const uint16_t	PIP_frq3 = 3200;
const uint8_t	SEQ_len = 40;
uint8_t			seqPredef[SEQ_len] = {
	0b1000, 0b0100, 0b0010, 0b0001, SEQ_end,
	0b1000, 0b0100, 0b0010, SEQ_end,
	0b1100, 0b0110, 0b0011, 0b1001, SEQ_end,
	0b1100, 0b0110, 0b1010, SEQ_end,
	0b1100, 0b0011, SEQ_end,
	0b1010, 0b0101, SEQ_end,
	0b1111, 0b0000, SEQ_end,
	SEQ_end
};

const uint8_t	ESEQ_len = 30;
EEMEM uint16_t	eKrokCas = 65535;
EEMEM uint8_t	eProgSel = 255;
EEMEM uint8_t	eAutoMeas = 1;
EEMEM uint8_t	eSeqMode = 'p';
EEMEM uint8_t	eRevMode = 0;
EEMEM uint8_t	eSeqVals[ESEQ_len];
EEMEM uint16_t	eSeqTimes[ESEQ_len];

uint8_t			progSelRestart = 0; // >0 volba predef sekvence po restartu
uint8_t			progEepRestart = 0; // restart EEPROM sekvence od 0

const uint8_t	ADC_cycles = 200;
const uint8_t	ADC_chanCnt = 2;
const uint8_t	ADC_chanTab[ADC_chanCnt] = {
	ADC_MUXPOS_AIN10_gc, ADC_MUXPOS_AIN11_gc 
};
const float		ADC_refVolt = 0.55;
const float		ADC_samples = 64.0 * static_cast<float>(ADC_cycles);
const float		ADC_maxVal = 1023.0;
const float		ADC_lsbV = ADC_refVolt / (ADC_maxVal * ADC_samples);
const float		ADC_calConst[ADC_chanCnt] = {
	ADC_lsbV * (1.0 + 27.0/3.0), ADC_lsbV * (1.0 + 27.0/3.0)
};
volatile uint32_t adRawData[ADC_chanCnt] = {0, 0};
float			adData[ADC_chanCnt] = {0.0, 0.0};
uint32_t adTest[]={0, 0};
	
uint8_t			autoMeas = 1;	// automaticky mìøit každou s
uint8_t			seqMode = 'p';	// p pøeddefonovné sekvence / e EEPROM / m maual - ze sériáku
uint8_t			revMode = 0;	// reverzovat bity 0/1

void putchar2(char c) { // ====================================================
	while ((USART0.STATUS & USART_DREIF_bm) == 0);
	USART0.TXDATAL=c;
}

void print(char t[]) { // =====================================================
	uint8_t n;
	
	for (n=0; n<50; n++) {
		if (t[n]==0) break;
		putchar2(t[n]);
	}
}
void print(long xx, int8_t m=0) { // cislo, mist (0 auto) ==================
	char t[10];
	signed char n=0;
	long x;
	
	x=xx;
	if (x<0) {x=-xx; putchar2('-');}
	for (n=0; n<9; n++) {
		t[n]=x % 10 + '0';
		x/=10;
		if (m<=0 && x==0) break;
	}
	if (m<0 && n<-m) {
		print("0.");
		for (uint8_t i=1; i<-m-n; i++) putchar2('0');
	}
	for (n=(m>0)? m-1: n; n>=0; n--) {
		putchar2(t[n]);
		if (m<0 && n==-m) putchar2('.');
	}
}
void print(float val, uint8_t mi) { // cislo, des.mist ========================
	float v = val;
	for (uint8_t n=0; n<mi; n++) v*=10.0;
	print(static_cast<long>(v), -mi);
}

void pip(uint8_t pocet); // aby byl pip vidìt už v cekej
void pip(uint16_t frq, uint16_t t);
void usartRxMessage();

bool atomMsTest(uint32_t val) { // ============================================
	__asm("cli");
	bool t = ms<val;
	__asm("sei");
	return t;
}

void atomMs0() { // ===========================================================
	__asm("cli");
	ms=0;
	__asm("sei");
}

void cekej(uint32_t delayTime, bool skipEn=0) { // ============================
	atomMs0();
	do {
		if (msSync==1) { // -----------------------------------------
			msSync=0;
			
			static uint8_t tlacCnt = 0;
			static uint8_t tlacLast = TLAC_none;
			uint8_t tlacTmp = TLAC_none;
			static bool tlacSemafor = 0;
			
			if (!(TL1_port.IN & TL1_bm)) tlacTmp |= TLAC_tl1;
			if (!(TL2_port.IN & TL2_bm)) tlacTmp |= TLAC_tl2;
			if (!(TL3_port.IN & TL3_bm)) tlacTmp |= TLAC_tl3;
			if (tlacTmp != tlacLast) tlacCnt = 0;
			
			if (!tlacSemafor) if (++tlacCnt>50) {
				// zaèal stisk TL..
				if (tlacitka == TLAC_none && tlacTmp == TLAC_tl1) {
					progPrepnout=1;
					skipCekani=1;
				}
				if (tlacitka == TLAC_none && tlacTmp == TLAC_tl2) {
					uint32_t tmp = krokCas;
					tmp *= 10000;
					tmp /= 11892; // 2^(1/4) = 1,18920711..
					tlacSemafor = 1;
					if (tmp >= KROK_min) {
						krokCas = static_cast<uint16_t>(tmp);
						pip(1);
					} else {
						krokCas = KROK_min;
						pip(2);
					}
					eeprom_update_word(&eKrokCas, krokCas);
					tlacSemafor = 0;
				}
				if (tlacitka == TLAC_none && tlacTmp == TLAC_tl3) {
					uint32_t tmp = krokCas;
					tmp *= 11892; // 2^(1/4) = 1,18920711..
					tmp /= 10000;
					tlacSemafor = 1;
					if (tmp <= KROK_max) {
						krokCas = static_cast<uint16_t>(tmp);
						pip(1);
					} else {
						krokCas = KROK_max;
						pip(2);
					}
					eeprom_update_word(&eKrokCas, krokCas);
					tlacSemafor = 0;
				}
				
				tlacitka = tlacTmp;
			}
			tlacLast = tlacTmp;
			
			
		}
		if (sekSync==1) { // ----------------------------------------
			sekSync=0;
			if (autoMeas==1) {
				for (uint8_t n=0; n<ADC_chanCnt; n++) {
					//print(adTest[n], 0);
					//print(" ");
					print(adData[n], (uint8_t)4);
					print(" V\t");
				}
				print("dif ");
				print(adData[1]-adData[0], (uint8_t)3); print(" V ");
				print(" \n");
			}
		}
		if (adSync==1) { // -----------------------------------------
			
			for (uint8_t n=0; n<ADC_chanCnt; n++) {
				adTest[n] = adRawData[n];
				adData[n] = static_cast<float>(adRawData[n]) * ADC_calConst[n];
				adRawData[n] = 0;
			}	
			adSync=0;
		}
		
		usartRxMessage();
		
		if (skipCekani && skipEn) {
			skipCekani=0;
			break;
		}
	} while (atomMsTest(delayTime));
}

uint16_t pipCnt = 0;
void pip(uint16_t frq, uint16_t t) { // =======================================
	__asm("cli");
	TCA0.SINGLE.PER = F_CPU / static_cast<uint32_t>(frq);
	TCA0.SINGLE.CMP2 = TCA0.SINGLE.PER / 2;
	TCA0.SINGLE.CTRLA |= TCA_SINGLE_ENABLE_bm;
	PIP_port.DIRSET = PIP_bm;
	pipCnt = t;
	__asm("sei");
	//print("pip "); print(frq); putchar2(' '); print(t); putchar2('\n');
}
void pip(uint8_t pocet) { // ==================================================
	for (uint8_t n=0; n<pocet; n++) {
		pip(3200, 100);
		cekej(200);
	}
}

void progSelect(uint8_t n) { // ===============================================
	if (n<1) return;
	if (progSelRestart==0) {pip(n); eeprom_update_byte(&eProgSel, n);}
}

void outSet(uint8_t val) { // =================================================
	uint8_t v = val & 15;
	if (revMode) { // obrátit poøadí bitù
		uint8_t t=0;
		for (uint8_t n=0; n<4; n++) {t <<= 1; t |= v & 1; v >>= 1;}
		v = t;
	}
	v <<= OUT_bp;
	OUT_port.OUTCLR = (15 << OUT_bp) - v;
	OUT_port.OUTSET = v;
}
void outSet(uint8_t val0, uint8_t val1, uint8_t val2, uint8_t val3) { // ======
	uint8_t v = (val0 & 1);
	v += (val1 & 1) << 1;
	v += (val2 & 1) << 2;
	v += (val3 & 1) << 3;
	outSet(v);
}


int main(void) { // ###########################################################
	_PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, 0);		// nedìlit f CPU, bude 20 MHz
	// ---------------------------------------------------------------------------------------------
	PORTA.DIR = 0b11110010;
	PORTA_PIN0CTRL = PORT_PULLUPEN_bm;
	PORTA_PIN2CTRL = PORT_PULLUPEN_bm;
	PORTA_PIN3CTRL = PORT_PULLUPEN_bm;
	PORTA.OUT = 2; // TxD
	
	PORTB.DIR = 0;
	PORTB_PIN0CTRL = PORT_ISC_INTDISABLE_gc;
	PORTB_PIN1CTRL = PORT_ISC_INTDISABLE_gc;
	PORTB_PIN3CTRL = PORT_PULLUPEN_bm;
	// ---------------------------------------------------------------------------------------------
	// TCA použijeme na pípání
	TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV1_gc;
	TCA0.SINGLE.CTRLB = TCA_SINGLE_WGMODE_SINGLESLOPE_gc + TCA_SINGLE_CMP2EN_bm;
	TCA0.SINGLE.PER = F_CPU / 1300UL;
	TCA0.SINGLE.CMP2 = TCA0.SINGLE.PER / 2;
	//TCA0.SINGLE.CTRLA |= TCA_SINGLE_ENABLE_bm;
	// ---------------------------------------------------------------------------------------------
	// TCB použijeme na èekání
	TCB0.CTRLA = TCB_CLKSEL_CLKDIV1_gc;
	TCB0.CTRLB = TCB_CNTMODE_INT_gc;
	TCB0.INTCTRL = TCB_CAPTEI_bm;
	TCB0.CCMP = F_CPU / 1000UL;
	TCB0.DBGCTRL = TCB_DBGRUN_bm;
	TCB0.CTRLA |= TCB_ENABLE_bm;
	TCB0.INTFLAGS = TCB_CAPTEI_bm;
	// ---------------------------------------------------------------------------------------------
	// sériový port
	PORTMUX.CTRLB|=PORTMUX_USART0_ALTERNATE_gc;			// RxD - PORTA.2, TxD - PORTA.1
	USART0.CTRLA = USART_RXCIE_bm;						// tx bez int, rx int, 485 off
	USART0_CTRLC = USART_CMODE_ASYNCHRONOUS_gc + USART_CHSIZE_8BIT_gc;
	USART0_BAUD = uartBaud(UART_bd);
	USART0.STATUS |= USART_RXCIF_bm;					// konec IF
	USART0_CTRLB = USART_RXEN_bm + USART_TXEN_bm;		// tx en, rx en
	// ---------------------------------------------------------------------------------------------
	VREF_CTRLA = VREF_ADC0REFSEL_0V55_gc;				// 2,5 V
	VREF_CTRLB = VREF_ADC0REFEN_bm;						// always on
	// ---------------------------------------------------------------------------------------------
	ADC0_CTRLA = ADC_RUNSTBY_bm + ADC_FREERUN_bm + ADC_ENABLE_bm;	// standby run on, freerun on, enable
	ADC0_CTRLB = ADC_SAMPNUM_ACC64_gc;					// zapnout akumulaci 64 vzorkù
	ADC0_CTRLC = ADC_REFSEL_INTREF_gc + ADC_PRESC_DIV64_gc;
	ADC0_CTRLD = 0; //ADC_INITDLY_DLY16_gc + (2<<ADC_SAMPDLY_gp); // init delay 16, samp delay 2?
	ADC0_CTRLE=0;										// No Window Comparison (default)
	ADC0_SAMPCTRL=0;									// Sample Length = 2+reg
	ADC0_MUXPOS=ADC_chanTab[0];							// ADC input pin
	ADC0_COMMAND=ADC_STCONV_bm;							//?Start Conversion
	ADC0_EVCTRL=0;										// no events
	ADC0_INTCTRL = ADC_RESRDY_bm;						// int result ready
	ADC0_INTFLAGS = ADC_RESRDY_bm;						// clear IF
	ADC0_DBGCTRL=0;										// not work in break
	ADC0_CALIB = ADC_DUTYCYC_DUTY25_gc;					// 25% Duty Cycle (high 25% and low 75%) must be used for ADCclk ? 1.5 MHz
	// ---------------------------------------------------------------------------------------------
	__asm("sei"); // povolit pøerušení
	
	while(1) {
		autoMeas = eeprom_read_byte(&eAutoMeas);
		seqMode = eeprom_read_byte(&eSeqMode);
		revMode = eeprom_read_byte(&eRevMode);
		
		krokCas = eeprom_read_word(&eKrokCas);
		progSelRestart = eeprom_read_byte(&eProgSel)-1;
		
		if (autoMeas<=1 || revMode<=1) break;
		// asi není inicializovaná EEPROM
		eeprom_write_byte(&eAutoMeas, 1);
		eeprom_write_byte(&eSeqMode, 'p');
		eeprom_write_byte(&eRevMode, 0);
			
		eeprom_write_byte(&eProgSel, 1);
		eeprom_write_word(&eKrokCas, KROK_default);
		
		uint8_t seqPos=0;
		for (; seqPos<ESEQ_len; seqPos++) {
			eeprom_write_byte(&eSeqVals[seqPos], seqPredef[seqPos]);
			eeprom_write_word(&eSeqTimes[seqPos], 500);
			if (seqPredef[seqPos]==SEQ_end) break;
		}
		for (; seqPos<ESEQ_len; seqPos++) {
			eeprom_write_byte(&eSeqVals[seqPos], SEQ_end);
			eeprom_write_word(&eSeqTimes[seqPos], 500);
		}		
		
		pip(500, 100); cekej(100);
		pip(600, 100); cekej(100);
		pip(700, 100); cekej(300);
	}

	if (autoMeas==0) print("autoMeas=0\n");
	// ------------------------------------------------------------------------
	while (1) { // hlavní
		uint8_t seqStartPos = 0;
		uint8_t seqSelected = 0;
		progSelRestart = eeprom_read_byte(&eProgSel)-1;
		while (seqMode=='p') { // pøeddef sekvence ------------------------
			while (seqMode=='p') {
				if (progSelRestart>0) {progSelRestart--; break;}
				for (uint8_t seqPos=seqStartPos; seqPos<SEQ_len; seqPos++) {
					if (seqPredef[seqPos]==SEQ_end) break;
					outSet(seqPredef[seqPos]);
					cekej(krokCas, 1);
				}
				if (progPrepnout) {progPrepnout=0; break;}
			}
			if (seqMode!='p') break;
			if (progSelRestart!=0 && seqMode=='e') {
				progSelRestart = 0;
			}
		
			seqSelected++;
			for (uint8_t n=0, c=0; n<SEQ_len-1; n++) {
				if (c==seqSelected) {
					seqStartPos=n;
					break;
				}
				if (seqPredef[n]==SEQ_end) {
					if (seqPredef[n+1]==SEQ_end) {
						seqSelected=0;
						seqStartPos=0;
						break;
					}
					c++;
				}
			}
			progSelect(seqSelected+1);
		}
		
		while (seqMode=='e') { // EEPROM sekvence -------------------------
			for (uint8_t seqPos=0; seqPos<ESEQ_len; seqPos++) {
				uint8_t d = eeprom_read_byte(&eSeqVals[seqPos]);
				uint16_t t = eeprom_read_word(&eSeqTimes[seqPos]);
				if (d==SEQ_end) break;
				switch (d) {
					case SEQ_pip0:
						pip(PIP_frq0, t);
						break;					
					case SEQ_pip1:
						pip(PIP_frq1, t);
						break;
					case SEQ_pip2:
						pip(PIP_frq2, t);
						break;
					case SEQ_pip3:
						pip(PIP_frq3, t);
						break;
					default:
						outSet(d);
						cekej(t, 1);
				}
				if (progEepRestart>0) {progEepRestart=0; break;}
			}
			if (seqMode!='e') break;
		}
		
		while (seqMode=='m') { // manual - z poèítaèe -------------------------
			cekej(100, 1);
		}
	}
}

const uint8_t cUsartRxBufLen = 44;
volatile char usartRxBuff[cUsartRxBufLen];
volatile uint8_t usartRxBuffIend = 0;	// index za poslední pøijatý znak
volatile uint8_t usartRxMsgComplete = 0;// 1 dokonèen pøíjem (pøijata mezera)
volatile uint8_t usartRxMsgFail = 0;	// 1 buff overflow / 2 timeout
volatile uint8_t usartRxMsgTout = 255;	// 0-253 bìží, 254 tout err, 255 zastaveno

ISR (USART0_RXC_vect) { // ====================================================
	if (usartRxMsgComplete==0) {
		if (usartRxBuffIend==0) usartRxMsgTout=0;
		if (usartRxBuffIend>=cUsartRxBufLen-1) {
			usartRxMsgTout=255;
			usartRxBuffIend=255;
			usartRxMsgFail |= 1;
		}
		usartRxBuff[usartRxBuffIend] = USART0.RXDATAL;
		usartRxBuffIend++;

		if (usartRxBuff[usartRxBuffIend-1]==' ') {
			usartRxMsgComplete=1;
			usartRxMsgTout = 255;
		}
		} else {
		volatile char dummy = USART0.RXDATAL;
	}
	USART0.STATUS |= USART_RXCIF_bm;
}

// pøerušení od èasovaèe A
ISR (TCA0_OVF_vect) { // ======================================================
	// aktuálnì není použito
	TCA0.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm;
}

// pøerušení od èasovaèe B
ISR (TCB0_INT_vect) { // ======================================================
	ms++;			  // milisekundy
	msSync = 1;
	if (usartRxMsgTout<254) usartRxMsgTout++;
	if (usartRxMsgTout==254) {
		usartRxMsgTout=255;
		usartRxBuffIend=0;
		usartRxMsgFail |= 2;
	}	
	static uint16_t sCnt = 0;
	if (++sCnt>999) { // sekundy
		sCnt = 0;
		sekSync = 1;
	}
		
	if (pipCnt>0) {	  // pípání
		pipCnt--;
	} else {
		TCA0.SINGLE.CTRLA &= ~TCA_SINGLE_ENABLE_bm;
		PIP_port.DIRCLR = PIP_bm;
	}

	TCB0.INTFLAGS = TCB_CAPTEI_bm;
}

// pøerušení od ADC - 23796 kS/s / 374 S/s
ISR (ADC0_RESRDY_vect) { // ===================================================
	static uint8_t mxIdx=0, cykly=0;
	
	if (adSync==0) {
		adRawData[mxIdx] += static_cast<uint32_t>(ADC0.RES);
	
		if (++mxIdx>=ADC_chanCnt) {
			mxIdx=0;
			if (++cykly>=ADC_cycles) {cykly=0; adSync=1;}
		}
		ADC0.MUXPOS = ADC_chanTab[mxIdx];
	}
	ADC0.INTFLAGS = ADC_RESRDY_bm;
}

void sendStat() { // ==============================================================================
	print("autoMeas:"); print(autoMeas);
	if (seqMode=='p') print(" seqMode:p");
	else if (seqMode=='e') print(" seqMode:e");
	else if (seqMode=='m') print(" seqMode:m");
	else {print(" seqMode:"); print(seqMode);}
	print(" eProgSel:"); print(eeprom_read_byte(&eProgSel));
	print(" krokCas:"); print(krokCas);
	putchar2('\n');
}
template <typename T> T readN(uint8_t start, uint8_t mist, T limit) { // ==========================
	T vi=0;
	uint8_t n;
	for (n=start; n<usartRxBuffIend; n++) {
		if (usartRxBuff[n]<'0' || usartRxBuff[n]>'9') break;
		if (n >= start+mist) break;
		vi *= static_cast<T>(10);
		vi += static_cast<T>(usartRxBuff[n]-'0');
	}
	/*print("st:"); print(start);
	print(" n:"); print(n);
	print(" out:"); print(vi);
	putchar2('\n');*/
	if ((usartRxBuff[n]==' ' || usartRxBuff[n]=='-') && n == start+mist && vi<=limit) {
		return vi;
	} else {
		return static_cast<T>(-1);
	}
}
// èíst mist 0/1 a poskládat do bin výstupu, 255 pøi chybì
uint8_t readBinTxt(uint8_t start, uint8_t mist) { // ==============================================
	uint8_t n;
	uint8_t out=0;
	for (n=start; n<usartRxBuffIend; n++) {
		if (usartRxBuff[n]<'0' || usartRxBuff[n]>'1') break;
		if (n >= start+mist) break;
		out <<= 1;
		out |= (usartRxBuff[n]-'0');
	}
	/*print("st:"); print(start);
	print(" n:"); print(n);
	print(" out:"); print(out);
	putchar2('\n');*/
	//_delay_ms(1);
	if ((usartRxBuff[n]==' ' || usartRxBuff[n]=='-') && n == start+mist) {
		return out;
	} else {
		return 255;
	}
}

uint8_t testCmdPar(const char str[], uint8_t start) { // ==========================================
	for (uint8_t n=start; n<cUsartRxBufLen && n<usartRxBuffIend; n++) {
		if (usartRxBuff[n]!=str[n-start] || str[n-start]==0) break;
		if (usartRxBuff[n]==' ' || usartRxBuff[n]=='-') return n;
	}
	return 0;
}

void usartRxMessage() { // ===============================================================
	static uint8_t failCnt[3] = {0, 0, 0}; // poèet usartRxMsgFail 1 / 2 / 1|2
	const char * usartPrikazy[] {
		"ident ", "m ", "stat ", "set-", "seq-", "serial ", "beep-", "help "
	};
	enum ePrikazy {
		ident, mmm, stat, set, seq, serial, beep, help, END
	};

	if (usartRxMsgComplete==1) {
		ePrikazy pIdx = END;
		uint8_t cmdEnd = 0;
		for (ePrikazy p=(ePrikazy)0; p<END; p = static_cast<ePrikazy>(static_cast<uint8_t>(p) + 1)) {
			for (uint8_t n=0; n<cUsartRxBufLen && n<usartRxBuffIend; n++) {
				if (usartRxBuff[n]!=usartPrikazy[p][n]) break;
				if (usartRxBuff[n]==' ' || usartRxBuff[n]=='-') {pIdx=p; cmdEnd=n; break;}
				if (usartPrikazy[p][n]==' ' || usartPrikazy[p][n]=='-') break;
			}
			if (pIdx!=END) break;
		}
		if (usartRxBuffIend<cUsartRxBufLen-1) usartRxBuff[usartRxBuffIend] = 0;

		float val;
		long vi, vk;
		uint8_t u8, u8ce;
		uint32_t u32;
		static uint8_t setSeqAdr = 0;
		
		switch (pIdx) { // --------------------------------------------------------------
			case ident: // "ident " -------------------------------------------
				print("Sekvencer814v1\n");
				break;
				
			case mmm: // "m " - 
				for (uint8_t n=0; n<ADC_chanCnt; n++) {
					print(adData[n], (uint8_t)4);
					putchar2('\t');
				}
				print(" V\n");
				break;
				
			case stat: // "stat " stavové informace ---------------------------
				sendStat();
				print("usartRxMsgFail:"); print(usartRxMsgFail); print(" /");
				for (uint8_t n=0; n<3; n++) {
					putchar2(' '); print(failCnt[n]);
					failCnt[n] = 0;
				}
				putchar2('\t');
				break;
				
			case seq: // "seq-app-0101-00100 " ---------------------------------------
				if (u8ce = testCmdPar("adr-", cmdEnd+1)) {  // "seq-adr-12 " 00-29 ------
					u8 = readN(u8ce+1, 2, ESEQ_len-1);
					if (u8==255) {
						if (u8ce = testCmdPar("skip ", u8ce+1)) { // "seq-adr-skip " -----
							if (setSeqAdr<ESEQ_len) setSeqAdr++;
							if (setSeqAdr>=ESEQ_len-1) setSeqAdr=0;
							print(setSeqAdr); print(" OK\n");
						} else {
							print("ERR seq-adr\n");
						}
					} else {
						setSeqAdr = u8;
						print(setSeqAdr); print(" OK\n");
					}
					
				} else if (u8ce = testCmdPar("app-", cmdEnd+1)) {  // "seq-app-0101-12345 " -----
					if (setSeqAdr>=ESEQ_len-1) {
						print("ERR seq-app-: ESEQ_len\n");
						break;
					}
					u8 = usartRxBuff[u8ce+1] - '0';
					if (u8<2) { // "seq-app-0101-12345 " bity:0000-1111 ms:00000-65000 --
						u8 = readBinTxt(u8ce+1, 4);
						vi = readN(u8ce+1+4+1, 5, 65000);
						if (u8==255 || vi<0) {
							//print(u8ce); putchar2(' '); print(u8); putchar2(' '); print(vi);
							print("ERR seq-app-bbbb-ttttt\n");
						} else {
							eeprom_update_byte(&eSeqVals[setSeqAdr], u8);
							eeprom_update_word(&eSeqTimes[setSeqAdr], vi);
							setSeqAdr++;
							print(setSeqAdr); print(" OK\n");
						}
					} else { // není app-0 / app-1
						cmdEnd = u8ce;
						if (u8ce = testCmdPar("end ", cmdEnd+1)) { // "seq-app-end " ----
							eeprom_update_byte(&eSeqVals[setSeqAdr], SEQ_end);
							//eeprom_update_word(&eSeqTimes[setSeqAdr], vi);
							setSeqAdr++;
							print(setSeqAdr); print(" OK\n");
						} else if (u8ce = testCmdPar("beep-", cmdEnd+1)) { // "seq-app-beep-0-1234
							u8 = usartRxBuff[u8ce+1] - '0';
							//vi = readN(cmdEnd+1, 4, 9999);
							vk = readN(u8ce+1+1+1, 4, 3000);
							if (u8>3 || vk<0) {
								if (usartRxMsgFail==0) print("ERR beep\n");
							} else {
								eeprom_update_byte(&eSeqVals[setSeqAdr], SEQ_pip0-u8);
								eeprom_update_word(&eSeqTimes[setSeqAdr], vk);
								setSeqAdr++;
								print(setSeqAdr); print(" OK\n");
							}
						}
					}
					
				} else if (u8ce = testCmdPar("reset ", cmdEnd+1)) {  // "seq-reset " -----
					progEepRestart=1;
					skipCekani=1;
					print("OK\n");

				} else if (u8ce = testCmdPar("list ", cmdEnd+1)) {  // "seq-list " -----
					for (uint8_t seqPos=0; seqPos<ESEQ_len; seqPos++) {
						uint8_t d = eeprom_read_byte(&eSeqVals[seqPos]);
						uint16_t t = eeprom_read_word(&eSeqTimes[seqPos]);
						print(seqPos); putchar2(' ');
						switch (d) {
							case SEQ_pip0:
								print("PIP_frq0 "); print(t); putchar2('\n');
								break;					
							case SEQ_pip1:
								print("PIP_frq1 "); print(t); putchar2('\n');
								break;
							case SEQ_pip2:
								print("PIP_frq2 "); print(t); putchar2('\n');
								break;
							case SEQ_pip3:
								print("PIP_frq3 "); print(t); putchar2('\n');
								break;
							case SEQ_end:
								print("SEQ_end"); putchar2('\n');
								break;
							default:
								if (d>15) {
									print("(undef) "); print(t); putchar2('\n');
								} else {
									print("out ");
									putchar2('0' + ((d & 8) >> 3)); d<<=1;
									putchar2('0' + ((d & 8) >> 3)); d<<=1;
									putchar2('0' + ((d & 8) >> 3)); d<<=1;
									putchar2('0' + ((d & 8) >> 3)); d<<=1;
									putchar2(' ');
									print(t); putchar2('\n');
								}
						}
					}
					print("Next app:"); print(setSeqAdr); print(" OK\n");
					
				} else {
					print("ERR seq-\n");
				}
				_delay_ms(1);
				break;
				
			case set: // "set-out-0011 " --------------------------------------
				if (u8ce = testCmdPar("out-", cmdEnd+1)) {   // "set-out-0011 " 0000-1111
					u8 = readBinTxt(u8ce+1, 4);
					//u8 = usartRxBuff[u8ce+1] - '0';
					if (u8==255) {
						print("ERR set-out\n");
					} else {
						outSet(u8);
						print("OK\n");
					}
				} else if (u8ce = testCmdPar("am-", cmdEnd+1)) {  // "set-am-0 " 0/1 --
					u8 = usartRxBuff[u8ce+1] - '0';
					if (u8>1 || usartRxBuffIend != u8ce+3) {
						if (testCmdPar("help ", u8ce+1)) {
							print("Auto mereni: 0 / 1 / help\n");
						} else {
							if (usartRxMsgFail==0) print("ERR set am\n");
						}
					} else {
						autoMeas = u8;
						eeprom_update_byte(&eAutoMeas, u8);
						print("OK\n");
					}
				} else if (u8ce = testCmdPar("smod-", cmdEnd+1)) { // "set-smod-e " e/p --
					u8 = usartRxBuff[u8ce+1];
					if (u8!='e' && u8!='p' && u8!='m' || usartRxBuffIend != u8ce+3) {
						if (testCmdPar("help ", u8ce+1)) {
							print("Mod sekvence: e EEPROM / p preddef./ m manual / help\n");
						} else {
							if (usartRxMsgFail==0) print("ERR set smod\n");
						}
					} else {
						if (seqMode != u8) skipCekani=1;
						seqMode = u8;
						eeprom_update_byte(&eSeqMode, u8);
						print("OK\n");
					}
				} else if (u8ce = testCmdPar("rmod-", cmdEnd+1)) { // "set-rmod-1 " 0/1 --
					u8 = usartRxBuff[u8ce+1] - '0';
					if (u8>1 || usartRxBuffIend != u8ce+3) {
						if (testCmdPar("help ", u8ce+1)) {
							print("Mod reverz: 0 / 1 / help\n");
						} else {
							if (usartRxMsgFail==0) print("ERR set rmod\n");
						}
					} else {
						revMode = u8;
						eeprom_update_byte(&eRevMode, u8);
						print("OK\n");
					}
				} else {
					print("ERR set\n");
				}
				_delay_ms(1);
				break;
			
			case serial: // "serial " - sériové èíslo procesoru, 10 èísel 0-255 --
				print("SN 0-9: ");
				for (uint16_t n=PROD_SIGNATURES_START; n<PROD_SIGNATURES_START+10; n++) {
					print(_SFR_MEM8(n));
					putchar2(' ');
				}
				print("OK\n");
				break;
				
			case beep: // "beep-2500-0200 " frekvence [Hz] èas [ms] -----------
				vi = readN(cmdEnd+1, 4, 9999);
				vk = readN(cmdEnd+1+4+1, 4, 3000);
				if (vi>=0 && vk>=0) {
					pip(vi, vk);
					print("OK\n");
				} else {
					if (usartRxMsgFail==0) print("ERR beep\n");
				}
				break;
				
			case help: // "help " --------------------------------------------
				for (ePrikazy p=(ePrikazy)0; p<END; p = static_cast<ePrikazy>(static_cast<uint8_t>(p) + 1)) {
					putchar2('"');
					for (uint8_t n=0; n<cUsartRxBufLen; n++) {
						putchar2(usartPrikazy[p][n]);
						if (usartPrikazy[p][n]==' ' || usartPrikazy[p][n]=='-') break;
					}
					putchar2('"'); putchar2('\n');
				}
				print("OK\n");
				break;
				
			default: // -------------------------------------------------------
				if (usartRxMsgFail==0) {
					print("ERRcmd\n");
				}
		}
		usartRxBuffIend=0;
		usartRxMsgComplete=0;
	}
	if (usartRxMsgFail!=0) {
		print("#usartRxMessage fail:");
		print(static_cast<uint32_t>(usartRxMsgFail));
		putchar2('\n');
		if (failCnt[(usartRxMsgFail & 3)-1]<255) failCnt[(usartRxMsgFail & 3)-1]++;
		usartRxMsgFail=0;
		return;
	}
}