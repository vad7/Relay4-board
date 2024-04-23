/*
* GeneratorVentFan.c
* Generator ventilation fan control
*
* Created: 29.03.2024
*  Author: Vadim Kulakov, vad7@yahoo.com
*
* ATtiny44A
*
*/ 
#define F_CPU 1000000UL
// Fuses: BODLEVEL = 2.7V (BODLEVEL[2:0] = 101), EESAVE(=0), RSTDISBL=0
//		  Int 8 Mhz fast start: CKSEL = 0010 SUT = 00, CKDIV8=1

//#define DEBUG_PROTEUS

#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <util/atomic.h>

//#define DEBUG_PROTEUS

#define LED1_PORT					PORTB		// Green LED
#define LED1						(1<<PORTB3)
#define LED1_ON						LED1_PORT &= ~LED1
#define LED1_OFF					LED1_PORT |= LED1
#define LED2_PORT					PORTB		// Red LED
#define LED2						(1<<PORTB2)
#define LED2_ON						LED2_PORT |= LED2
#define LED2_OFF					LED2_PORT &= ~LED2
#define LEDS_SETUP					DDRB |= LED1 | LED2

#define KEY1						(1<<PORTA7) // SW1
#define KEY1_PRESSING				!(PINA & KEY1)
#define KEYS_SETUP					PORTA |= KEY1	// pullup

#define IN1							(1<<PINB0)	// Line status
#define IN1_ACTIVE					(PINB & IN1)
#define IN2							(1<<PINB1)	// Gen status
#define IN2_ACTIVE					(PINB & IN2)

#define RELAY1_PORT					PORTA
#define RELAY1						(1<<PORTA0)
#define RELAY1_ON					RELAY1_PORT |= RELAY1
#define RELAY1_OFF					RELAY1_PORT &= ~RELAY1
#define RELAY2_PORT					PORTA
#define RELAY2						(1<<PORTA1)
#define RELAY2_ON					RELAY2_PORT |= RELAY2
#define RELAY2_OFF					RELAY2_PORT &= ~RELAY2
#define RELAY3_PORT					PORTA
#define RELAY3						(1<<PORTA2)
#define RELAY3_ON					RELAY3_PORT |= RELAY3
#define RELAY3_OFF					RELAY3_PORT &= ~RELAY3
#define RELAY4_PORT					PORTA
#define RELAY4						(1<<PORTA3)
#define RELAY4_ON					RELAY4_PORT |= RELAY4
#define RELAY4_OFF					RELAY4_PORT &= ~RELAY4
#define RELAYS_SETUP				DDRA = RELAY1 | RELAY2 | RELAY3 | RELAY4 // First code

#define UNUSED_PINS_SETUP			PORTA |= (1<<PORTA4) | (1<<PORTA5) | (1<<PORTA6);	// Pullup: not used
#define SETUP_WATCHDOG WDTCSR = (1<<WDCE) | (1<<WDE); WDTCSR = (1<<WDE) | (0<<WDIE) | (0<<WDP3) | (1<<WDP2) | (1<<WDP1) | (0<<WDP0);	//  Watchdog 1 s

typedef enum {
	cmd_Relay1_On				= 0,
	cmd_Relay1_Off				= 1,
	cmd_Relay2_On				= 2,
	cmd_Relay2_Off				= 3,
	cmd_Relay3_On				= 4,
	cmd_Relay3_Off				= 5,
	cmd_Relay4_On				= 6,
	cmd_Relay4_Off				= 7,
	cmd_RelayDelay				= 8,	// delay - EEPROM.RelaySwitchDelay
	cmd_Delay					= 9,	// delay - next 2 byte - 0..65535 sec
	cmd_End						= 0xFF,
} CMD;

const uint8_t cmds_default_1[] PROGMEM = {	cmd_Relay4_On, cmd_Relay1_Off, cmd_Relay2_Off, cmd_Relay3_Off, cmd_RelayDelay, cmd_Relay1_On, cmd_Relay4_Off, cmd_End,
											cmd_Relay4_On, cmd_Relay1_Off, cmd_Relay2_Off, cmd_Relay3_Off, cmd_RelayDelay, cmd_Relay4_Off, cmd_End };	// IN1 on/off. ON cmds, [cmd_End], OFF cmds, [cmd_End]
const uint8_t cmds_default_2[] PROGMEM = {	cmd_Relay4_On, cmd_Relay1_Off, cmd_Relay2_On, cmd_Relay3_On, cmd_RelayDelay, cmd_Relay1_On, cmd_Relay4_Off, cmd_End,
											cmd_Relay4_On, cmd_Relay1_Off, cmd_Relay2_Off, cmd_Relay3_Off, cmd_RelayDelay, cmd_Relay4_Off, cmd_End };	// IN2 on/off. ON cmds, [cmd_End], OFF cmds, [cmd_End]

#define EEPROM_cmds_size		16*6
struct _EEPROM {
	uint8_t		_OSCCAL;
	uint8_t		KeyDeadtime;				// sec
	uint16_t	OffDelay;					// sec
	uint8_t		RelaySwitchDelay;			// * 0.1 sec
	uint8_t		_reserved[16-5];
	CMD			cmds_1[EEPROM_cmds_size];	// ON cmds 1, [cmd_End], OFF cmd, [cmd_End]
	CMD			cmds_2[EEPROM_cmds_size];	// ON cmds 2, [cmd_End], OFF cmd, [cmd_End]
} __attribute__ ((packed));
struct _EEPROM EEMEM EEPROM;

typedef enum {
	W_OFF = 0,
	W_ON_1  = 1,	// only in1 was active
	W_ON_2  = 2,	// in2 was active
	W_TURN_OFF = 3
} WORK;

WORK	work						= W_OFF;
uint8_t in1_work					= 0;
uint8_t in1_timer					= 0;
uint8_t in2_work					= 0;
uint8_t in2_timer					= 0;
uint8_t key1_press_on				= 0;
uint8_t key1_press_off				= 0;
uint8_t key1_pressed				= 0;
uint8_t key1_repeat					= 0;
uint8_t key1_delay					= 0;
volatile uint8_t Timer				= 0;	// sec
uint16_t OffDelay					= 0;	// sec

#if(1)
void Delay10us(uint8_t ms) {
	while(ms-- > 0) _delay_us(10); 
	wdt_reset();
}
//void Delay1ms(uint8_t ms) {
//	while(ms-- > 0) { 
//		_delay_ms(1); 
//		wdt_reset(); 
//	}
//}
void Delay100ms(uint8_t ms) {
	while(ms-- > 0) { 
		_delay_ms(100); 
		wdt_reset();
	}
}

void FlashLED(uint8_t num, uint8_t toff, uint8_t ton) {
	while (num-- > 0) {
		LED1_OFF;
		Delay100ms(toff);
		LED1_ON;
		Delay100ms(ton);
	}
	LED1_OFF;
}

void FlashLED2(uint8_t num, uint8_t toff, uint8_t ton) {
	while (num-- > 0) {
		LED2_OFF;
		Delay100ms(toff);
		LED2_ON;
		Delay100ms(ton);
	}
	LED2_OFF;
}

void FlashNumber(uint8_t Number)
{ // HEX
	FlashLED(Number / 16, 5, 15);
	Delay100ms(20);
	FlashLED(Number % 16, 5, 5);
	Delay100ms(20);
}

#endif

ISR(PCINT0_vect)
{
	if(key1_delay == 0) {
		if(KEY1_PRESSING) {
			if(key1_press_on <= 15 && key1_press_off <= 10) key1_repeat++; else key1_repeat = 0;
			key1_press_on = 0;
			key1_press_off = 0;
			key1_pressed = 1;
		}
		key1_delay = 2;
	}
}

uint8_t TimerCnt = 0;
ISR(TIM0_OVF_vect, ISR_NOBLOCK) // ~10Hz, 99.3 ms
{
	if(++TimerCnt == 10) { // 1 sec
		TimerCnt = 0;
		if(Timer) Timer--;
		ATOMIC_BLOCK(ATOMIC_FORCEON) {
			if(OffDelay) {
				if(--OffDelay == 0) {
					if(!in2_work) work = W_TURN_OFF;
					LED1_OFF;
				} else LED1_PORT ^= LED1;
			}
		}
	}
	if(IN1_ACTIVE != in1_work && ++in1_timer > 10) {
		in1_work = IN1_ACTIVE;
		in1_timer = 0;
	}
	if(IN2_ACTIVE != in2_work && ++in2_timer > 10) {
		in2_work = IN2_ACTIVE;
		in2_timer = 0;
	}
	if(KEY1_PRESSING) {
		if(key1_press_on < 255) key1_press_on++;
	} else {
		if(key1_press_off < 255) key1_press_off++; 
	}
	if(key1_delay) key1_delay--;
}

void GetSettings(void)
{
	//uint8_t b = eeprom_read_byte(&EEPROM._OSCCAL);
	//if(b != 0xFF) OSCCAL = b;
	//Flags = eeprom_read_byte(&EEPROM.Flags);
}

void ResetSettings(void)
{
	eeprom_update_byte(&EEPROM._OSCCAL, OSCCAL);
	eeprom_update_byte(&EEPROM.KeyDeadtime, 5);
	eeprom_update_byte(&EEPROM.RelaySwitchDelay, 1);// *0.1s
	eeprom_update_word(&EEPROM.OffDelay, 15*60);	// sec
	for(uint8_t i = 0; i < sizeof(cmds_default_1); i++) eeprom_update_byte(&EEPROM.cmds_1[i], pgm_read_byte(&cmds_default_1[i]));
	for(uint8_t i = 0; i < sizeof(cmds_default_2); i++) eeprom_update_byte(&EEPROM.cmds_2[i], pgm_read_byte(&cmds_default_2[i]));
}

void Switch_FAN(uint8_t on)
{
	uint8_t *p;
	if(!on || in1_work || in2_work) {
		p = (uint8_t *)(in2_work ? &EEPROM.cmds_2 : &EEPROM.cmds_1);
		uint8_t i = 0;
		if(on == 0) { // -> off, skip on cmds
			for(; i < EEPROM_cmds_size; i++) if(eeprom_read_byte(&p[i]) == cmd_End) break;
			i++;
			LED2_OFF;
			OffDelay = 0;
			work = W_OFF;
		} else {
			LED2_ON;
			work = in2_work ? W_ON_2 : W_ON_1;
		}
		for(; i < EEPROM_cmds_size; i++) {
			uint8_t cmd = eeprom_read_byte(&p[i]);
			if(cmd == cmd_Relay1_On) RELAY1_ON;
			else if(cmd == cmd_Relay1_Off) RELAY1_OFF;
			else if(cmd == cmd_Relay2_On) RELAY2_ON;
			else if(cmd == cmd_Relay2_Off) RELAY2_OFF;
			else if(cmd == cmd_Relay3_On) RELAY3_ON;
			else if(cmd == cmd_Relay3_Off) RELAY3_OFF;
			else if(cmd == cmd_Relay4_On) RELAY4_ON;
			else if(cmd == cmd_Relay4_Off) RELAY4_OFF;
			else if(cmd == cmd_RelayDelay) Delay100ms(eeprom_read_byte(&EEPROM.RelaySwitchDelay));
			else if(cmd == cmd_Delay) { 
				uint16_t s = eeprom_read_word((uint16_t*)&p[++i]);
				i++;
				while(s-- > 0) {
					LED2_PORT ^= LED2;
					Delay100ms(10); 
					LED2_PORT ^= LED2;
				}
			} else break;
		}
		Timer = eeprom_read_byte(&EEPROM.KeyDeadtime);
	}
}

int main(void)
{
	RELAYS_SETUP;
	CLKPR = (1<<CLKPCE); CLKPR = (0<<CLKPS3) | (0<<CLKPS2) | (1<<CLKPS1) | (1<<CLKPS0); // Clock prescaler division factor: 8
	MCUCR = (1<<SE) | (0<<SM1) | (0<<SM0); // Idle sleep enable
	LEDS_SETUP;
	KEYS_SETUP;
	UNUSED_PINS_SETUP;
	// Timer 8 bit		NRF24L01_Buffer	Unknown identifier	Error
	TCCR0A = (1<<WGM01) | (1<<WGM00);  // Timer0: Fast PWM OCRA
	TCCR0B = (1<<WGM02) | (1 << CS02) | (0 << CS01) | (1 << CS00); // Timer0 prescaller: 1024
	OCR0A = 96; // OC0A =Fclk/prescaller/Freq - 1, F=Fclk/(prescaller*(1+TOP)) = ~10hz
	//OCR0B = 0; // Half Duty cycle ((TOP+1)/2-1)
	TIMSK0 = (1<<TOIE0); // Timer/Counter0, Overflow Interrupt Enable
	// Timer 16 bit
	//TCCR1A = (1<<WGM11) | (1<<WGM10);  // Timer1: Fast PWM Top OCR1A (15)
	//TCCR1B = (1<<ICES1) | (0<<ICNC1) | (1<<WGM13) | (1<<WGM12) | (0<<CS12) | (0<<CS11) | (1<<CS10); // Timer1: /1, Input Capture Rising Edge
	//OCR1A = 0xFFFF; // =Fclk/prescaller/Freq - 1; Freq=Fclk/(prescaller*(1+TOP)). resolution 0.5us/2
	//TIMSK1 = (1<<ICIE1) | (1<<TOIE1); // Timer/Counter1: Input Capture Interrupt Enable
	// ADC
 	//ADMUX = (0<<REFS1) | (1<<MUX2)|(1<<MUX1)|(1<<MUX0); // ADC7 (PA7)
 	//ADCSRA = (1<<ADEN) | (0<<ADATE) | (1<<ADIE) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0); // ADC enable, Free Running mode, Interrupt, ADC 128 divider
 	//ADCSRB = (1<<ADLAR) | (0<<ADTS2) | (0<<ADTS1) | (0<<ADTS0); // ADC Left Adjust Result
	// Pin change
 	GIMSK = (1<<PCIE0) | (0<<PCIE1); // Pin Change Interrupt Enable 0, 1
 	PCMSK0 = (1<<PCINT7); // Pin Change Mask Register 0 - Key1
 	//PCMSK1 = (1<<PCINT8) | (1<<PCINT9) | (1<<PCINT10); // Pin Change Mask Register 0 - Keys
	if(eeprom_read_byte(&EEPROM._OSCCAL) == 0xFF) ResetSettings();
	GetSettings();
	SETUP_WATCHDOG;
	sei();
	uint8_t num = 2;
	while(num-- > 0) {
		LED1_ON;
		LED2_ON;
		Delay100ms(1);
		LED1_OFF;
		LED2_OFF;
		Delay100ms(1);
	}
	//FlashLED(2,1,1);
	while(1)
	{
		__asm__ volatile ("" ::: "memory"); // Need memory barrier
		sleep_cpu();
		wdt_reset();
		
		if(Timer == 0) {
			if(work == W_OFF) {
				if(in2_work) { // Gen ON
					Switch_FAN(1);
				} else if(key1_pressed || KEY1_PRESSING) {
					FlashLED(1,1,1);
					Switch_FAN(1);
					LED1_ON;
					ATOMIC_BLOCK(ATOMIC_FORCEON) OffDelay = eeprom_read_word(&EEPROM.OffDelay);
					key1_press_off = 0;
					key1_press_on = 0;
					key1_pressed = 0;
				}
			} else if(work == W_TURN_OFF) {
				if(in2_work) Switch_FAN(1); else Switch_FAN(0);
				LED1_OFF;
			} else { // ON
				uint8_t _is_dly;
				ATOMIC_BLOCK(ATOMIC_FORCEON) _is_dly = OffDelay != 0;
				if(work == W_ON_1) { // only in1 on
					if(!in1_work) { // switch to in2
						if(_is_dly) ATOMIC_BLOCK(ATOMIC_FORCEON) OffDelay = 0;
						Switch_FAN(1);
						LED1_OFF;
					} else if(!in2_work) {
						if(key1_pressed && !key1_repeat) { // short press
							FlashLED(1,1,1);
							Switch_FAN(0);
							key1_pressed = 0;
							key1_press_off = 0;
							key1_press_on = 0;
						} else {
							if(KEY1_PRESSING) {
								if(_is_dly && key1_press_on > 150) ATOMIC_BLOCK(ATOMIC_FORCEON) OffDelay = 0; // toggle button switched on
							} else if(key1_press_on > 150) { // toggle button switched off
								Switch_FAN(0);
								LED1_OFF;
								key1_press_on = 0;
							} else if(!_is_dly) LED1_ON;
						}
					}
				} else { // in2 on
					if(!in2_work) { // switch to in1 with off delay
						Switch_FAN(1);
						ATOMIC_BLOCK(ATOMIC_FORCEON) OffDelay = eeprom_read_word(&EEPROM.OffDelay);
					}
					if(key1_pressed) key1_pressed = 0;
				}
			}
		} else {
			key1_pressed = 0;
		}
		if(key1_repeat >= 4) { // 5 clicks - Setup delay -> 1 click = +1 minute to delay
			ATOMIC_BLOCK(ATOMIC_FORCEON) OffDelay = 0;
			LED1_OFF;
			Delay100ms(2);
			FlashLED(5,1,1);
			Delay100ms(10);
			uint16_t d = 0;
			Timer = 20;
			while(Timer) {
				__asm__ volatile ("" ::: "memory"); // Need memory barrier
				sleep_cpu();
				wdt_reset();
				if(key1_pressed) {
					FlashLED(1,0,1);
					key1_pressed = 0;
					d++;
					Timer = 20;
				}
			}
			FlashLED(3,1,1);
			if(d) {
				FlashLED(3,1,1);
				Delay100ms(20);
				FlashNumber(d / 256);	// HEX
				Delay100ms(20);
				FlashNumber(d & 255);	// HEX
				eeprom_update_word(&EEPROM.OffDelay, d * 60);
			}
			key1_press_on = 0;
			key1_press_off = 0;
			key1_pressed = 0;
			key1_repeat = 0;
		}
	}
}
