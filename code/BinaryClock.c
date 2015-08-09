/*
 * BinaryClock.c
 *
 * Created: 13.03.2014 15:29:36
 *  Author: Sven Stucki
 */


#include <avr/io.h>
#include <avr/interrupt.h>
#include <inttypes.h>

#include <util/delay.h>

#include "USI_TWI_Master.h"


/* this macro reorders the bits of a uint16_t so they correspond to the order of the LEDs on the board */
#define LED_ORDER(x)	( ((x) & 0xE0) >> 5 | ((x) & 0x1F) << 3 | ((x) & 0xFF00) )


/* application state */

/* 0 - normal, 1 - set hour, 2 - set minute */
uint8_t mode = 0;

/* current time*/
uint8_t time_h = 0, time_m = 0;

uint8_t tick_odd = 0;


/* application logic forward declaration */
void s1();
void s2();
void s3();
void tick();


uint16_t led_state = 0x0000;

/* send new LED state to LED driver */
void set_leds(uint16_t in) {
	unsigned int i;
	
	/* save new state in global */
	led_state = in;
	
	/* make sure latch is low */
	PORTB &= ~(1 << PB0);
	
	for (i = 0; i < 16; i++) {
		/* set clock to low */
		PORTB &= ~(1 << PB1);
		
		/* send bit */
		if (in & 0x8000) {
			PORTB |= (1 << PB2);
		} else {
			PORTB &= ~(1 << PB2);
		}
		
		/* set clock to high */
		PORTB |= (1 << PB1);
		
		/* move to next bit */
		in <<= 1;
	}
	
	/* send latch pulse */
	PORTB |= (1 << PB0);
	PORTB &= ~(1 << PB0);
}


/* interrupt handling */

uint8_t btns_prev;

ISR(PCINT0_vect) {
	uint8_t btns = PINA ^ btns_prev;
	btns_prev = PINA;
	
	if ((PINA & (1 << PA2)) == 0 && (btns & (1 << PA2)) != 0) {
		/* S1 pressed */
		s1();
	}
	if ((PINA & (1 << PA3)) == 0 && (btns & (1 << PA3)) != 0) {
		/* S2 pressed */
		s2();
	}
	if ((PINA & (1 << PA5)) == 0 && (btns & (1 << PA5)) != 0) {
		/* S3 pressed */
		s3();
	}
}

ISR(TIM1_COMPA_vect) {
	tick();
}

ISR(ADC_vect) {
	OCR0B = ADCH >> 1;
	ADCSRA |= (1 << ADSC);
}


#define RTC_ADDRESS	0xD0

void update_time() {
	unsigned char i2c_buffer[3];
	
	/* select minutes register */
	i2c_buffer[0] = RTC_ADDRESS | 0;	// write
	i2c_buffer[1] = 0x01;
	USI_TWI_Start_Transceiver_With_Data(i2c_buffer, 2);
	
	/* read minute and hour register */
	i2c_buffer[0] = RTC_ADDRESS | 1;	// read
	USI_TWI_Start_Transceiver_With_Data(i2c_buffer, 3);
	
	/* convert BCD digits to number */
	unsigned char minute_bcd = i2c_buffer[1];
	time_m = ((minute_bcd >> 4) & 0x07) * 10 + (minute_bcd & 0x0F);
	unsigned char hour_bcd = i2c_buffer[2];
	time_h = ((hour_bcd >> 4) & 0x03) * 10 + (hour_bcd & 0x0F);
}

void set_time() {
	unsigned char i2c_buffer[5];
	i2c_buffer[0] = RTC_ADDRESS | 0;	// write
	i2c_buffer[1] = 0x00;	// seconds register (followed by minute, then hour)
	i2c_buffer[2] = 0x00;	// set seconds to 0
	
	/* convert minute to BCD */
	i2c_buffer[3] = 0;
	i2c_buffer[3] |= (((time_m / 10) % 10) & 0x07) << 4;
	i2c_buffer[3] |= (time_m % 10) & 0x0F;
	
	/* convert hour to BCD */
	i2c_buffer[4] = 0;
	i2c_buffer[4] |= (((time_h / 10) % 10) & 0x03) << 4;
	i2c_buffer[4] |= (time_h % 10) & 0x0F;
	
	USI_TWI_Start_Transceiver_With_Data(i2c_buffer, 5);
}


int main(void) {
	/* disable /8 system clock prescaler */
	CLKPR = 0x80;
	CLKPR = 0x00;
	
	/* setup port directions */
	DDRA = (1 << DDA7);
	DDRB = (1 << DDB0) | (1 << DDB1) | (1 << DDB2);
	
	/* enable pull-ups for buttons */
	PORTA |= (1 << PA2) | (1 << PA3) | (1 << PA5);
	
	USI_TWI_Master_Initialise();

	/* activate LEDs */
	//PORTA &= ~(1 << PA7);
	
	/* prepare and start timer 0 LED PWM */
	TCCR0A = (1 << COM0B1) | (1 << COM0B0) | (1 << WGM01) | (1 << WGM00);
	TCCR0B = (0 << WGM02) | (0 << CS02) | (0 << CS01) | (1 << CS00);
	OCR0B = 0xA0;
	
	/* prepare and start timer 1 for 2 Hz interrupt */
	TCCR1A = 0x00;
	TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10);	// CTC mode, /64 prescale
	TCCR1C = 0x00;
	OCR1A = 0xF424;		// count to 62500 -> exactly 2 Hz
	TIMSK1 = (1 << OCIE1A);
	
	/* prepare and enable button interrupts */
	btns_prev = PINA;
	PCMSK0 = (1 << PCINT2) | (1 << PCINT3) | (1 << PCINT5);
	GIMSK = 0x10;
	
	/* prepare and start ADC */
	ADMUX = (1 << MUX0);
	ADCSRA = (1 << ADEN) | (1 << ADSC) | (1 << ADIE) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
	ADCSRB = (1 << ADLAR);
	
	/* enable global interrupts */
	sei();
	
	while(1);
		return 0;
}


/* application logic */

void s1() {
	mode = (mode + 1) % 3;
}

void s2() {
	if (mode == 1) {
		/* increase hour */
		time_h++;
		if (time_h >= 24) {
			time_h = 0;
		}
	} else if (mode == 2) {
		/* increase minute */
		time_m++;
		if (time_m >= 60) {
			time_m = 0;
			time_h++;
			if (time_h >= 24) {
				time_h = 0;
			}
		}
	}
}

void s3() {
	if (mode == 1) {
		/* decrease hour */
		if (time_h == 0) {
			time_h = 23;
		} else {
			time_h--;
		}
	} else if (mode == 2) {
		/* decrease minute */
		if (time_m == 0) {
			if (time_h == 0) {
				time_h = 23;
			} else {
				time_h--;
			}
			time_m = 59;
		} else {
			time_m--;
		}
	}
}


void tick() {
	tick_odd++;
	
	uint16_t leds = 0;
	if (mode == 0) {
		/* display time */
		update_time();
		leds = LED_ORDER((time_m << 5) | time_h);
	} else if (mode == 1) {
		/* flash hour */
		set_time();
		if (tick_odd % 2 == 0) {
			leds = LED_ORDER(time_m << 5);
		} else {
			leds = LED_ORDER((time_m << 5) | time_h);
		}
		leds |= 0x8000;
	} else if (mode == 2) {
		/* flash minute */
		set_time();
		if (tick_odd % 2 == 0) {
			leds = LED_ORDER(time_h);
		} else {
			leds = LED_ORDER((time_m << 5) | time_h);
		}
		leds |= 0x4000;
	}
	set_leds(leds);
}
