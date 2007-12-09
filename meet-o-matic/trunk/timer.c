#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#include "timer.h"

// The number of times timer 0 has overflowed since the program started.
// Must be volatile or gcc will optimize away some uses of it.
volatile uint32_t timer0_overflow_count;

SIGNAL(SIG_OVERFLOW0)
{
	timer0_overflow_count++;
}

uint32_t millis(void)
{
	// timer 0 increments every 64 cycles, and overflows when it reaches
	// 256.  we would calculate the total number of clock cycles, then
	// divide by the number of clock cycles per millisecond, but this
	// overflows too often.
	//return timer0_overflow_count * 64UL * 256UL / (F_CPU / 1000UL);
	
	// instead find 1/128th the number of clock cycles and divide by
	// 1/128th the number of clock cycles per millisecond
	return timer0_overflow_count * 64UL * 2UL / (F_CPU / 128000UL);
}

void delay(uint32_t ms) {
	uint32_t start = millis();
	
	while (millis() - start < ms)
		;
}


void TimerInit(void) {
 	timer0_overflow_count = 0;
	// on the ATmega168, timer 0 is also used for fast hardware pwm
	// (using phase-correct PWM would mean that timer 0 overflowed half as often
	// resulting in different millis() behavior on the ATmega8 and ATmega168)
//	sbi(TCCR0A, WGM01);
//	sbi(TCCR0A, WGM00);
	// set timer 0 prescale factor to 64
//	sbi(TCCR0B, CS01);
//	sbi(TCCR0B, CS00);
	// enable timer 0 overflow interrupt
//	sbi(TIMSK0, TOIE0);


	 //Timer0 Settings: Timer Prescaler /64, 
  TCCR0B &= ~(1<<CS02); 
  TCCR0B |= ((1<<CS01) | (1<<CS00));
  // Use normal mode, do not use the OC0B and OC0A Pins.
  TCCR0A |= ~((1<<COM0B1) | (1<<COM0B0) | (1<<COM0A1) | (1<<COM0A0) | (1<<WGM01) | (1<<WGM00));   
  TCCR0B &= ~(1<<WGM02);                 
  TIMSK |= (1<<TOIE0) | (0<<OCIE0A);        //Timer0 Overflow Interrupt Enable  
}

