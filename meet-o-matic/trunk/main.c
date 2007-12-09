#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

#include "timer.h"


/* define CPU frequency in Mhz here if not defined in Makefile */
#ifndef F_CPU
#define F_CPU 7372800UL
#endif

#define LED_CHUNK_TIME 1000
// We have three LED banks to multiplex
#define MULTIPLEX_LED_MAX 3 
#define TIMER_MAX_VALUE 30 
#define DEBOUNCE_MILLIS 1

/**
 * Calculate the period for a frequency of 4,5 khz:
 * 4,5 khz = 4500 hz = 4500 per second
 * 1 period = 1/4500 second 
 * We need two halfwaves, so WAVE_DELAY = 2/4500 = 1/2250 = 0.0004444 s
 * = 444 us
 */
#define WAVE_DELAY 444

/**
 * Function prototypes - see below for declaration
 */
void multiplexDisplays(void);
void initialize(void);
void beep(void);
void shutdown(void);
void updateTimer(void);
uint8_t checkPushbutton(void);

/**
 * Global variables
 */
volatile uint32_t chunk_start_time = 0;
volatile uint32_t last_pushbutton_time = 0; 
uint8_t timer_value = 0;
volatile uint8_t pushbutton = 0;
uint8_t multiplex_counter = 0;

int main(void) {
  while (1) {
    initialize();
    do {
      multiplexDisplays();
      updateTimer();
      if (checkPushbutton())
        timer_value+=1;
      if (timer_value > TIMER_MAX_VALUE)
        timer_value=1;
    } while (timer_value);
    multiplexDisplays();
    for (int i=0; i<5; i++) {
      beep();
      delay(250);
    }
    // Finished counting - go to sleep, reduce power consumption.
    shutdown();
    // If we are here, an external interrupt 0 occured - reinitialize the device.
  }
}



void multiplexDisplays(void){
  uint8_t value;
  value = 10*multiplex_counter;
  switch (multiplex_counter) {
    case 0: PORTD &= ~(1<<PD4); break;
    case 1: PORTD &= ~(1<<PD0); break;
    case 2: PORTD &= ~(1<<PD3); break;
  }
  if (timer_value > 0+value) PORTD &= ~(1<<PD5); else PORTD |= (1<<PD5); 
  if (timer_value > 1+value) PORTD &= ~(1<<PD6); else PORTD |= (1<<PD6);
  if (timer_value > 2+value) PORTB &= ~(1<<PB0); else PORTB |= (1<<PB0); 
  if (timer_value > 3+value) PORTB &= ~(1<<PB1); else PORTB |= (1<<PB1); 
  if (timer_value > 4+value) PORTB &= ~(1<<PB2); else PORTB |= (1<<PB2); 
  if (timer_value > 5+value) PORTB &= ~(1<<PB3); else PORTB |= (1<<PB3); 
  if (timer_value > 6+value) PORTB &= ~(1<<PB4); else PORTB |= (1<<PB4); 
  if (timer_value > 7+value) PORTB &= ~(1<<PB5); else PORTB |= (1<<PB5); 
  if (timer_value > 8+value) PORTB &= ~(1<<PB6); else PORTB |= (1<<PB6); 
  if (timer_value > 9+value) PORTB &= ~(1<<PB7); else PORTB |= (1<<PB7); 
  switch (multiplex_counter) {
    case 0: PORTD |= (1<<PD0); break;
    case 1: PORTD |= (1<<PD3); break;
    case 2: PORTD |= (1<<PD4); break;
  }
  multiplex_counter++;
  if (multiplex_counter >= MULTIPLEX_LED_MAX) {
    multiplex_counter = 0;
  }
}

void updateTimer(void) {
  /**
   * Check if we need to update the time.
   */
  if (millis() - chunk_start_time > LED_CHUNK_TIME) {
    chunk_start_time=millis();
    timer_value--;
  }
}



/**
 * Interrupthandler for INT0 - called when the signal on Pin PD2 changes.
 */
ISR(INT0_vect) {
   if (PIND & (1 << PD2))
     pushbutton = 1;
};

uint8_t checkPushbutton(void) {
  if (pushbutton > 0) { 
    delay(DEBOUNCE_MILLIS);
    if (PIND & (1<<PD2)) { 
      pushbutton = 0;
      chunk_start_time = last_pushbutton_time = millis();
      return 1;
    }
  }
  return 0;
}

void initialize(void) {

  /**
   * Configure the LED frame output ports.
   */
  DDRB |= (1<<PB7) | (1<<PB6) | (1<<PB5) | (1<<PB4) | \
          (1<<PB3) | (1<<PB2) | (1<<PB1) | (1<<PB0);
  DDRD |= (1<<PD5) | (1<<PD6);
  
  /**
   * Buzzer is an output port as well.
   */
  DDRD |= (1<<PD1);

  /**
   * Configure the frame selector output ports.
   */
  DDRD |= (1<<PD4) | (1<<PD3) | (1<<PD0);

  /**
   * Initialize the frame - all LEDs are glowing.
   */
  PORTB = 0x00;
  PORTD &= ~(1<<PD5);
  PORTD &= ~(1<<PD6);
  PORTD |= (1<<PD4) | (1<<PD3) | (1<<PD0);
  
  /**
   * Initialize the external interrupt 0 - this is our pushbutton.
   * Use PD2 as an input pin. The interrupt should be activated when a
   * falling flank is detected.
   */
  DDRD &= ~(1<<PD2);
  MCUCR |= (1<<ISC01) | (1<<ISC00);
  GIMSK |= (1<<INT0);

  TimerInit();
  sei();
  
  beep();
  delay(1000);
  checkPushbutton();
  /**
   * Initialize the global variables
   */
  chunk_start_time = last_pushbutton_time = millis();
  timer_value = TIMER_MAX_VALUE;
}

void shutdown(void) {
  /**
   * activate the interrupt when a new level occurs - we need a
   * different behaviour than above for reviving the device after a
   * sleep.
   */
  MCUCR &= ~(1<<ISC01) & ~(1<<ISC00);
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_mode();
}

void beep(void) {
  /**
   * We use PD2 as an output port for the buzzer
   */
  DDRD |= (1<<PD2);
  for (int i=0; i<1000; i++) {
    PORTD |= (1<<PD1);
    PORTD &= ~(1<<PD2);
    _delay_us(WAVE_DELAY);
    PORTD &= ~(1<<PD1);
    PORTD |= (1<<PD2);
    _delay_us(WAVE_DELAY);
  }
  /**
   * Make sure we can use the port as input for the pushbutton again
   */
  DDRD &= ~(1<<PD2);
}

