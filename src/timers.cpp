#include "timers.h"


// Configure timer0 to run at 1kHz
void timer0_init() {

  // Initialize to zero - Arduino initializes this to something else intially
  TCCR0A = 0;
  TCCR0B = 0;
 
  // Set Timer 0 to CTC mode (Clear Timer on Compare Match)
  TCCR0A |= (1 << WGM01);

  // Set the prescaler
  if (TIMER0_PRESCALER == 1) {
    TCCR0B |= (1 << CS00);
  } else if (TIMER0_PRESCALER == 8) {
    TCCR0B |= (1 << CS01);
  } else if (TIMER0_PRESCALER == 64) {
    TCCR0B |= (1 << CS01) | (1 << CS00);
  } else if (TIMER0_PRESCALER == 256) {
    TCCR0B |= (1 << CS02);
  } else if (TIMER0_PRESCALER == 1024) {
    TCCR0B |= (1 << CS02) | (1 << CS00);
  } else {
    // Handle invalid prescaler value
    // For example, set a default prescaler or return an error
    TCCR0B |= (1 << CS01) | (1 << CS00); // Defaults to 64
  }

  // Set the compare value
  OCR0A = TIMER0_COMPARE_VALUE;

  // Start the count clean
  TCNT0 = 0;                 // start the count clean too

  // Enable Timer 0 compare match A interrupt
  TIMSK0 |= (1 << OCIE0A);
}


void timer1_init() {
  // Set Timer 1 to normal mode, count up to 0xFFFF
  TCCR1A = 0;
  TCCR1B = 0;

  // Set the prescaler
  if (TIMER1_PRESCALER == 1) {
    TCCR1B |= (1 << CS10);
  } else if (TIMER1_PRESCALER == 8) {
    TCCR1B |= (1 << CS11);
  } else if (TIMER1_PRESCALER == 64) {
    TCCR1B |= (1 << CS11) | (1 << CS10);
  } else if (TIMER1_PRESCALER == 256) {
    TCCR1B |= (1 << CS12);
  } else if (TIMER1_PRESCALER == 1024) {
    TCCR1B |= (1 << CS12) | (1 << CS10);
  } else {
    // Handle invalid prescaler value
    // For example, set a default prescaler or return an error
    TCCR1B |= (1 << CS11) | (1 << CS10); // Defaults to 64
  }

  // Set initial compare values
  OCR1A = TIMER1_COMPARE_VALUE;
  OCR1B = TIMER1_COMPARE_VALUE;
  OCR1C = WARN_BLINK_COUNTS;

  // Enable compare match interrupts for A, B, and C
  TIMSK1 |= (1 << OCIE1A) | (1 << OCIE1B) | (1 << OCIE1C);

  // Enable global interrupts
  sei();
}