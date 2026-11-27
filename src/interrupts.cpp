#include "interrupts.h"

// Interrupt service routines (ISRs)
ISR(TIMER0_COMPA_vect) {
  flag_mainloop = 1;    // Set a flag for the main loop to do stuff
}

ISR(TIMER1_COMPA_vect) {
  // Tach interrupt
  if(tach_enable){
    PORTB ^= (1 << PB7);        // Digital pin 11
  } else {
    PORTB &= ~(1 << PB7);        // Digital pin 11
  }
  OCR1A += count_intvl_tach;
}

ISR(TIMER1_COMPB_vect) {
  // Speedo interrupt
  if(speed_enable){
    PORTD ^= (1 << PD0);        // Digital pin 3 
  } else {
    PORTD &= ~(1 << PD0);       // Digital pin 3 
  }
  OCR1B += count_intvl_speed;
}

ISR(TIMER1_COMPC_vect) {
  // Warning light interrupt
  if(warn_blink_flag){
    PORTD ^= (1 << PD1);        // Digital pin 2
  }
  OCR1C += WARN_BLINK_COUNTS;
}

// Fuel gauge software PWM on Timer4. The pin goes high at the start of each period
// and low at the compare-A match. Duty 0 and TIMER4_TOP never toggle.
// COMPA has a higher vector priority than OVF, so if both are pending at once the
// pin ends high. Near the top that is correct; near the bottom it would give a full
// high period, so keep the fuel table's lowest duty well above zero (or exactly zero).
ISR(TIMER4_OVF_vect) {
  if(fuel_duty > 0){
    PORTB |= (1 << PB5);        // Digital pin 9, must match PIN_FUEL
  }
}

ISR(TIMER4_COMPA_vect) {
  if(fuel_duty < TIMER4_TOP){
    PORTB &= ~(1 << PB5);       // Digital pin 9, must match PIN_FUEL
  }
}
