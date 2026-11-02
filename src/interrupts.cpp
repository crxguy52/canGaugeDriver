#include "interrupts.h"

// Interrupt service routines (ISRs)
ISR(TIMER0_COMPA_vect) {
  flag_mainloop = 1;    // Set a flag for the main loop to do stuff
}

ISR(TIMER1_COMPA_vect) {
  // Tach interrupt
  if(tach_enable){
    digitalWrite(PIN_TACH, !digitalRead(PIN_TACH));
  } else {
    digitalWrite(PIN_TACH, LOW);
  }
  OCR1A += count_intvl_tach;
}

ISR(TIMER1_COMPB_vect) {
  // Speedo interrupt
  if(speed_enable){
    digitalWrite(PIN_SPEED, !digitalRead(PIN_SPEED));
  } else {
    digitalWrite(PIN_SPEED, LOW);
  }
  OCR1B += count_intvl_speed;
}

ISR(TIMER1_COMPC_vect) {
  // Warning light interrupt
  if(warn_blink_flag){
    digitalWrite(PIN_SRS, !digitalRead(PIN_SRS));
  }
  OCR1C += WARN_BLINK_COUNTS;
}
