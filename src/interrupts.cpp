#include "interrupts.h"

// Interrupt service routines (ISRs)
ISR(TIMER0_COMPA_vect) {
  flag_mainloop = 1;    // Set a flag for the main loop to do stuff
}

ISR(TIMER1_COMPA_vect) {
  // Tach interrupt
   if(engData.rpm > MIN_RPM){         
    PORTB ^= (1 << PB7);        // Digital pin 11
   } else {
    PORTB &= ~(1 << PB7);        // Digital pin 11
   }
  OCR1A += count_intvl_tach;
}

ISR(TIMER1_COMPB_vect) {
  // Speedo interrupt
  if(engData.mph > MIN_MPH){   
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
