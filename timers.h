
// Loop timing, Hz
#define MAIN_INT_FREQ_HZ    250    // Timer0: runs the main loop at this frequency. Prescaler and compare value are calculated below.
#define FREQ_V_BUS_HZ       10     // Frequency to sample and transmit bus voltage
#define WARN_BLINK_HZ       4      // When there's a warning, blink a light this fast
#define INIT_LIGHT_ON_S     1      // Time in seconds to keep all lights on before turning off

#define TIMER1_PRESCALER    64      // Timer1 is a 16-bit counter


// Calculate the prescaler and compare value
#define INIT_LIGHT_ON_CTS     INIT_LIGHT_ON_S*MAIN_INT_FREQ_HZ
#define COUNT_INTVL_V_BUS     MAIN_INT_FREQ_HZ / FREQ_V_BUS_HZ
#define TIMER0_COMPARE_VALUE  100       // Initialize fast so it updates quickly
#define TIMER1_FREQ           (F_CPU / TIMER1_PRESCALER)
#define TIMER1_COMPARE_VALUE  100       // Initialize fast so it updates quickly
#define WARN_BLINK_COUNTS     TIMER1_FREQ / 2*WARN_BLINK_HZ


// ---- Timer0 (8-bit CTC) config, derived from MAIN_INT_FREQ_HZ ----
// Timer0 counts 0..OCR0A then resets, dividing F_CPU by
// prescaler * (OCR0A + 1). OCR0A is 8 bits (max 255), so only
// F_CPU / (prescaler * N) for N in [1,256], prescaler in {1,8,64,256,1024}
// are actually reachable.

// Pick the smallest prescaler that keeps the tick count within 8 bits
#if   (F_CPU / 1UL    / MAIN_INT_FREQ_HZ) <= 256
  #define TIMER0_PRESCALER 1UL
#elif (F_CPU / 8UL    / MAIN_INT_FREQ_HZ) <= 256
  #define TIMER0_PRESCALER 8UL
#elif (F_CPU / 64UL   / MAIN_INT_FREQ_HZ) <= 256
  #define TIMER0_PRESCALER 64UL
#elif (F_CPU / 256UL  / MAIN_INT_FREQ_HZ) <= 256
  #define TIMER0_PRESCALER 256UL
#elif (F_CPU / 1024UL / MAIN_INT_FREQ_HZ) <= 256
  #define TIMER0_PRESCALER 1024UL
#else
  #error "MAIN_INT_FREQ_HZ too low - no Timer0 prescaler can reach it"
#endif

#define TIMER0_TICKS ((F_CPU + (TIMER0_PRESCALER * MAIN_INT_FREQ_HZ) / 2) \
                        / (TIMER0_PRESCALER * MAIN_INT_FREQ_HZ))

#define TIMER0_COMPARE_VALUE (TIMER0_TICKS - 1)

#if TIMER0_COMPARE_VALUE > 255
  #error "TIMER0_COMPARE_VALUE out of 8-bit range - check MAIN_INT_FREQ_HZ"
#endif


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



