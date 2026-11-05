/*
freqTest
Bench test for the frequency gauge outputs (tach or speedo), one at a time.
Set TEST_OUTPUT below, flash, then open the serial monitor (SERIAL_BAUD) and
send an integer: square-wave frequency in Hz (0 = off, pin held low).

Timer1 runs in CTC mode, and the COMPA ISR toggles the pin. Timer0 is left alone,
so millis()/Serial work normally.

Standalone sketch: Arduino can't include files from outside the sketch folder,
so the pins below are copied from src/hwdefs.h. Keep them in sync.
For coolant/oil/fuel, use dutyTest.
*/

enum TestOutput { OUT_TACH, OUT_SPEEDO };

// ---- Pick the output under test ----
const TestOutput TEST_OUTPUT = OUT_TACH;

// Pins
const uint8_t PIN_TACH  = 11;   // PB7
const uint8_t PIN_SPEED = 3;    // PD0

const uint32_t SERIAL_BAUD     = 115.2e3;
const uint8_t  MAX_INPUT_DIGITS = 9;      // Longest input that can't overflow a long
const long     FREQ_MAX_HZ     = 10e3;    // Keeps the toggle ISR well below CPU saturation
const uint32_t TIMER1_TOP_MAX  = 0xFFFF;
const uint16_t TIMER1_PRESCALERS[] = {1, 8, 64, 256, 1024};   // Index + 1 = CS1[2:0] bits
const uint8_t  NUM_PRESCALERS  = sizeof(TIMER1_PRESCALERS) / sizeof(TIMER1_PRESCALERS[0]);

const uint8_t OUT_PIN = (TEST_OUTPUT == OUT_TACH) ? PIN_TACH : PIN_SPEED;
volatile uint8_t freqPinState = LOW;


ISR(TIMER1_COMPA_vect) {
  freqPinState = !freqPinState;
  digitalWrite(OUT_PIN, freqPinState);
}


// Stop Timer1 and hold the pin low
void stop_frequency() {
  TCCR1B = 0;
  TIMSK1 &= ~(1 << OCIE1A);
  freqPinState = LOW;
  digitalWrite(OUT_PIN, LOW);
}


// Output a square wave at hz. Picks the smallest prescaler whose compare value fits
// in 16 bits, which gives the finest frequency resolution.
void set_frequency(long hz) {
  stop_frequency();

  if (hz <= 0) {
    Serial.println("Output off");
    return;
  }
  if (hz > FREQ_MAX_HZ) {
    Serial.print("Out of range, max Hz = ");  Serial.println(FREQ_MAX_HZ);
    return;
  }

  for (uint8_t i = 0; i < NUM_PRESCALERS; i++) {
    // Two toggles per period
    uint32_t top = F_CPU / (2UL * TIMER1_PRESCALERS[i] * (uint32_t)hz);
    if (top >= 1 && top - 1 <= TIMER1_TOP_MAX) {
      TCCR1A = 0;                         // Clear the PWM mode Arduino's init() leaves on Timer1
      TCNT1  = 0;
      OCR1A  = top - 1;
      TIFR1  = (1 << OCF1A);              // Clear any stale compare flag
      TIMSK1 |= (1 << OCIE1A);
      TCCR1B = (1 << WGM12) | (i + 1);    // CTC mode, start clock

      float actual = (float)F_CPU / (2.0 * TIMER1_PRESCALERS[i] * top);
      Serial.print("Freq set: ");        Serial.print(hz);
      Serial.print(" Hz, actual = ");    Serial.print(actual, 3);
      Serial.print(" Hz, prescaler = "); Serial.print(TIMER1_PRESCALERS[i]);
      Serial.print(", OCR1A = ");        Serial.println(top - 1);
      return;
    }
  }
  Serial.println("Frequency too low for Timer1");
}


// Parse a non-negative decimal integer. Rejects anything else, because String::toInt()
// silently returns zero (or a partial number) for typos.
bool parse_uint(const String& s, long& out) {
  if (s.length() == 0 || s.length() > MAX_INPUT_DIGITS) {
    return false;
  }
  for (unsigned int i = 0; i < s.length(); i++) {
    if (!isDigit(s[i])) {
      return false;
    }
  }
  out = s.toInt();
  return true;
}


void setup() {
  Serial.begin(SERIAL_BAUD);
  while (!Serial) {}            // 32U4 native USB: wait for the monitor so the banner isn't lost

  pinMode(OUT_PIN, OUTPUT);
  stop_frequency();

  Serial.print("freqTest: output = ");  Serial.print(TEST_OUTPUT == OUT_TACH ? "TACH" : "SPEEDO");
  Serial.print(", pin = ");             Serial.println(OUT_PIN);
  Serial.println("Send frequency in Hz (0 = off)");
}


void loop() {
  if (Serial.available() == 0) {
    return;
  }

  // Works with any serial-monitor line ending; with "No line ending" or CR only,
  // each entry takes effect after the read timeout instead of immediately
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) {
    return;
  }

  long hz;
  if (!parse_uint(line, hz)) {
    Serial.print("Invalid input, expected a whole number of Hz: ");  Serial.println(line);
    return;
  }
  set_frequency(hz);
}
