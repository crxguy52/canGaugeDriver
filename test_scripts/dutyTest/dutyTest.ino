/*
dutyTest
Bench test for the duty-cycle gauge outputs (coolant, oil, or fuel), one at a time.
Set TEST_OUTPUT below, flash, then open the serial monitor (SERIAL_BAUD) and
send an integer: PWM duty, 0..PWM_MAX (clamped).

Uses analogWrite, i.e. Arduino's default hardware PWM:
  coolant (D5) on Timer3, oil (D10) and fuel (D9) on Timer1.
Note the firmware can't do this for oil/fuel (Timer1 drives tach/speedo there),
so it will use software PWM at a different frequency. If the gauge response
depends on PWM frequency, the duty -> needle calibration may not carry over exactly.

Standalone sketch: Arduino can't include files from outside the sketch folder,
so the pins below are copied from pinMap.xlsx. Keep them in sync.
For tach/speedo, use freqTest.
*/

enum TestOutput { OUT_COOLANT, OUT_OIL, OUT_FUEL };

// ---- Pick the output under test ----
const TestOutput TEST_OUTPUT = OUT_COOLANT;

// Pins
const uint8_t PIN_COOLANT = 5;    // PC6, Timer3 PWM
const uint8_t PIN_OIL     = 10;   // PB6, Timer1 PWM (pinMap: Oil_pressure)
const uint8_t PIN_FUEL    = 9;    // PB5, Timer1 PWM (pinMap: Fuel_lvl_out)

const uint32_t SERIAL_BAUD      = 115.2e3;
const uint8_t  MAX_INPUT_DIGITS = 9;      // Longest input that can't overflow a long
const long     PWM_MAX          = 255;


uint8_t select_pin() {
  switch (TEST_OUTPUT) {
    case OUT_COOLANT: return PIN_COOLANT;
    case OUT_OIL:     return PIN_OIL;
    case OUT_FUEL:    return PIN_FUEL;
  }
  return PIN_COOLANT;
}


const char* output_name() {
  switch (TEST_OUTPUT) {
    case OUT_COOLANT: return "COOLANT";
    case OUT_OIL:     return "OIL";
    case OUT_FUEL:    return "FUEL";
  }
  return "?";
}


const uint8_t OUT_PIN = select_pin();


void set_pwm(long duty) {
  duty = constrain(duty, 0, PWM_MAX);
  analogWrite(OUT_PIN, duty);
  Serial.print("PWM set: ");  Serial.println(duty);
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
  digitalWrite(OUT_PIN, LOW);

  Serial.print("dutyTest: output = ");  Serial.print(output_name());
  Serial.print(", pin = ");             Serial.println(OUT_PIN);
  Serial.print("Send PWM duty 0..");    Serial.println(PWM_MAX);
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

  long duty;
  if (!parse_uint(line, duty)) {
    Serial.print("Invalid input, expected a whole number: ");  Serial.println(line);
    return;
  }
  set_pwm(duty);
}
