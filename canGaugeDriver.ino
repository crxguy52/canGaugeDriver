#include <mcp_canbus.h>
#include <SPI.h>

/*
GaugeDriver
V. Zaccardo, March 2025
Program for a Seeed CANBed to recieve CAN data from a GM ECU
And drive a NB Miata tach, speedo, coolant gauge, and all lights

Timer0 is used to run the main loop at 1kHz
  - Receives CAN messages
  - Turns lights on and off based on received values
  - Sets PWM value based on coolant value
  - Sets tachInterval and speedInterval
Timer1 is used to drive variable frequency outputs
  - TIMER1 COMPA is the tach
    - If rpm > RPM_MIN, interrupt toggles PIN_TACH, sets COMPA += tachInterval
    - Else COMBA += tachInterval
  - TIMER1 COMPB is the speedo  -> Interrupt toggles PIN_SPEED, sets COMPB += speedInterval
      - If mph > MPH_MIN, interrupt toggles PIN_TACH, sets COMPB += tachInterval
      - Else COMPB += tachInterval
  - TIMER1 COMPC is used to blink a warning light in the event a param is out of range
*/

// Constants
#define CAN_500KBPS         16
#define CAN_1000KBPS        18
#define LIGHT_ON            HIGH    // Value to turn light on
#define LIGHT_OFF           LOW     // Value to turn light off
#define F_CPU               16000000
//#define DEBUG               1

// Scaling Values
#define ADC2VBUS            0.025   // Convert ADC reading to bus voltage
#define RPM2HZ              0.0167  // RPM to Rev/s (Hz)
#define MPH2HZ              1
#define COOL_T_LOW          80      // Temperature at which coolant gauge should be on the L line
#define COOL_PWM_LOW        50      // PWM value to make coolant gauge read L
#define COOL_T_HIGH         120     // Temperature at which coolant gauge should be on H line
#define COOL_PWM_HIGH       200     // PWM value to make coolant gauge read H
#define COOL_T_HIGHHIGH     200     // Temp at which coolant gauge is pegged H

// Value Thresholds
// LFX spec is 10psi at idle, 30psi at 2,000 RPM
#define P_OIL_LOW_RPM         2000    // Below this RPM low1 is used, above low2        
#define P_OIL_LOW1_PSI        5
#define P_OIL_LOW2_PSI        20
#define V_BUS_LOW             12.5
#define RPM_SHIFT             6800

// Loop timing, Hz
#define MAIN_INT_FREQ_HZ    1000    // Timer0: runs the main loop at 1kHz
#define FREQ_V_BUS_HZ       10      // Frequency to sample and transmit bus voltage
#define WARN_BLINK_HZ       4       // When there's a warning, blink a light this fast
#define INIT_LIGHT_ON_S     1       // Time in seconds to keep all lights on before turning off
#define TIMER0_PRESCALER    64      // Timer0 is an 8-bit counter
#define TIMER1_PRESCALER    64    // Timer1 is a 16-bit counter

// RX CAN message IDs
#define ID_ENGINE_GENERAL_STATUS_1    201   // Contains engine_rpm
#define ID_VEHICLE_SPEED_AND_DISTANCE 1001  // Contains vehicle speed
#define ID_ENGINE_GENERAL_STATUS_4    1217  // Contains eng_coolant_temp
#define ID_ENGINE_GENERAL_STATUS_5    1233  // Contains eng_oil_pressure, CEL

// TX CAN message IDs
#define ID_V_BUS            0x780

// Pin assignments
#define SPI_CS_PIN          17 
#define MCP_PIN_INT         7   // MCP2515 interrupt pin

#define PIN_UART_RX         0
#define PIN_UART_TX         1
#define PIN_SRS             2   // Port D1
#define PIN_SPEED           3   // Port D0
#define PIN_CEL             4
#define PIN_COOLANT         5
#define PIN_ABS             6
#define PIN_SEATBELT        8
#define PIN_CRUISE          9
#define PIN_OILP            10
#define PIN_TACH            11    // Port B7
#define PIN_ALT             12
#define PIN_LED             13

// Calculate the prescaler and compare value
#define INIT_LIGHT_ON_CTS     INIT_LIGHT_ON_S*MAIN_INT_FREQ_HZ
#define COUNT_INTVL_V_BUS     MAIN_INT_FREQ_HZ / FREQ_V_BUS_HZ
#define TIMER0_COMPARE_VALUE  100       // Initialize fast so it updates quickly
#define TIMER1_FREQ           (F_CPU / TIMER1_PRESCALER)
#define TIMER1_COMPARE_VALUE  100       // Initialize fast so it updates quickly
#define WARN_BLINK_COUNTS     TIMER1_FREQ / 2*WARN_BLINK_HZ

// Define minimum values that can will enable the output
// Make it larger than the theoretical minimum to keep update rates reasonalbe, since it only updates
// When the timer interrupts happen
//#define MIN_RPM               TIMER1_FREQ/(2*(pow(2, 16)-1)*RPM2HZ)
//#define MIN_MPH               TIMER1_FREQ/(2*(pow(2, 16)-1)*MPH2HZ)
#define MIN_RPM               200
#define MIN_MPH               5

// Set CAN tranciever chip select pin
MCP_CAN CAN(SPI_CS_PIN);    

// Configure timer0 to run at 1kHz
void timer0_init() {
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

  // Set prescaler to 1024
  // TCCR1B |= (1 << CS12) | (1 << CS10);

  // Set initial compare values
  OCR1A = TIMER1_COMPARE_VALUE;
  OCR1B = TIMER1_COMPARE_VALUE;
  OCR1C = WARN_BLINK_COUNTS;

  // Enable compare match interrupts for A, B, and C
  TIMSK1 |= (1 << OCIE1A) | (1 << OCIE1B) | (1 << OCIE1C);

  // Enable global interrupts
  sei();
}

void setup()
{
    Serial.begin(115200);       // USB serial port
//    Serial1.begin(115200);  /// Pins 0 and 1
    
    while (CAN_OK != CAN.begin(CAN_500KBPS))    // init can bus : baudrate = 500k
    {
        Serial.println("CAN BUS FAILED TO INITIALIZE!");
        delay(100);
    }
    Serial.println("CAN BUS INITIALIZED!");

    // Configure the output pins
    pinMode(PIN_ALT,      OUTPUT);
    pinMode(PIN_SRS,      OUTPUT);
    pinMode(PIN_SPEED,    OUTPUT);
    pinMode(PIN_CEL,      OUTPUT);
    pinMode(PIN_COOLANT,  OUTPUT);
    pinMode(PIN_ABS,      OUTPUT);
    pinMode(PIN_CRUISE,   OUTPUT);
    pinMode(PIN_OILP,     OUTPUT);
    pinMode(PIN_TACH,     OUTPUT);
    pinMode(PIN_LED,      OUTPUT);

    // Configure the arbID masks on the MCP2515 so we only recieve the messages we care about
    // MCP2515 has 2 masks and 6 filters:
    // 1 mask (RXM0) and 2 filters (RXF0-1) for RXB0 and
    // 1 mask (RXM1) and 4 filters (RXF2-5) for RXB1
    // This libarary enables rolling messages from RXB0 to RXB1 by default (RXB0CTRL:BUKT = 1)

    CAN.init_Mask(0, 0, ID_ENGINE_GENERAL_STATUS_1);        // Engine RPM
    CAN.init_Mask(1, 0, 
                        ID_VEHICLE_SPEED_AND_DISTANCE |     // The other messages
                        ID_ENGINE_GENERAL_STATUS_4 | 
                        ID_ENGINE_GENERAL_STATUS_5);

    // Filters for RXB0 - there are two but we only need one
    CAN.init_Filt(0, 0, ID_ENGINE_GENERAL_STATUS_1);

    // Filters for RXB1 - we need three
    CAN.init_Filt(2, 0, ID_VEHICLE_SPEED_AND_DISTANCE);
    CAN.init_Filt(3, 0, ID_ENGINE_GENERAL_STATUS_4);
    CAN.init_Filt(4, 0, ID_ENGINE_GENERAL_STATUS_5);
    CAN.init_Filt(5, 0, 0x0);   // Don't accept anything

    // Set up the timer0 interrupt to run at 1kHz
    timer0_init();

    // Congifure timer1
    timer1_init();

    // Enable global interrupts
    sei();
}

  // Define all the variables
  unsigned char len = 0;
  unsigned char buf[8];  
  float rpm         = 0;
  float kph         = 0;
  float mph         = 0;
  float t_cool      = 0;
  float p_oil_kpa   = 0;
  float p_oil_psi   = 0;
  bool  cel_on      = 0;
  uint16_t adc_bus  = 0;
  float v_bus       = 0;
  volatile bool flag_mainloop = 0;
  bool init_cpt     = 0;
  uint8_t coolPWMval = 0;
  bool flag_oil_low   = 0;
  bool flag_cool_hot  = 0;
  bool flag_v_low     = 0;

  // Counter variables
  uint16_t loopCount                    = 0;
  uint16_t count_end_adc                = COUNT_INTVL_V_BUS;
  volatile uint16_t count_intvl_tach    = TIMER1_COMPARE_VALUE;
  volatile uint16_t count_intvl_speed   = TIMER1_COMPARE_VALUE;
  volatile bool warn_blink_flag         = 0;

// Interrupt service routines (ISRs)
ISR(TIMER0_COMPA_vect) {
  flag_mainloop = 1;    // Set a flag for the main loop to do stuff
}

ISR(TIMER1_COMPA_vect) {
  // Tach interrupt
   if(rpm > MIN_RPM){         
    PORTB ^= (1 << PB7);        // Digital pin 11
   }
  OCR1A += count_intvl_tach;
}

ISR(TIMER1_COMPB_vect) {
  // Speedo interrupt
  if(mph > MIN_MPH){   
    PORTD ^= (1 << PD0);        // Digital pin 3 
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


void loop(){

  if(flag_mainloop == 1){

    flag_mainloop = 0;  


    if(loopCount >= INIT_LIGHT_ON_CTS && !init_cpt){
      // These lights turn on for INIT_LIGHT_ON_CTS and are then off by default
      // They may be turned on or off when CAN data is received 
      digitalWrite(PIN_ALT,      LIGHT_OFF);
      digitalWrite(PIN_SEATBELT, LIGHT_OFF);
      digitalWrite(PIN_SRS,      LIGHT_OFF);
      digitalWrite(PIN_CEL,      LIGHT_OFF);      
      digitalWrite(PIN_ABS,      LIGHT_OFF);
      digitalWrite(PIN_CRUISE,   LIGHT_OFF);
      init_cpt = 1;
    }

    // Check if there's CAN data available
    if(CAN_MSGAVAIL == CAN.checkReceive()){
      // If there's new data, read it
      digitalWrite(PIN_LED, HIGH);   // Set the LED pin high to measure CPU usage      
      
      // read data,  len: data length, buf: data buf
      CAN.readMsgBuf(&len, buf);                    
      unsigned long canId = CAN.getCanId();

      // Decode the correct message
      switch(canId){
        case ID_ENGINE_GENERAL_STATUS_1:
          // Decode engine RPM
          // uint16_t rpmInt = buf[1] << 8 | buf[2];
          rpm = (buf[1] << 8 | buf[2]) * 0.25;

          // Set new interval
          // counts_switch = t_switch_s * TIMER1FREQ
          // t_cycle_s = 1/Freq
          // t_switch_s = t_cycle_s/2
          // Freq = rpm * RPM2HZ
          // counts_switch = TIMER1FREQ/(2*rpm*RPM2HZ)
          if(rpm > MIN_RPM){
            count_intvl_tach = TIMER1_FREQ/(2*rpm*RPM2HZ);
          } 

          if(rpm > RPM_SHIFT){
            digitalWrite(PIN_CRUISE, LIGHT_ON);
          } else {
            digitalWrite(PIN_CRUISE, LIGHT_OFF);
          }

          break;

        case ID_VEHICLE_SPEED_AND_DISTANCE:
          // Decode vehicle speed
          kph = ((buf[0] & 0x7F) << 8 | buf[1]) * 0.015625;
          mph = kph * 0.6213712;

          // Set new interval
          // counts_switch = t_switch_s * TIMER1FREQ
          // t_cycle_s = 1/Freq
          // t_switch_s = t_cycle_s/2
          // Freq = rpm * RPM2HZ
          // counts_switch = TIMER1FREQ/(2*rpm*RPM2HZ)
          if(mph > MIN_MPH){
            count_intvl_speed = TIMER1_FREQ/(2*mph*MPH2HZ);
          }             
          break;

        case ID_ENGINE_GENERAL_STATUS_4:
          // Decode engine coolant temp, degC
          t_cool = buf[2] - 40;

          // Constrain the min val to 0 to make the math easier
          if(t_cool < 0){
            t_cool = 0;
          }

          flag_cool_hot = 0;

          // If we're below the low value
          if(t_cool < COOL_T_LOW){
            // Lineraly interpolate between 0 and COOL_PWM_LOW
            coolPWMval = (t_cool / COOL_T_LOW) * (COOL_PWM_LOW) ;

            // If we're above the low value and below the hot value
          } else if ((t_cool > COOL_T_LOW) && (t_cool < COOL_T_HIGH)){
            // Linearly interpolate between COOL_LOW and COOL_HIGH
            coolPWMval = COOL_PWM_LOW + 
                      (t_cool - COOL_T_LOW) *
                      (COOL_PWM_HIGH - COOL_PWM_LOW) / (COOL_T_HIGH - COOL_T_LOW) ;

            // Otherwise we're above the hot value
          } else {
            coolPWMval = COOL_PWM_HIGH + 
                      (t_cool - COOL_T_HIGH) *
                      (COOL_T_HIGHHIGH - COOL_PWM_HIGH) / (COOL_T_HIGHHIGH - COOL_T_HIGH); 
            flag_cool_hot = 1;
          }

          // Write PWM value
          analogWrite(PIN_COOLANT, coolPWMval);
          break;

        case ID_ENGINE_GENERAL_STATUS_5:
          // Decode engine oil pressure
          p_oil_kpa = buf[2] * 4;
          p_oil_psi = buf[2] * 0.580152;
          cel_on    = (buf[6] & 0x4) >> 2;

          // Set warning lights based on new data
          // Oil pressure is excluded from init sequence
          // Set the threshold based on rpm
          float thresh = P_OIL_LOW1_PSI;
          if(rpm > P_OIL_LOW_RPM){
            thresh = P_OIL_LOW2_PSI;
          } 

          if(p_oil_psi < thresh){
            digitalWrite(PIN_OILP, LIGHT_ON);
            if(rpm > 100){flag_oil_low = 1;}
          } else {
            digitalWrite(PIN_OILP, LIGHT_OFF);
            flag_oil_low = 0;
          }

          if(init_cpt){
            if(cel_on){
              digitalWrite(PIN_CEL, LIGHT_ON);
            } else {
              digitalWrite(PIN_CEL, LIGHT_OFF);
            }
          }
          break;

        } // End decode CAN data     
    // End RX CAN data
    // If we haven't received CAN data and it's time to read bus voltage, read and transmit it       
    } else if ( (loopCount - count_end_adc) >= 0 && (loopCount - count_end_adc) < 2*COUNT_INTVL_V_BUS) {
      
      digitalWrite(PIN_LED, HIGH);   // Set the LED pin high to measure CPU usage

      count_end_adc += COUNT_INTVL_V_BUS;
      adc_bus = analogRead(A0);
      v_bus = adc_bus * ADC2VBUS;

      uint8_t tx[2] = {(adc_bus & 0xFF00) >> 8, (adc_bus & 0x00FF)};

      // TODO
      // Transmit it over CAN  
      // CAN.sendMsgBuf(ID_V_BUS, 0, 0, 2, tx);


      if(init_cpt){
        if(v_bus <= V_BUS_LOW){
          digitalWrite(PIN_ALT, LIGHT_ON);
          if(rpm > 100){flag_v_low = 1;} else {flag_v_low = 0;}
        } else {
          digitalWrite(PIN_ALT, LIGHT_OFF);
        }
      }

    #ifdef DEBUG
        // Serial.println("-----------------------------");
        // Serial.print("Get data from ID: ");
        // Serial.println(canId, HEX);

        // for(int i = 0; i<len; i++)    // print the data
        // {
        //     Serial.print(buf[i], HEX);
        //     Serial.print("\t");
        // }
        // Serial.println();
        Serial.print("RPM = "); Serial.print(rpm); Serial.print(", ");
        Serial.print("mph = "); Serial.print(mph); Serial.print(", ");
        Serial.print("t_cool = "); Serial.print(t_cool); Serial.print(", ");
        Serial.print("p_oil = "); Serial.print(p_oil_psi); Serial.print(", ");
        Serial.print("cel_on = "); Serial.print(cel_on); Serial.print(", ");
        Serial.print("v_bus = "); Serial.print(v_bus); Serial.print(", ");
        Serial.print("Flags for oil, cool, voltage: ");Serial.print(flag_oil_low); Serial.print(",");Serial.print(flag_cool_hot);Serial.print(",");Serial.println(flag_v_low);
      #endif          
    }

 
    if( flag_oil_low | flag_cool_hot | flag_v_low ){
      warn_blink_flag = 1;
    } else {
      warn_blink_flag = 0;
      digitalWrite(PIN_SRS, LIGHT_OFF);
    }
 
    digitalWrite(PIN_LED, LOW);     

    loopCount += 1;   // Increment the loop counter

  } // End if mainloop flag
} // End mainloop


// END FILE
