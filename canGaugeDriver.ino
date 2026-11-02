#include "src/Longan_CAN_MCP2515/mcp_can.h"
#include "src/typedefs.h"
#include "src/utils/utils.h"
#include "src/hwdefs.h"
#include "src/limits.h"
#include "src/timers.h"
#include "src/can.h"
#include "src/interrupts.h"
#include "src/outputs.h"
#include <SPI.h>

#define CPU_LED_MAINLOOP
#define CPU_LED_TX1
//#define DEBUG    


/*
GaugeDriver
Program for a Seeed CANBed to recieve CAN data from a GM ECU
and drive a NB Miata tach, speedo, coolant gauge, and lights
Also, send the appropriate CAN messages to command the AC compressor

Timer0 sets a flag at MAIN_INT_FREQ_HZ; each tick the main loop:
  - Reads one CAN message (if available) and decodes it into engData (can.cpp)

  - At FREQ_V_BUS_HZ
    - Reads inputs (bus voltage, fuel level, CEL pin, AC request), transmits bus voltage over CAN

  - At FREQ_V_BUS_HZ, offset by half an interval
    - Transmits Platform General Status (AC compressor request)

  - At FREQ_1HZ
    - Transmits Platform Configuration Data (AC compressor type)

  - Sets all outputs from engData (outputs.cpp): lights, coolant PWM,
    tach/speedo intervals and enables, warning flags

Timer1 is used to drive variable frequency outputs
  - TIMER1 COMPA is the tach
    - If tach_enable, interrupt toggles PIN_TACH, else holds it low
    - Always COMPA += count_intvl_tach
  - TIMER1 COMPB is the speedo
    - If speed_enable, interrupt toggles PIN_SPEED, else holds it low
    - Always COMPB += count_intvl_speed
  - TIMER1 COMPC blinks PIN_SRS while warn_blink_flag is set (any warning flag active)
*/
 

// Set CAN tranciever chip select pin
MCP_CAN CAN(SPI_CS_PIN);    

 // Define all the variables
unsigned char len = 0;
unsigned char buf[8];  
EngineData engData;

volatile bool flag_mainloop = 0;

// Counter variables
uint16_t loopCount                    = 0;
uint16_t count_end_adc                = COUNT_INTVL_V_BUS;    // Runs at FREQ_V_BUS_HZ
uint16_t count_end_patform_gen_stat   = COUNT_INTVL_V_BUS/2;  // Same rate, just offset by half an interval
uint16_t count_end_platform_cfg_data  = COUNT_INTVL_1HZ - 1;  // Offset this interval by 1 so all the events don't happen at once
volatile uint16_t count_intvl_tach    = TIMER1_COMPARE_VALUE;
volatile uint16_t count_intvl_speed   = TIMER1_COMPARE_VALUE;
volatile bool tach_enable             = 0;    // Set in main loop, read by the tach ISR
volatile bool speed_enable            = 0;    // Set in main loop, read by the speedo ISR
volatile bool warn_blink_flag         = 0;


void setup()
{
    Serial.begin(115200);       // USB serial port
    
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
    pinMode(PIN_CEL_OUT,  OUTPUT);
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

    CAN.init_Mask(0, 0, 0x7FF);        // All bits must match
    CAN.init_Mask(1, 0, 0x7FF);        // All bits must match

    // Filters for RXB0 (RXF0-1)
    CAN.init_Filt(0, 0, ID_ENGINE_GENERAL_STATUS_1);
    CAN.init_Filt(1, 0, ID_VEHICLE_SPEED_AND_DISTANCE);

    // Filters for RXB1 (RXF2-5)
    CAN.init_Filt(2, 0, ID_ENGINE_GENERAL_STATUS_4);
    CAN.init_Filt(3, 0, ID_ENGINE_GENERAL_STATUS_5);
    CAN.init_Filt(4, 0, ID_ENGINE_GENERAL_STATUS_5);   // Duplicate just to blank out this filter
    CAN.init_Filt(5, 0, ID_ENGINE_GENERAL_STATUS_5);   // Duplicate just to blank out this filter

    // Set up the timer0 interrupt to run at MAIN_INT_FREQ_HZ
    timer0_init();

    // Congifure timer1
    timer1_init();

    // Enable global interrupts
    sei();
}



void loop(){

  if(flag_mainloop == 1){     // Main loop flag is set in TIMER0 interrupt 

    flag_mainloop = 0;  

    #ifdef CPU_LED_MAINLOOP
      digitalWrite(PIN_LED, HIGH);
    #endif
  
    // Check if there's CAN data available
    if(CAN_MSGAVAIL == CAN.checkReceive()){
      // If there's new data, read it. len: data length, buf: data buf
      CAN.readMsgBuf(&len, buf);                    
      unsigned long canId = CAN.getCanId();

      // update engData based on the message
      handle_received_message(canId, buf, &engData);

    #ifdef CPU_LED_MAINLOOP
      digitalWrite(PIN_LED, LOW);     
    #endif         
    }

    // If it's time to read bus voltage, read and transmit it 
    if (is_due(loopCount, &count_end_adc, COUNT_INTVL_V_BUS)) {

      #ifdef CPU_LED_TX1
        digitalWrite(PIN_LED, HIGH);
      #endif

      // Read bus voltage and gas
      engData.v_bus.volts = analogRead(ADC_VBUS)* ADC2VBUS;

      // Read fuel level
      engData.gas_gallons = interp1f(analogRead(ADC_GAS), gasGaugeCounts, gasGaugeGallons, gasNumPts);
      engData.gas_perc = engData.gas_gallons / GAS_CAPACITY;

      // Read the check engine light pin 
      engData.CEL = digitalRead(PIN_CEL_IN);

      // Read AC compressor request
      engData.AC_req = digitalRead(PIN_AC_REQ);
            
      // Transmit it over CAN  
      CAN.sendMsgBuf(ID_V_BUS, 0, 0, 4, engData.v_bus.bytes);


    #ifdef DEBUG
        Serial.print("RPM = ");    Serial.print(engData.rpm);         Serial.print(", ");
        Serial.print("mph = ");    Serial.print(engData.mph);         Serial.print(", ");
        Serial.print("t_cool = "); Serial.print(engData.t_cool);      Serial.print(", ");
        Serial.print("p_oil = ");  Serial.print(engData.p_oil_psi);   Serial.print(", ");
        Serial.print("CEL = ");    Serial.print(engData.CEL);         Serial.print(", ");
        Serial.print("AC_req = "); Serial.print(engData.AC_req);      Serial.print(", ");
        Serial.print("v_bus = ");  Serial.print(engData.v_bus.volts); Serial.print(", ");
        Serial.print("gas = ");    Serial.print(engData.gas_gallons); Serial.print(" "); Serial.print(engData.gas_perc); Serial.print(", ");
        Serial.print("Flags for oil, cool, voltage: ");
        Serial.print(engData.flag_oil_low);  Serial.print(",");
        Serial.print(engData.flag_cool_hot); Serial.print(",");
        Serial.print(engData.flag_v_low);
        Serial.println();
      #endif          
    }

    // If it's time, transmit Platform General Status (AC compressor request)
    if (is_due(loopCount, &count_end_patform_gen_stat, COUNT_INTVL_V_BUS)) {
    }

    // If it's time, transmit Platform Configuration Data (AC compressor type)
    if (is_due(loopCount, &count_end_platform_cfg_data, COUNT_INTVL_1HZ)) {
    }

    // Set the outputs based on the data and measurements we've received
    set_outputs(&engData);

    #ifdef CPU_LED_TX1
        digitalWrite(PIN_LED, LOW);     
    #endif   

    loopCount += 1;   // Increment the loop counter

  } // End if mainloop flag
} // End mainloop
