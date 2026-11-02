#include "src/Longan_CAN_MCP2515/mcp_can.h"
#include "src/typedefs.h"
#include "src/utils/utils.h"
#include "src/hwdefs.h"
#include "src/limits.h"
#include "src/timers.h"
#include "src/can.h"
#include "src/interrupts.h"
#include <SPI.h>

#define CPU_LED_MAINLOOP
#define CPU_LED_TX1
//#define DEBUG    


/*
GaugeDriver
Program for a Seeed CANBed to recieve CAN data from a GM ECU
and drive a NB Miata tach, speedo, coolant gauge, and lights
Also, send the appropriate CAN messages to command the AC compressor

Timer0 is used to run the main loop:
  - Checks for CAN messages at MAIN_INT_FREQ_HZ
    - Receives CAN messages
    - Turns lights on and off based on received values
    - Sets PWM value based on coolant value
    - Sets tachInterval and speedInterval
    
  - If no CAN messages available and at FREQ_V_BUS_HZ
    - Reads inputs, transmits values over CAN

  - Offset by FREQ_V_BUS_HZ/2
    - Transmits BCM message for compressor
    
  - At 1HZ
    - Transmits BCM message
  
Timer1 is used to drive variable frequency outputs
  - TIMER1 COMPA is the tach
    - If rpm > RPM_MIN, interrupt toggles PIN_TACH, sets COMPA += tachInterval
    - Else COMBA += tachInterval
  - TIMER1 COMPB is the speedo  -> Interrupt toggles PIN_SPEED, sets COMPB += speedInterval
      - If mph > MPH_MIN, interrupt toggles PIN_TACH, sets COMPB += tachInterval
      - Else COMPB += tachInterval
  - TIMER1 COMPC is used to blink a warning light in the event a param is out of range
*/
 

// Set CAN tranciever chip select pin
MCP_CAN CAN(SPI_CS_PIN);    

 // Define all the variables
unsigned char len = 0;
unsigned char buf[8];  
EngineData engData;
uint16_t adc_bus  = 0;
union {
  float volts;
  uint8_t bytes[4];
} v_bus;

volatile bool flag_mainloop = 0;

// Counter variables
uint16_t loopCount                    = 0;
uint16_t count_end_adc                = COUNT_INTVL_V_BUS;    // This is 10Hz
uint16_t count_end_patform_gen_stat   = COUNT_INTVL_V_BUS/2;  // Same 10Hz, just offset by half an interval
uint16_t count_end_platform_cfg_data  = COUNT_INTVL_1HZ - 1;  // Offset this interval by 1 so all the events don't happen at once
volatile uint16_t count_intvl_tach    = TIMER1_COMPARE_VALUE;
volatile uint16_t count_intvl_speed   = TIMER1_COMPARE_VALUE;
volatile bool warn_blink_flag         = 0;


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



void loop(){

  if(flag_mainloop == 1){     // Main loop flag is set in TIMER0 interrupt 

    flag_mainloop = 0;  

    #ifdef CPU_LED_MAINLOOP
      digitalWrite(PIN_LED, HIGH);
    #endif
  
    // Check if there's CAN data available
    if(CAN_MSGAVAIL == CAN.checkReceive()){
      // If there's new data, read it          
      // read data,  len: data length, buf: data buf
      CAN.readMsgBuf(&len, buf);                    
      unsigned long canId = CAN.getCanId();

      // do stuff with the received data
      handle_received_message(canId, buf, &engData);

    #ifdef CPU_LED_MAINLOOP
      digitalWrite(PIN_LED, LOW);     
    #endif
             
    }

    // If it's time to read bus voltage, read and transmit it 
    if ( (loopCount - count_end_adc) >= 0 && (loopCount - count_end_adc) < 2*COUNT_INTVL_V_BUS) {
      
      #ifdef CPU_LED_TX1
        digitalWrite(PIN_LED, HIGH);
      #endif

      count_end_adc += COUNT_INTVL_V_BUS;

      // Read bus voltage and gas
      v_bus.volts = analogRead(ADC_VBUS)* ADC2VBUS;

      // Read fuel level
      engData.gas_gallons = interp1f(analogRead(ADC_GAS), gasGaugeCounts, gasGaugeGallons, gasNumPts);
      engData.gas_perc = engData.gas_gallons / GAS_CAPACITY;

      // Read the check engine light pin 
      engData.CEL = digitalRead(PIN_CEL_IN);
      if(engData.CEL){
        digitalWrite(PIN_CEL_OUT, LIGHT_ON);    
      }else{
        digitalWrite(PIN_CEL_OUT, LIGHT_OFF);
      }

      // Read AC compressor request
      engData.AC_req = digitalRead(PIN_AC_REQ);
      
      
      // Transmit it over CAN  
      CAN.sendMsgBuf(ID_V_BUS, 0, 0, 4, v_bus.bytes);


      if(v_bus.volts <= V_BUS_LOW){
        digitalWrite(PIN_ALT, LIGHT_ON);
        if(engData.rpm > 100){engData.flag_v_low = 1;} else {engData.flag_v_low = 0;}
      } else {
        digitalWrite(PIN_ALT, LIGHT_OFF);
      }


    #ifdef DEBUG
        // Serial.println("-----------------------------");
        // Serial.print("Get data from ID: ");
        // Serial.println(canId, HEX);

        // Serial.println();
//        Serial.print("RPM = "); Serial.print(rpm); Serial.print(", ");
//        Serial.print("mph = "); Serial.print(mph); Serial.print(", ");
//        Serial.print("t_cool = "); Serial.print(t_cool); Serial.print(", ");
//        Serial.print("p_oil = "); Serial.print(p_oil_psi); Serial.print(", ");
//        Serial.print("cel_on = "); Serial.print(cel_on); Serial.print(", ");
//        Serial.print("v_bus = "); Serial.print(v_bus.volts); Serial.print(",\t\t");
//        Serial.print("gas = "); Serial.print(gas_gallons); Serial.print(" "); Serial.print(gas_perc);        
//        Serial.print("Flags for oil, cool, voltage: ");Serial.print(flag_oil_low); Serial.print(",");Serial.print(flag_cool_hot);Serial.print(",");Serial.print(flag_v_low);
        Serial.println();
      #endif          
    }

    // If it's time to read bus voltage, read and transmit it 
    if ( (loopCount - count_end_patform_gen_stat) >= 0 && (loopCount - count_end_patform_gen_stat) < 2*COUNT_INTVL_V_BUS) {}

    // If it's time to read bus voltage, read and transmit it 
    if ( (loopCount - count_end_platform_cfg_data) >= 0 && (loopCount - count_end_platform_cfg_data) < 2*COUNT_INTVL_1HZ) {}        

 
    if( engData.flag_oil_low | engData.flag_cool_hot | engData.flag_v_low ){
      warn_blink_flag = 1;
    } else {
      warn_blink_flag = 0;
      digitalWrite(PIN_SRS, LIGHT_OFF);
    }
 
    #ifdef CPU_LED_TX1
      digitalWrite(PIN_LED, LOW);     
    #endif   

    loopCount += 1;   // Increment the loop counter

  } // End if mainloop flag
} // End mainloop
