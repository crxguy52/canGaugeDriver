#include "can.h"
#include "utils/utils.h"
#include <util/atomic.h>

// The slowest gauge interval (at MIN_RPM / MIN_MPH) must fit in the 16-bit compare
// register, otherwise the float -> uint16_t conversion below is undefined
static_assert(TIMER1_FREQ / (2.0 * MIN_RPM * RPM2HZ) <= UINT16_MAX, "MIN_RPM too low for TIMER1_FREQ/RPM2HZ - tach interval overflows 16 bits");
static_assert(TIMER1_FREQ / (2.0 * MIN_MPH * MPH2HZ) <= UINT16_MAX, "MIN_MPH too low for TIMER1_FREQ/MPH2HZ - speedo interval overflows 16 bits");

static void decodeGenStatus1(const unsigned char *buf, EngineData *engData){
    // Decode engine RPM
    // uint16_t rpmInt = buf[1] << 8 | buf[2];
    engData->rpm = ((uint16_t)buf[1] << 8 | buf[2]) * 0.25;
}

static void decodeVehicleSpeedAndDistance(const unsigned char *buf, EngineData *engData){
    // Decode vehicle speed
    engData->kph = ((buf[0] & 0x7F) << 8 | buf[1]) * 0.015625;
    engData->mph = engData->kph * 0.6213712;
}

// Coolant gauge map, temperature (degC) -> PWM. interp1f clamps outside the table ends.
// interp1f requires coolTempPts to be strictly increasing; PWM values must stay within 0..255 (uint8_t).
static const float coolTempPts[] = {COOL_T_COLD, COOL_T_LOW,   COOL_T_HIGH,   COOL_T_HIGHHIGH};
static const float coolPWMPts[]  = {COOL_PWM_COLD, COOL_PWM_LOW, COOL_PWM_HIGH, COOL_PWM_HIGHHIGH};
static const uint8_t coolNumPts  = sizeof(coolTempPts) / sizeof(coolTempPts[0]);

static void decodeEngGenStat4(const unsigned char *buf, EngineData *engData){

    // Decode engine coolant temp, degC
    engData->t_cool = buf[2] - 40;

    engData->coolPWMval = interp1f(engData->t_cool, coolTempPts, coolPWMPts, coolNumPts);
    engData->flag_cool_hot = (engData->t_cool >= COOL_T_HIGH);
}

static void decodeEngGenStat5(const unsigned char *buf, EngineData *engData){
    // Decode engine oil pressure
    engData->p_oil_kpa = buf[2] * 4;
    engData->p_oil_psi = buf[2] * 0.580152;
    engData->cel_on    = (buf[6] & 0x4) >> 2;
}

void handle_received_message(
    const unsigned long canId,
    const unsigned char *buf,
    EngineData *engData){
 
      // Decode the correct message
      switch(canId){
        case ID_ENGINE_GENERAL_STATUS_1: 
        
            decodeGenStatus1(buf, engData);
            // Set new interval
            // counts_switch = t_switch_s * TIMER1FREQ
            // t_cycle_s = 1/Freq
            // t_switch_s = t_cycle_s/2
            // Freq = rpm * RPM2HZ
            // counts_switch = TIMER1FREQ/(2*rpm*RPM2HZ)
            // The ISRs only read these flags and intervals, never engData. The 16-bit
            // interval writes are atomic so an ISR can't read a half-written value.
            tach_enable = (engData->rpm > MIN_RPM);
            if(tach_enable){
                uint16_t intvl = TIMER1_FREQ/(2*engData->rpm*RPM2HZ);
                ATOMIC_BLOCK(ATOMIC_RESTORESTATE){ count_intvl_tach = intvl; }
            } 

            if(engData->rpm > RPM_SHIFT){
                digitalWrite(PIN_CRUISE, LIGHT_ON);
            } else {
                digitalWrite(PIN_CRUISE, LIGHT_OFF);
            }

            break;


        case ID_VEHICLE_SPEED_AND_DISTANCE:
            decodeVehicleSpeedAndDistance(buf, engData);


            speed_enable = (engData->mph > MIN_MPH);
            if(speed_enable){
            uint16_t intvl = TIMER1_FREQ/(2*engData->mph*MPH2HZ);
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE){ count_intvl_speed = intvl; }
            }             
            break;


        case ID_ENGINE_GENERAL_STATUS_4:
            decodeEngGenStat4(buf, engData);

            // Write PWM value
            analogWrite(PIN_COOLANT, engData->coolPWMval);
            break;


        case ID_ENGINE_GENERAL_STATUS_5: {

            decodeEngGenStat5(buf, engData);

            // Set warning lights based on new data
            // Set the threshold based on rpm
            float thresh = P_OIL_LOW1_PSI;
            if(engData->rpm > P_OIL_LOW_RPM){
            thresh = P_OIL_LOW2_PSI;
            }             

            if(engData->p_oil_psi < thresh){
                    digitalWrite(PIN_OILP, LIGHT_ON);
                    if(engData->rpm > RPM_ENGINE_RUNNING){
                        engData->flag_oil_low = 1;
                    }else{
                        engData->flag_oil_low = 0;
                    }
                        
                } else {
                    digitalWrite(PIN_OILP, LIGHT_OFF);
                    engData->flag_oil_low = 0;
                }

            break;
        }

        }   
}


