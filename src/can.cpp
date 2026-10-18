#include "can.h"

static void decodeGenStatus1(const unsigned char *buf, EngineData *engData){
    // Decode engine RPM
    // uint16_t rpmInt = buf[1] << 8 | buf[2];
    engData->rpm = (buf[1] << 8 | buf[2]) * 0.25;
}

static void decodeVehicleSpeedAndDistance(const unsigned char *buf, EngineData *engData){
    // Decode vehicle speed
    engData->kph = ((buf[0] & 0x7F) << 8 | buf[1]) * 0.015625;
    engData->mph = engData->kph * 0.6213712;
}

static void decodeEngGenStat4(const unsigned char *buf, EngineData *engData){

    // Decode engine coolant temp, degC
    engData->t_cool = buf[2] - 40;

    // Constrain the min val to 0 to make the math easier
    if(engData->t_cool < 0){
    engData->t_cool = 0;
    }

    engData->flag_cool_hot = 0;

    // If we're below the low value
    if(engData->t_cool < COOL_T_LOW){
    // Lineraly interpolate between 0 and COOL_PWM_LOW
    engData->coolPWMval = (engData->t_cool / COOL_T_LOW) * (COOL_PWM_LOW) ;

    // If we're above the low value and below the hot value
    } else if ((engData->t_cool > COOL_T_LOW) && (engData->t_cool < COOL_T_HIGH)){
    // Linearly interpolate between COOL_LOW and COOL_HIGH
    engData->coolPWMval = COOL_PWM_LOW + 
            (engData->t_cool - COOL_T_LOW) *
            (COOL_PWM_HIGH - COOL_PWM_LOW) / (COOL_T_HIGH - COOL_T_LOW) ;

    // Otherwise we're above the hot value
    } else {
    engData->coolPWMval = COOL_PWM_HIGH + 
            (engData->t_cool - COOL_T_HIGH) *
            (COOL_T_HIGHHIGH - COOL_PWM_HIGH) / (COOL_T_HIGHHIGH - COOL_T_HIGH); 
    engData->flag_cool_hot = 1;
    }
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
            if(engData->rpm > MIN_RPM){
            // count_intvl_tach = TIMER1_FREQ/(2*engData->rpm*RPM2HZ);
            } 

            if(engData->rpm > RPM_SHIFT){
            digitalWrite(PIN_CRUISE, LIGHT_ON);
            } else {
            digitalWrite(PIN_CRUISE, LIGHT_OFF);
            }

            break;


        case ID_VEHICLE_SPEED_AND_DISTANCE:
            decodeVehicleSpeedAndDistance(buf, engData);


            if(engData->mph > MIN_MPH){
            // count_intvl_speed = TIMER1_FREQ/(2*engData->mph*MPH2HZ);
            }             
            break;


        case ID_ENGINE_GENERAL_STATUS_4:
            decodeEngGenStat4(buf, engData);

            // Write PWM value
            analogWrite(PIN_COOLANT, engData->coolPWMval);
            break;


        case ID_ENGINE_GENERAL_STATUS_5:

            decodeEngGenStat5(buf, engData);

            // Set warning lights based on new data
            // Oil pressure is excluded from init sequence
            // Set the threshold based on rpm
            float thresh = P_OIL_LOW1_PSI;
            if(engData->rpm > P_OIL_LOW_RPM){
            thresh = P_OIL_LOW2_PSI;
            }             

            if(engData->p_oil_psi < thresh){
                    digitalWrite(PIN_OILP, LIGHT_ON);
                    if(engData->rpm > 100){
                        engData->flag_oil_low = 1;
                        }
                } else {
                    digitalWrite(PIN_OILP, LIGHT_OFF);
                    engData->flag_oil_low = 0;
                }

            break;

        }   
}


