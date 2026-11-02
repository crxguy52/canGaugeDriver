#include "outputs.h"
#include "utils/utils.h"
#include <util/atomic.h>

// The slowest gauge interval (at MIN_RPM / MIN_MPH) must fit in the 16-bit compare
// register, otherwise the float -> uint16_t conversion below is undefined
static_assert(TIMER1_FREQ / (2.0 * MIN_RPM * RPM2HZ) <= UINT16_MAX, "MIN_RPM too low for TIMER1_FREQ/RPM2HZ - tach interval overflows 16 bits");
static_assert(TIMER1_FREQ / (2.0 * MIN_MPH * MPH2HZ) <= UINT16_MAX, "MIN_MPH too low for TIMER1_FREQ/MPH2HZ - speedo interval overflows 16 bits");

// Coolant gauge map, temperature (degC) -> PWM. interp1f clamps outside the table ends.
// interp1f requires coolTempPts to be strictly increasing; PWM values must stay within 0..255 (uint8_t).
static const float coolTempPts[] = {COOL_T_COLD, COOL_T_LOW,   COOL_T_HIGH,   COOL_T_HIGHHIGH};
static const float coolPWMPts[]  = {COOL_PWM_COLD, COOL_PWM_LOW, COOL_PWM_HIGH, COOL_PWM_HIGHHIGH};
static const uint8_t coolNumPts  = sizeof(coolTempPts) / sizeof(coolTempPts[0]);


// Use the data we've measured or received and translate it into outputs
void set_outputs(EngineData *engData){

    // Tach interval
    tach_enable = (engData->rpm > MIN_RPM);
    if(tach_enable){
        uint16_t intvl = TIMER1_FREQ/(2*engData->rpm*RPM2HZ);
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE){ count_intvl_tach = intvl; }
    } 

    // Shift light
    if(engData->rpm > RPM_SHIFT){
        digitalWrite(PIN_CRUISE, LIGHT_ON);
    } else {
        digitalWrite(PIN_CRUISE, LIGHT_OFF);
    }

    // Sepeedo interval
    speed_enable = (engData->mph > MIN_MPH);
    if(speed_enable){
        uint16_t intvl = TIMER1_FREQ/(2*engData->mph*MPH2HZ);
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE){ count_intvl_speed = intvl; }
    }                
    // Coolant gauge 
    engData->coolPWMval = interp1f(engData->t_cool, coolTempPts, coolPWMPts, coolNumPts);
    engData->flag_cool_hot = (engData->t_cool >= COOL_T_HIGH);    
    analogWrite(PIN_COOLANT, engData->coolPWMval);

    // Oil pressure
    // Set the threshold based on rpm
    float thresh = P_OIL_LOW1_PSI;
    if(engData->rpm > P_OIL_LOW_RPM){
        thresh = P_OIL_LOW2_PSI;
    }             

    if(engData->p_oil_psi < thresh){
        digitalWrite(PIN_OILP, LIGHT_ON);

        if(engData->rpm > RPM_ENGINE_RUNNING){
            engData->flag_oil_low = 1;
        } else {
            engData->flag_oil_low = 0;
        }
                
    } else {
        digitalWrite(PIN_OILP, LIGHT_OFF);
        engData->flag_oil_low = 0;
    }

    // Check engine light
    if(engData->CEL){
        digitalWrite(PIN_CEL_OUT, LIGHT_ON);    
        } else {
        digitalWrite(PIN_CEL_OUT, LIGHT_OFF);
        }

    // Alternator light
    if(engData->v_bus.volts <= V_BUS_LOW){
        digitalWrite(PIN_ALT, LIGHT_ON);
    } else {
        digitalWrite(PIN_ALT, LIGHT_OFF);
    }
    // Only blink while running, so low voltage with the engine off doesn't latch the warning
    engData->flag_v_low = (engData->v_bus.volts <= V_BUS_LOW) && (engData->rpm > RPM_ENGINE_RUNNING);

    // Flash idiot light if any of these are true
    if( engData->flag_oil_low | engData->flag_cool_hot | engData->flag_v_low ){
        warn_blink_flag = 1;
    } else {
        warn_blink_flag = 0;
        digitalWrite(PIN_SRS, LIGHT_OFF);
    }
}



