#include "outputs.h"
#include "utils/utils.h"
#include <util/atomic.h>
#include "limits.h"

// The slowest gauge interval (at MIN_RPM / MIN_MPH) must fit in the 16-bit compare
// register, otherwise the float -> uint16_t conversion below is undefined
static_assert(TIMER1_FREQ / (2.0 * MIN_RPM * RPM2HZ) <= UINT16_MAX, "MIN_RPM too low for TIMER1_FREQ/RPM2HZ - tach interval overflows 16 bits");
static_assert(TIMER1_FREQ / (2.0 * MIN_MPH * MPH2HZ) <= UINT16_MAX, "MIN_MPH too low for TIMER1_FREQ/MPH2HZ - speedo interval overflows 16 bits");

// Coolant gauge map, temperature (degC) -> PWM. interp1f clamps outside the table ends.
// interp1f requires coolTempPts to be strictly increasing; PWM values must stay within 0..255 (uint8_t).
static const float coolTempPts[] = {COOL_T_COLD,   COOL_T_LOW,   COOL_T_MID,   COOL_T_HIGH,   COOL_T_HIGHHIGH};
static const float coolPWMPts[]  = {COOL_PWM_COLD, COOL_PWM_LOW, COOL_PWM_MID, COOL_PWM_HIGH, COOL_PWM_HIGHHIGH};
static const uint8_t coolNumPts  = sizeof(coolTempPts) / sizeof(coolTempPts[0]);

// Fuel gauge map, percent full -> PWM. Same interp1f rules as the coolant map.
static const float fuelPctPts[]  = {FUEL_PCT_E, FUEL_PCT_HALF, FUEL_PCT_F};
static const float fuelPWMPts[]  = {FUEL_PWM_E, FUEL_PWM_HALF, FUEL_PWM_F};
static const uint8_t fuelNumPts  = sizeof(fuelPctPts) / sizeof(fuelPctPts[0]);
// fuel_duty is uint8_t and OCR4A is written with TC4H = 0, so TOP must fit in 8 bits.
// A table value above TOP would never match compare A and leave the pin stuck high.
static_assert(TIMER4_TOP <= UINT8_MAX, "TIMER4_TOP must fit in fuel_duty (uint8_t) and OCR4A's low byte");
static_assert(FUEL_PWM_E <= TIMER4_TOP && FUEL_PWM_HALF <= TIMER4_TOP && FUEL_PWM_F <= TIMER4_TOP, "Fuel PWM table exceeds TIMER4_TOP");


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
        digitalWrite(PIN_SHIFT, LIGHT_ON);
    } else {
        digitalWrite(PIN_SHIFT, LIGHT_OFF);
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

    // Fuel gauge. gas_perc is -1 until the first ADC read, which clamps to E.
    // OCR4A is double-buffered, so the new duty starts cleanly at the next period.
    uint8_t fuelPWM = interp1f(engData->gas_perc, fuelPctPts, fuelPWMPts, fuelNumPts);
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
        fuel_duty = fuelPWM;
        TC4H  = 0;              // OCR4A is 10-bit, TC4H holds the high bits
        OCR4A = fuelPWM;
    }

    // Oil pressure
    // Set the threshold based on rpm
    float oilThreshPsi = interp1f(engData->rpm, oilLimRPM, oilLimPsi, oilLimNpts);   

    if(engData->p_oil_psi < oilThreshPsi){
        digitalWrite(PIN_OILP, OIL_NO_PRESSURE);

        if(engData->rpm > RPM_ENGINE_RUNNING){
            engData->flag_oil_low = 1;
        } else {
            engData->flag_oil_low = 0;
        }
                
    } else {
        digitalWrite(PIN_OILP, OIL_PRESSURE);
        engData->flag_oil_low = 0;
    }

    // Check engine light
    if(engData->CEL){
        digitalWrite(PIN_CEL_OUT, LIGHT_ON);    
        } else {
        digitalWrite(PIN_CEL_OUT, LIGHT_OFF);
        }

    // Alternator light
    if(engData->v_bus <= V_BUS_LOW){
        digitalWrite(PIN_ALT, LIGHT_ON);
    } else {
        digitalWrite(PIN_ALT, LIGHT_OFF);
    }
    // Only blink while running, so low voltage with the engine off doesn't latch the warning
    engData->flag_v_low = (engData->v_bus <= V_BUS_LOW) && (engData->rpm > RPM_ENGINE_RUNNING);

    // Flash idiot light if any of these are true
    if( engData->flag_oil_low | engData->flag_cool_hot | engData->flag_v_low ){
        warn_blink_flag = 1;
    } else {
        warn_blink_flag = 0;
        digitalWrite(PIN_SRS, LIGHT_OFF);
    }
}



