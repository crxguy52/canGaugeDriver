#pragma once
#include "inttypes.h"

// Gauge PWM calibration, measured on the cluster with test_scripts/dutyTest (analogWrite)

// Coolant gauge, degC -> PWM. Driven by analogWrite on Timer3.
// The gauge face isn't linear in PWM, so a midpoint makes the needle linear in temperature from C to H.
#define COOL_T_COLD         0       // Temperature at and below which the coolant gauge is fully off
#define COOL_PWM_COLD       0       // PWM value for a fully-off (cold) coolant gauge
#define COOL_T_LOW          70      // Temperature at which coolant gauge should be on the C line
#define COOL_PWM_LOW        50      // PWM value to make coolant gauge read C
#define COOL_T_MID          ((COOL_T_LOW + COOL_T_HIGH) / 2)  // Temperature shown at gauge center
#define COOL_PWM_MID        103     // PWM value to put the coolant needle at center
#define COOL_T_HIGH         120     // Temperature at which coolant gauge should be on H line
#define COOL_PWM_HIGH       130     // PWM value to make coolant gauge read H
#define COOL_T_HIGHHIGH     140     // Temp at which coolant gauge is pegged H
#define COOL_PWM_HIGHHIGH   150     // PWM value to peg coolant gauge past H

// Fuel gauge, percent full -> PWM. Driven by the Timer4 ISRs.
#define FUEL_PCT_E          0
#define FUEL_PWM_E          120     // PWM value to make fuel gauge read E
#define FUEL_PCT_HALF       50
#define FUEL_PWM_HALF       178     // PWM value to make fuel gauge read 1/2
#define FUEL_PCT_F          100
#define FUEL_PWM_F          242     // PWM value to make fuel gauge read F

// Oil pressure gauge PWM, for when it's driven as a gauge instead of on/off.
// Currently PIN_OILP is digital: HIGH = needle at off, LOW = needle centered.
#define OIL_PWM_OFF         255     // Needle at the bottom stop
#define OIL_PWM_L           150     // Needle on the L line
#define OIL_PWM_QUARTER     120     // Needle at the quarter mark
#define OIL_PWM_FIRST_TICK  85      // Needle at the first tick
#define OIL_PWM_CENTER      0       // Needle centered

// Value Thresholds
// LFX spec is 10psi at idle, 30psi at 2,000 RPM
#define V_BUS_LOW             12.5
#define RPM_ENGINE_RUNNING    100     // Above this RPM the engine is considered running; warnings only blink while running
#define RPM_SHIFT             6800

// Define minimum values that can will enable the output
// Make it larger than the theoretical minimum to keep update rates reasonalbe, since it only updates
// When the timer interrupts happen
//#define MIN_RPM               TIMER1_FREQ/(2*(pow(2, 16)-1)*RPM2HZ)
//#define MIN_MPH               TIMER1_FREQ/(2*(pow(2, 16)-1)*MPH2HZ)
#define MIN_RPM               200
#define MIN_MPH               5


extern const float oilLimRPM[];
extern const float oilLimPsi[];
extern const uint8_t oilLimNpts;
