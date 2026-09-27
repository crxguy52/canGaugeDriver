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

// Define minimum values that can will enable the output
// Make it larger than the theoretical minimum to keep update rates reasonalbe, since it only updates
// When the timer interrupts happen
//#define MIN_RPM               TIMER1_FREQ/(2*(pow(2, 16)-1)*RPM2HZ)
//#define MIN_MPH               TIMER1_FREQ/(2*(pow(2, 16)-1)*MPH2HZ)
#define MIN_RPM               200
#define MIN_MPH               5
