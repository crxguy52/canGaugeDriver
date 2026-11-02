#define COOL_T_COLD         0       // Temperature at and below which the coolant gauge is fully off
#define COOL_PWM_COLD       0       // PWM value for a fully-off (cold) coolant gauge
#define COOL_T_LOW          80      // Temperature at which coolant gauge should be on the L line
#define COOL_PWM_LOW        50      // PWM value to make coolant gauge read L
#define COOL_T_HIGH         120     // Temperature at which coolant gauge should be on H line
#define COOL_PWM_HIGH       200     // PWM value to make coolant gauge read H
#define COOL_T_HIGHHIGH     200     // Temp at which coolant gauge is pegged H
#define COOL_PWM_HIGHHIGH   255     // PWM value to peg coolant gauge past H (placeholder: full scale, not calibrated)

// Value Thresholds
// LFX spec is 10psi at idle, 30psi at 2,000 RPM
#define P_OIL_LOW_RPM         2000    // Below this RPM low1 is used, above low2        
#define P_OIL_LOW1_PSI        5
#define P_OIL_LOW2_PSI        20
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
