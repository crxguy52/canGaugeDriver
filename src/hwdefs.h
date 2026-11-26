#pragma once
#include "inttypes.h"

#define LIGHT_ON            HIGH    // Value to turn light on
#define LIGHT_OFF           LOW     // Value to turn light off

// Scaling Values
#define ADC2VBUS            0.01996 // Convert ADC reading to bus voltage
#define RPM2HZ              0.032   // RPM to outupt frequency (Hz)
#define MPH2HZ              2.221

// RX CAN message IDs
#define ID_ENGINE_GENERAL_STATUS_1    201   // Contains engine_rpm. Transmitted at 80hz
#define ID_VEHICLE_SPEED_AND_DISTANCE 1001  // Contains vehicle speed. Transmitted at 10hz
#define ID_ENGINE_GENERAL_STATUS_4    1217  // Contains eng_coolant_temp. Transmitted at 2hz
#define ID_ENGINE_GENERAL_STATUS_5    1233  // Contains eng_oil_pressure. Transmitted at 2hz

// TX CAN message IDs
#define ID_GAUGEDRIVER      0x780   // gaugedriver_status, see dbc/gaugeDriver.dbc
#define DLC_GAUGEDRIVER     4

// gaugedriver_status scaling (physical units per bit), must match the DBC
#define GD_SCALE_V_BUS      0.1     // V/bit
#define GD_SCALE_FUEL_GAL   0.1     // gal/bit
#define GD_SCALE_FUEL_PCT   1.0     // %/bit
#define GD_BIT_AC_REQ       0       // Bit position in byte 3
#define GD_BIT_CEL          1       // Bit position in byte 3

// ---- AC compressor: impersonate the Platform (BCM) so the ECM runs the compressor ----
// Uncomment to transmit Platform General Status and Platform Configuration Data.
// Leave commented out until the AC wiring is connected.
//#define AC_COMPRESSOR_ENABLE

#ifdef AC_COMPRESSOR_ENABLE
  // Layouts are from platform_general_status / platform_configuration_data in
  // test_scripts/gmlan_v1.8_custom.dbc (GMW8762 4.2.3.19, 4.2.3.23).
  // Every signal set here fits inside one byte, so a Motorola start bit maps to
  // byte = start / 8, LSB shift = (start % 8) - (length - 1).
  #define ID_PLATFORM_GEN_STATUS    0x1F1   // platform_general_status
  #define DLC_PLATFORM_GEN_STATUS   8
  #define ID_PLATFORM_CFG_DATA      0x4E9   // platform_configuration_data
  #define DLC_PLATFORM_CFG_DATA     8

  // platform_general_status signal positions
  #define PGS_BYTE_PWR_MODE         0       // system_power_mode, 2 bits
  #define PGS_SHIFT_PWR_MODE        0
  #define PGS_BYTE_AC               1       // AC compressor signals are all in byte 1
  #define PGS_SHIFT_AC_MODE_REQ     3       // ac_comp_mode_request, 2 bits
  #define PGS_SHIFT_AC_LOAD_INVALID 2       // ac_comp_norm_load_validity, 1 bit (0 = valid)
  #define PGS_SHIFT_AC_VDA          1       // ac_comp_sys_vda, 1 bit

  // platform_general_status signal values
  #define PWR_MODE_RUN              2
  #define AC_MODE_DISENGAGE_NOW     0       // Disengage Immediately
  #define AC_MODE_DISENGAGE         1       // Graceful disengage
  #define AC_MODE_ENGAGE            2
  #define AC_VDA_AVAILABLE          1

  // platform_configuration_data signal positions and values
  #define PCD_BYTE_AC_COMP_TYPE     1       // ac_compressor_type, 3 bits
  #define PCD_SHIFT_AC_COMP_TYPE    5
  #define AC_COMP_TYPE_FIXED_CLUTCHED 1     // Fixed Displacement Clutched
#endif

// Digital input polarity: pin level that means "active" (lamp on / AC requested)
// Set to LOW if the baseboard inverts the signal
#define CEL_IN_ACTIVE       LOW
#define AC_REQ_ACTIVE       LOW

// Pin assignments
#define SPI_CS_PIN          17 
#define MCP_PIN_INT         7   // MCP2515 interrupt pin

#define PIN_AC_REQ          0
#define PIN_CEL_IN          1
#define PIN_SRS             2   // Port D1
#define PIN_SPEED           3   // Port D0
#define PIN_CEL_OUT         4
#define PIN_COOLANT         5
#define PIN_ABS             6
#define PIN_SEATBELT        8
#define PIN_CRUISE          9
#define PIN_OILP            10
#define PIN_TACH            11    // Port B7
#define PIN_ALT             12
#define PIN_LED             13

#define ADC_VBUS 			A0
#define ADC_GAS 			A1
#define ADC_CEL 			A2
#define ADC_AC_CMD 			A3


/*
gallons		ohms	volts	mA		counts
13			5.62	0.12	22.2	25
12			7.63	0.17	22.0	34
11			9.65	0.21	21.8	43
10			13.69	0.29	21.4	60
9			18.16	0.38	21.0	78
8			23.47	0.48	20.5	99
7			29.63	0.59	20.0	121
6			35.42	0.69	19.6	142
5			41.65	0.80	19.1	163
4			55.92	1.01	18.1	207
3			64.41	1.13	17.6	232
2			81.17	1.35	16.6	276
1			108.81	1.65	15.2	339
0			111.04	1.68	15.1	343
*/
#define GAS_CAPACITY 		12.7
extern const float gasGaugeCounts[];
extern const float gasGaugeGallons[];
extern const uint8_t gasNumPts;
