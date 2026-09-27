#define LIGHT_ON            HIGH    // Value to turn light on
#define LIGHT_OFF           LOW     // Value to turn light off
#define CAN_500KBPS         16
#define CAN_1000KBPS        18

#define F_CPU               16000000

// Scaling Values
#define ADC2VBUS            0.01996 // Convert ADC reading to bus voltage
#define RPM2HZ              0.0167  // RPM to Rev/s (Hz)
#define MPH2HZ              1

// RX CAN message IDs
#define ID_ENGINE_GENERAL_STATUS_1    201   // Contains engine_rpm. Transmitted at 80hz
#define ID_VEHICLE_SPEED_AND_DISTANCE 1001  // Contains vehicle speed. Transmitted at 10hz
#define ID_ENGINE_GENERAL_STATUS_4    1217  // Contains eng_coolant_temp. Transmitted at 2hz
#define ID_ENGINE_GENERAL_STATUS_5    1233  // Contains eng_oil_pressure. Transmitted at 2hz

// TX CAN message IDs
#define ID_V_BUS            0x780

// Pin assignments
#define SPI_CS_PIN          17 
#define MCP_PIN_INT         7   // MCP2515 interrupt pin

#define PIN_UART_RX         0
#define PIN_UART_TX         1
#define PIN_SRS             2   // Port D1
#define PIN_SPEED           3   // Port D0
#define PIN_CEL             4
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
static const float gasGaugeCounts[] = 	{25,	34,	43,	60,	78,	99,	121,	142,	163,	207,	232,	276,	339,	343};
static const float gasGaugeGallons[] = 	{13,	12,	11,	10,	9,	8,	7,		6,		5,		4,		3,		2,		1,		0};
static const uint8_t gasNumPts = 14;
