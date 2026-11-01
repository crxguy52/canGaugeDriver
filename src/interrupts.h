#include "Arduino.h"
#include "typedefs.h"
#include <avr/interrupt.h>
#include "hwdefs.h"
#include "limits.h"
#include "timers.h"

extern volatile bool flag_mainloop;
extern EngineData engData;
extern volatile uint16_t count_intvl_tach;
extern volatile uint16_t count_intvl_speed;
extern volatile bool warn_blink_flag;
