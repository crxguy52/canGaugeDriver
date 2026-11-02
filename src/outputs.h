#pragma once
#include "Arduino.h"
#include "hwdefs.h"
#include "typedefs.h"
#include "limits.h"
#include "timers.h"


extern volatile uint16_t count_intvl_tach;
extern volatile uint16_t count_intvl_speed;
extern volatile bool tach_enable;
extern volatile bool speed_enable;
extern volatile bool warn_blink_flag;

void set_outputs(EngineData *engData);