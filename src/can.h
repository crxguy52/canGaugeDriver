#pragma once
#include "Arduino.h"
#include "hwdefs.h"
#include "typedefs.h"
#include "limits.h"
#include "timers.h"

void handle_received_message(const unsigned long canId, const unsigned char *buf, EngineData *engData);

extern EngineData engData;
extern volatile uint16_t count_intvl_tach;
extern volatile uint16_t count_intvl_speed;
extern volatile bool tach_enable;
extern volatile bool speed_enable;
