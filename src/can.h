#pragma once
#include "Arduino.h"
#include "hwdefs.h"
#include "typedefs.h"
#include "limits.h"

void handle_received_message(const unsigned long canId, const unsigned char *buf, EngineData *engData);