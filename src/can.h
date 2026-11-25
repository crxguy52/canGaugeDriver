#pragma once
#include "Arduino.h"
#include "hwdefs.h"
#include "typedefs.h"
#include "limits.h"
#include "timers.h"

#include "Longan_CAN_MCP2515/mcp_can.h"

void send_gaugedriver_status(MCP_CAN &can, const EngineData *engData);
void handle_received_message(const unsigned long canId, const unsigned char *buf, EngineData *engData);
