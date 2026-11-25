#include "can.h"


static void decodeGenStatus1(const unsigned char *buf, EngineData *engData){
    // Decode engine RPM
    // uint16_t rpmInt = buf[1] << 8 | buf[2];
    engData->rpm = ((uint16_t)buf[1] << 8 | buf[2]) * 0.25;
}

static void decodeVehicleSpeedAndDistance(const unsigned char *buf, EngineData *engData){
    // Decode vehicle speed
    engData->kph = ((buf[0] & 0x7F) << 8 | buf[1]) * 0.015625;
    engData->mph = engData->kph * 0.6213712;
}

static void decodeEngGenStat4(const unsigned char *buf, EngineData *engData){

    // Decode engine coolant temp, degC
    engData->t_cool = buf[2] - 40;
}

static void decodeEngGenStat5(const unsigned char *buf, EngineData *engData){
    // Decode engine oil pressure
    engData->p_oil_kpa = buf[2] * 4;
    engData->p_oil_psi = buf[2] * 0.580152;
    engData->cel_on    = (buf[6] & 0x4) >> 2;
}

void handle_received_message(
    const unsigned long canId,
    const unsigned char *buf,
    EngineData *engData){
 
      // Decode the correct message
      switch(canId){
        case ID_ENGINE_GENERAL_STATUS_1:     
            decodeGenStatus1(buf, engData);
            break;

        case ID_VEHICLE_SPEED_AND_DISTANCE:
            decodeVehicleSpeedAndDistance(buf, engData);
            break;

        case ID_ENGINE_GENERAL_STATUS_4:
            decodeEngGenStat4(buf, engData);
            break;

        case ID_ENGINE_GENERAL_STATUS_5: 
            decodeEngGenStat5(buf, engData);
            break;

        }   
}




static uint8_t scale_to_u8(float value, float scale){
    // Convert a physical value to a rounded uint8 count, saturating at 0 and 255
    float counts = value / scale + 0.5;
    if (counts <= 0)   return 0;
    if (counts >= UINT8_MAX) return UINT8_MAX;
    return (uint8_t)counts;
}

void send_gaugedriver_status(MCP_CAN &can, const EngineData *engData){
    // Pack and transmit gaugedriver_status. Layout must match dbc/gaugeDriver.dbc
    //   byte 0: v_bus     (GD_SCALE_V_BUS)
    //   byte 1: fuel_gal  (GD_SCALE_FUEL_GAL)
    //   byte 2: fuel_pct  (GD_SCALE_FUEL_PCT)
    //   byte 3: bit GD_BIT_AC_REQ = AC_req, bit GD_BIT_CEL = CEL
    unsigned char txBuf[DLC_GAUGEDRIVER];

    txBuf[0] = scale_to_u8(engData->v_bus,       GD_SCALE_V_BUS);
    txBuf[1] = scale_to_u8(engData->gas_gallons, GD_SCALE_FUEL_GAL);
    txBuf[2] = scale_to_u8(engData->gas_perc,    GD_SCALE_FUEL_PCT);
    txBuf[3] = (engData->AC_req << GD_BIT_AC_REQ) | (engData->CEL << GD_BIT_CEL);

    can.sendMsgBuf(ID_GAUGEDRIVER, 0, 0, DLC_GAUGEDRIVER, txBuf);
}
