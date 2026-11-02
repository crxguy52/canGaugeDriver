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


