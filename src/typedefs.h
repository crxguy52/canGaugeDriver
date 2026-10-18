  #pragma once
  #include "Arduino.h"
  
  typedef struct{
    float rpm         = 0;
    float kph         = 0;
    float mph         = 0;
    float t_cool      = 0;
    float p_oil_kpa   = 0;
    float p_oil_psi   = 0;
    bool  cel_on      = 0;
    uint8_t coolPWMval = 0;
    bool flag_oil_low   = 0;
    bool flag_cool_hot  = 0;
    bool flag_v_low     = 0; 
    float gas_gallons   = -1;
    float gas_perc      = -1;       
  } EngineData;