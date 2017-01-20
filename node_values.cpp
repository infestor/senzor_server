#include <stdio.h>
#include "main.h"

extern const double TempCorrections[];

//=================================================================================
// ========= Sensor Data/value specific type dependent manipulation functions =====
//=================================================================================
void countDS1820Temp STORE_VALUE_PARAMS
{
    int TReading = rawData[0] + (rawData[1] << 8);
    
    uchar SignBit = ((TReading & 0x8000) >> 8);  // test most sig bit
    
    if (SignBit > 0) { // negative
        TReading = (TReading ^ 0xffff) + 1; // 2's comp
    }
    
    double result;
    result = (6.0 * TReading) + TReading / 4.0; // multiply by (100 * 0.0625) or 6.25
    result = result / 100.0;
    
    if (SignBit) { // If its negative
        result = result * (-1.0);
    }
    
    result -= TempCorrections[ nodeP->node ];
    
    nodeP->sensors[sensorNum]->float_val = result;
}

void countInternalProcTemp STORE_VALUE_PARAMS
{
    int TReading = rawData[0];
    nodeP->sensors[sensorNum]->float_val = (TReading - 128.0) / 1.07;
}

void decideOnOffValue STORE_VALUE_PARAMS
{
    int TReading = rawData[0];
    nodeP->sensors[sensorNum]->int_val = (TReading > 0) ? 1 : 0;
}

void countBatteryVoltOneCell STORE_VALUE_PARAMS
{
    int val = rawData[0] + (rawData[1] << 8);
    double volts = (5.0 / 1023.0) * val; //with 5V reference
    nodeP->sensors[sensorNum]->float_val = volts;
}

void countBatteryVoltTwoCell STORE_VALUE_PARAMS
{
    int val = rawData[0] + (rawData[1] << 8);
    double volts = (5.0 / 1023.0) * val * 2; //cell voltage divisor on circuit is 2
    nodeP->sensors[sensorNum]->float_val = volts;
}

void countLowPowerVcc STORE_VALUE_PARAMS
{
    int TReading = rawData[2];
    nodeP->low_power_voltage = TReading * 20;
}

void getFloatValStr GET_VALUE_PARAMS
{
    *strLen = asprintf( (char**)strBuf, "%.1f", nodeP->sensors[sensorNum]->float_val );
}

void getIntValStr GET_VALUE_PARAMS
{
    *strLen = asprintf( (char**)strBuf, "%i", nodeP->sensors[sensorNum]->int_val );
}

void getUintValStr GET_VALUE_PARAMS
{
    *strLen = asprintf( (char**)strBuf, "%d", nodeP->sensors[sensorNum]->uint_val );
}

void getUcharValStr GET_VALUE_PARAMS
{
    *strLen = asprintf( (char**)strBuf, "%d", nodeP->sensors[sensorNum]->uchar_val );
}
