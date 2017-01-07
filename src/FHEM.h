
//
//    FILE: FHEM.h
//  AUTHOR: Michael AT Kloeffer DOT de
// VERSION: 0.1
//    DATE: 2017-jan-01
// PURPOSE: Connect ESP to FHEM
//


#ifndef FHEM_h
#define FHEM_h


#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include "Arduino.h"

class FHEM
{
public:
    boolean setReading(String deviceName, String readingName, String readingValue);
    boolean setReading(String deviceName, String reading, double readingValue, byte decimals);
    //explicit FHEM();
    //~FHEM();

};

#endif
// END OF FILE
