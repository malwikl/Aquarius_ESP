/*
ESP8266 : NTP Time + Timezone
Created 07 May 2016 by Ralf Bohnen - www.Arduinoclub.de
This example code is in the public domain.
*/

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <TimeLib.h>  //by Paul Stoffregen, not included in the Arduino IDE !!!
#include <Timezone.h> //by Jack Christensen, not included in the Arduino IDE !!!
#include <MQ135.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <RunningAverage.h>
#include "FHEM.h"
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include "WiFiManager.h"          //https://github.com/tzapu/WiFiManager


#define DBG_OUTPUT_PORT Serial
#define DHTTYPE DHT22
#define DHTPIN 14

unsigned long previousMillis = 0;
unsigned long previousMeassMillis = 0;
unsigned long previousSendMillis = 0;
const long interval = 1000;       // Update Clock each second
const long meassInterval = 10000; //Measure each 10 seconds
const long sendInterval = 300000; //Send only each 300 seconds to FHEM
float lastTemp = 0.0;
float lastHum = 0.0;

RunningAverage myRA(10);
FHEM myFHEM;

volatile bool wasConnected = false;
byte icon_heart[] = {0x00,0xa,0x1f,0x1f,0xe,0x4,0x00,0x00};
byte icon_house[] = {0x04,0x0E,0x1F,0x11,0x11,0x11,0x1F,0x00};
byte icon_tree[] = {0x0E,0x1F,0x1F,0x0E,0x04,0x04,0x1F,0x00};
byte icon_fish[] = {0x00,0x00,0x0D,0x13,0x13,0x0D,0x00,0x00};
byte icon_cloud[] = {0x0C,0x12,0x1E,0x00,0x06,0x09,0x0F,0x00};

char s_dec[5];

LiquidCrystal_I2C lcd(0x3F, 20, 4);
DHT dht(DHTPIN, DHTTYPE);
MQ135 mq135_sensor = MQ135(0);

void configModeCallback (WiFiManager *myWiFiManager) {
  lcd.clear();
  lcd.setCursor(0, 0);
  Serial.println("Entered config mode");
  lcd.print("Entered config mode");
  lcd.setCursor(0, 1);
  Serial.println(WiFi.softAPIP());
  lcd.print("my IP: ");
  lcd.print(WiFi.softAPIP());
  //if you used auto generated SSID, print it
  lcd.setCursor(0, 2);
  Serial.println(myWiFiManager->getConfigPortalSSID());
  lcd.print("AP: ");
  lcd.print(myWiFiManager->getConfigPortalSSID());
}


bool isConnected(long timeOutSec) {
  timeOutSec = timeOutSec * 1000;
  int z = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(200);
    //DBG_OUTPUT_PORT.print(".");
    if (z == timeOutSec / 200) { return false; }
    z++;
  }
  return true;
}

//### NTP, TIME & TIMEZONE ###################################################################


//UDP
WiFiUDP Udp;
unsigned int localPort = 123;

//NTP Server
char ntpServerName1[] = "ntp1.t-online.de";
char ntpServerName2[] = "time.nist.gov";

//Timezone
//Central European Time (Frankfurt, Paris)
TimeChangeRule CEST = { "CEST", Last, Sun, Mar, 2, 120 };     //Central European Summer Time
TimeChangeRule CET = { "CET ", Last, Sun, Oct, 3, 60 };       //Central European Standard Time
Timezone CE(CEST, CET);
TimeChangeRule *tcr;        //pointer to the time change rule, use to get the TZ abbrev
time_t utc, local;


const int NTP_PACKET_SIZE = 48; // NTP time is in the first 48 bytes of message
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets

bool getNtpTime(char* ntpServerName)
{
  //DBG_OUTPUT_PORT.print(F("NTP request..."));
  if (timeStatus() == timeSet) {
    //DBG_OUTPUT_PORT.println(F("not necessary"));
    return true;
  }

  IPAddress ntpServerIP; // NTP server's ip address

  while (Udp.parsePacket() > 0); // discard any previously received packets
  DBG_OUTPUT_PORT.println(F("Transmit NTP Request"));
  // get a random server from the pool
  WiFi.hostByName(ntpServerName, ntpServerIP);
  DBG_OUTPUT_PORT.print(ntpServerName);
  DBG_OUTPUT_PORT.print(": ");
  DBG_OUTPUT_PORT.println(ntpServerIP);
  sendNTPpacket(ntpServerIP);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500) {
    int size = Udp.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      DBG_OUTPUT_PORT.println(F("Receive NTP Response"));
      Udp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
      unsigned long secsSince1900;
      // convert four bytes starting at location 40 to a long integer
      secsSince1900 = (unsigned long)packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long)packetBuffer[43];
      setTime(secsSince1900 - 2208988800UL);
      //setTime(23, 55, 0, 30, 3, 2016); //simulate time for test
      return true;
    }
  }
  DBG_OUTPUT_PORT.println(F("FATAL ERROR : No NTP Response."));
  return false; // return 0 if unable to get the time
}

// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress &address)
{
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12] = 49;
  packetBuffer[13] = 0x4E;
  packetBuffer[14] = 49;
  packetBuffer[15] = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}

//Function to return the compile date and time as a time_t value
time_t compileTime(void)
{
  #define FUDGE 25        //fudge factor to allow for compile time (seconds, YMMV)
  char *compDate = __DATE__, *compTime = __TIME__, *months = "JanFebMarAprMayJunJulAugSepOctNovDec";
  char chMon[3], *m;
  int d, y;
  tmElements_t tm;
  time_t t;
  strncpy(chMon, compDate, 3);
  chMon[3] = '\0';
  m = strstr(months, chMon);
  tm.Month = ((m - months) / 3 + 1);
  tm.Day = atoi(compDate + 4);
  tm.Year = atoi(compDate + 7) - 1970;
  tm.Hour = atoi(compTime);
  tm.Minute = atoi(compTime + 3);
  tm.Second = atoi(compTime + 6);
  t = makeTime(tm);
  return t + FUDGE;        //add fudge factor to allow for compile time
}

void printTime(time_t t)
{
  sPrintI00(hour(t));
  sPrintDigits(minute(t));
  sPrintDigits(second(t));
  DBG_OUTPUT_PORT.print(' ');
  DBG_OUTPUT_PORT.print(dayShortStr(weekday(t)));
  DBG_OUTPUT_PORT.print(' ');
  sPrintI00(day(t));
  DBG_OUTPUT_PORT.print(' ');
  DBG_OUTPUT_PORT.print(monthShortStr(month(t)));
  DBG_OUTPUT_PORT.print(' ');
  DBG_OUTPUT_PORT.print(year(t));
  DBG_OUTPUT_PORT.println(' ');
}

void printTime_LCD(time_t t) {
  byte b_second, b_minute,b_hour, b_day, b_month;
  char timeString[21];
  sprintf(timeString, "%02d.%02d.%4d  %02d:%02d:%02d", day(t), month(t),year(t),hour(t),minute(t),second(t));
  sprintf(timeString, "%-*s", 20, timeString);
  lcd.print(timeString);
}

//Print an integer in "00" format (with leading zero).
//Input value assumed to be between 0 and 99.
void sPrintI00(int val)
{
  if (val < 10) DBG_OUTPUT_PORT.print('0');
  DBG_OUTPUT_PORT.print(val, DEC);
  return;
}

//Print an integer in ":00" format (with leading zero).
//Input value assumed to be between 0 and 99.
void sPrintDigits(int val)
{
  DBG_OUTPUT_PORT.print(':');
  if (val < 10) DBG_OUTPUT_PORT.print('0');
  DBG_OUTPUT_PORT.print(val, DEC);
}
//############################################################################################


void setup()
{
  lcd.begin();
  lcd.backlight();
  lcd.createChar(0, icon_heart);
  lcd.createChar(1, icon_house);
  lcd.createChar(2, icon_tree);
  lcd.createChar(3, icon_fish);
  lcd.createChar(4, icon_cloud);

  dht.begin();
  lcd.clear();

  WiFiManager wifiManager;
  //reset settings - for testing
  //wifiManager.resetSettings();

  //set callback that gets called when connecting to previous WiFi fails, and enters Access Point mode
  wifiManager.setAPCallback(configModeCallback);

  //fetches ssid and pass and tries to connect
  //if it does not connect it starts an access point with the specified name
  //here  "AutoConnectAP"
  //and goes into a blocking loop awaiting configuration
  if(!wifiManager.autoConnect()) {
    Serial.println("failed to connect and hit timeout");
    //reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(1000);
  }

  lcd.setCursor(0, 0);
  lcd.print("Starting...");
  DBG_OUTPUT_PORT.begin(115200);
  DBG_OUTPUT_PORT.setDebugOutput(false);
  DBG_OUTPUT_PORT.println(F("ArduinoClub-NTP-Timezone"));


  if (isConnected(30)) {
    wasConnected = true;
    lcd.setCursor(0, 0);
    lcd.print("Connected!");
    DBG_OUTPUT_PORT.println(F("Starting UDP"));
    Udp.begin(localPort);
    DBG_OUTPUT_PORT.print(F("Local port: "));
    DBG_OUTPUT_PORT.println(Udp.localPort());
    DBG_OUTPUT_PORT.println(F("waiting for sync"));

  }


  /* add setup code here */

  lcd.clear();
}

void loop()
{
  /* Temp & Hum vom DHT22 */

  unsigned long currentMillis = millis();
  if (currentMillis - previousMeassMillis >= meassInterval) {
    previousMeassMillis = currentMillis;
    //lcd.clear();
    lcd.setCursor(0, 1);

    float h = dht.readHumidity();
    float t = dht.readTemperature();
    char bufferString[21];
    char _h[7], _t[7], _co2[7];

    //Check if reading DHT worked, otherwise use last correct measured values
    if (isnan(h) || isnan(t)) {
      h = lastHum;
      t = lastTemp;
    } else {
      lastHum = h;
      lastTemp = t;
    }

    dtostrf(h, 4, 1, _h);
    dtostrf(t, 4, 1, _t);
    sprintf(bufferString, "%c %s%cC %s%%", (char)1, _t,(char)223, _h);
    sprintf(bufferString, "%-*s", 20, bufferString);
    lcd.print(bufferString);
    Serial.println(bufferString);

    /* MQ135 */
    float rzero = mq135_sensor.getRZero();
    float correctedRZero = mq135_sensor.getCorrectedRZero(t, h);
    float resistance = mq135_sensor.getResistance();
    float ppm = mq135_sensor.getPPM();
    float correctedPPM = mq135_sensor.getCorrectedPPM(t, h);
    Serial.print("MQ135 RZero: ");
    Serial.print(rzero);
    Serial.print("\t Corrected RZero: ");
    Serial.print(correctedRZero);
    Serial.print("\t Resistance: ");
    Serial.print(resistance);
    Serial.print("\t PPM: ");
    Serial.print(ppm);
    Serial.print("\t Corrected PPM: ");
    Serial.print(correctedPPM);
    Serial.println("ppm");

    myRA.addValue(correctedPPM);


    if (currentMillis - previousSendMillis >= sendInterval) {
      previousSendMillis = currentMillis;
      myFHEM.setReading("TESTMK", "humidity", h, 1);
      myFHEM.setReading("TESTMK", "temperature", t, 1);
      myFHEM.setReading("TESTMK", "ppm", correctedPPM, 0);
      myFHEM.setReading("TESTMK", "rawppm", ppm, 0);
      myFHEM.setReading("TESTMK", "rawrzero", rzero, 0);
      myFHEM.setReading("TESTMK", "rzero", correctedRZero, 0);
      myFHEM.setReading("TESTMK", "co2", correctedPPM, 0);
    }

    lcd.setCursor(0, 3);
    dtostrf(myRA.getAverage(), 1, 0, _co2);
    sprintf(bufferString, "%c %i Avg: %s", (char)4, (int)correctedPPM, _co2);
    sprintf(bufferString, "%-*s", 20, bufferString);
    lcd.print(bufferString);

    /* Aqua */
    lcd.setCursor(0,2);
    lcd.write(3);
    lcd.print(" ");
    lcd.print("tbd");
  }

  /* Date & Time */
  //Serial.print(WiFi.status());
  if (WiFi.status() != WL_CONNECTED) {
    lcd.setCursor(0, 0);
    lcd.print("No Wifi!");
  } else {
    lcd.setCursor(0, 0);
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;
      if (!isConnected(10) && wasConnected) { delay(200); ESP.restart(); }
      if (!getNtpTime(ntpServerName1)) { getNtpTime(ntpServerName2); }
      local = CE.toLocal(now(), &tcr);
      //printTime(local);
      printTime_LCD(local);
    }
  }

  //delay(000);
}
