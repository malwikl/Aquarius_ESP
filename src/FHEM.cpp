#include "FHEM.h"
#include <ESP8266WiFi.h>
#include <WiFiClient.h>


boolean FHEM::setReading(String deviceName, String readingName, double readingValue, byte decimals) {
      char tempString[7];

      //Convert readingValue to String with given decimals
      dtostrf(readingValue, 1, decimals, tempString);
      setReading(deviceName, readingName, tempString);
}

boolean FHEM::setReading(String deviceName, String readingName, String readingValue) {

  // We now create a URI for the request
  String url = F("/fhem?cmd=setReading%20");
  url += deviceName;
  url += F("%20");
  url += readingName;
  url += F("%20");
  url += readingValue;

  boolean success = false;
  /*
  String authHeader = "";
  if ((SecuritySettings.ControllerUser[0] != 0) && (SecuritySettings.ControllerPassword[0] != 0)) {
  base64 encoder;
  String auth = SecuritySettings.ControllerUser;
  auth += ":";
  auth += SecuritySettings.ControllerPassword;
  authHeader = "Authorization: Basic " + encoder.encode(auth) + " \r\n";
}
*/


char host[20] = "192.168.178.34";

// Use WiFiClient class to create TCP connections
WiFiClient client;
if (!client.connect(host, 8083)) {
  Serial.println("HTTP : connection failed");
  return false;
}

// This will send the request to the server
client.print(String("GET ") + url + " HTTP/1.1\r\n" +
"Content-Length: 0\r\n" +
"Host: " + host + "\r\n" + //authHeader +
"Connection: close\r\n\r\n");

unsigned long timer = millis() + 200;
while (!client.available() && millis() < timer)
delay(1);

// Read all the lines of the reply from server and print them to Serial
while (client.available()) {
  String line = client.readStringUntil('\n');
  if (line.substring(0, 15) == "HTTP/1.1 200 OK") {
    Serial.println("HTTP : Success");
    success = true;
  }
  else if (line.substring(0, 24) == "HTTP/1.1 400 Bad Request") {
    Serial.println("HTTP : Unauthorized");
  }
  else if (line.substring(0, 25) == "HTTP/1.1 401 Unauthorized") {
    Serial.println("HTTP : Unauthorized");
  }
  delay(1);
}
Serial.println("HTTP : closing connection");
client.flush();
client.stop();
}
