#include <Arduino.h>

#include <WiFi.h>
#include <EEPROM.h>
#include <Update.h>
#include <WebServer.h>
#include <DNSServer.h>

#include "Draw.h"
#include "Display.h"

#include "Common.h"
#include "creds_mqtt.h"

Draw draw;
Display display;

void setup()
{
  Serial.begin(115200);
  display.Sdbegin();
  display.DisplayInit(0,80);
  display.LogStatusMessage("SD setup"); 
  display.DisplyTest(1000);

  display.LogStatusMessage("Connecting to WiFi...");
  // Serial.print("Connecting to ");
  // Serial.println(WIFI_SSID);
  // WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  // while (WiFi.status() != WL_CONNECTED) {
  //   delay(500);
  //   Serial.print(".");
  // }
  // Serial.println("");
  // Serial.println("WiFi connected.");
  // display.LogStatusMessage("WiFi connected!");
}

void loop(){

  display.AutoGif(1000);


}
