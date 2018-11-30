#include <WiFi.h>
#include <WiFiMulti.h>
#include <ESPmDNS.h>

#include <esp_wifi.h>
#include <esp_bt.h>

#include "creds.h"
#include "base.h"

#include "wifi.h"

WIFI::WIFI() {
  WiFiMulti wifiMulti;

  uint64_t chipid = ESP.getEfuseMac();
  unsigned int l = sprintf(nodename, "ESP_%llx", chipid);
  nodename[l] = '\0';
}


/*
 * Connect to configured wifi's
 *
 * WIFI_CREDS must be defined in creds.h and always terminate with NULL:
 * static const char*  WIFI_CREDS[] = {"ssid1", "pw1", "ssid2", ..., NULL}
 */
int WIFI::connect() {
  for (int i=0; WIFI_CREDS[i] != NULL; i+=2) {
    //DF("i: %i, ssid: %s, pw: %s\n", i, WIFI_CREDS[i], WIFI_CREDS[i+1]);
    wifiMulti.addAP(WIFI_CREDS[i], WIFI_CREDS[i+1]);
  }

  if (WiFi.status() == WL_CONNECTED) {
    D("Wifi already connected to: ");
    DL(WiFi.localIP());
    return 1;
  }

  DL();
  DF("Hello from %s\n", nodename);
  D("Connecting");

  WiFi.mode(WIFI_STA); // default is WIFI_AP_STA
  WiFi.setHostname(nodename); // must be called as very 1st but with wifi on
  unsigned long startMillis = millis();

  while (wifiMulti.run() != WL_CONNECTED) {
    D(".");
    if (millis() - startMillis > WIFI_TIMEOUT) {
      DL("Timeout connecting to WiFi...");
      return 0;
    }
    delay(250);
  }

  D(" connected to ");
  Serial.print(WiFi.SSID());

  D(". IP address: ");
  DL(WiFi.localIP());

  // setup MDNS
  if (MDNS.begin(nodename)) {
    DF("server started on http://%s.local\n\r", nodename);
  }

  return 1;
}

void WIFI::deep_sleep(uint64_t s) {
  DF("deep sleeping for %llus\n", s);
  esp_wifi_stop();
  esp_bt_controller_disable();

  esp_sleep_enable_timer_wakeup(1000000 * s);
  esp_deep_sleep_start();
  while(true){}; // do nothing until we sleep
}
