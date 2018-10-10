#ifndef WIFI_H
#define WIFI_H

#include <WiFiMulti.h>

#define WIFI_TIMEOUT 5e3

class WIFI {
  public:
    WIFI();
    WiFiMulti wifiMulti;
    char nodename[20];
    int connect();
    void deep_sleep(uint64_t ms);
};

#endif
