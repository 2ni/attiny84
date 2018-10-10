#ifndef MQTT_H
#define MQTT_H

#include <Arduino.h>
#include <PubSubClient.h>
#include "wifi.h"

#define MQTT_FALLBACK_SERVER "192.168.1.8" // fallback, we use mqtt/tcp discovery service (via /etc/avahi/service)
#define MQTT_TOPIC "sensorPOC" // outbound topic
#define MQTT_TOPIC_IN "sensorPOCIN" // inbound topic

struct Ipport {
  IPAddress ip;
  int port;
};

class MQTT {
  private:
    Ipport mqtt_findservice();
    void mqtt_reconnect();

  public:
    char nodename[20];
    PubSubClient client;
    WIFI wifi;

    MQTT(char* nodename);
    MQTT();
    void send_to_mqtt(char* msg);
    static void callback(char* topic, byte* payload, unsigned int length);
};

#endif
