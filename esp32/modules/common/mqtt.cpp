#include "base.h"
#include "mqtt.h"
#include "wifi.h"

#include <WiFi.h>
#include <WiFiMulti.h>
#include <PubSubClient.h>
#include <ESPmDNS.h>


MQTT::MQTT() {
  WIFI wifi;
  strcpy(nodename, wifi.nodename);

  WiFiClient wifiClient;
  PubSubClient client(wifiClient);
}

Ipport MQTT::mqtt_findservice() {
  Ipport ipport;
  int trials=0;
  int n = MDNS.queryService("mqtt", "tcp");

  while (n != 1) {
    // fallback to ip if service not found
    if (++trials > 3) {
      DL("failed finding mqtt service, fallback to IP!");
      IPAddress fallback;
      ipport.ip = fallback.fromString(MQTT_FALLBACK_SERVER);
      ipport.port = 1883;
      return ipport;
    }
    delay(2000);
    n = MDNS.queryService("mqtt", "tcp");
  }

  D("found at ");
  D(MDNS.IP(0));
  DL();
  ipport.ip = MDNS.IP(0);
  ipport.port = MDNS.port(0);
  return ipport;
}

void MQTT::callback(char* topic, byte* payload, unsigned int length) {
  char msg_cb[100] = "";
  for (int i=0; i<length; i++) {
    msg_cb[i] = payload[i];
  }
  msg_cb[length] = '\0';
  //DF("mqtt msg [%s]: %s\n", topic, msg_cb);

  char id[20] = "", cmd[20] = "";
  char *pos = strchr(msg_cb, ':');
  strcpy(cmd, pos+1);
  strncpy(id, msg_cb, pos-msg_cb);
  DF("got mqtt response. id: %s, cmd: %s\n", id, cmd);

  // do stuff
}

/*
void MQTT::keepalive() {
  if ((millis() - connection_watchdog) > 12000) && (WiFi.status() != WL_CONNECTED)) {
    connection_watchdog = millis();
    DF("Establishing wifi connection\n");
    init_wifi();
  } else if ((millis() - connection_watchdog) > 12000) && (WiFi.status() == WL_CONNECTED && (!client.connected())) {
    DF("Establishing mqtt connection\n");
    mqtt_reconnect();
  }
  client.loop();
  delay(1);
}
*/

/*
 * needs client.loop() in loop() function
 */
void MQTT::mqtt_reconnect() {
  //connect to wifi if not connected
  if (WiFi.status() != WL_CONNECTED)  {
    wifi.connect();
  }

  // connect to mqtt broker if not connected
  if (!client.connected()) {
    DF("Establishing MQTT with id %s, ", nodename);

    Ipport ipport = mqtt_findservice();

    client.setServer(ipport.ip, ipport.port); // 8883 for SSL
    //client.setCallback(callback);

    // if no static callback -> these options do not work (yet)
    /*
    https://stackoverflow.com/questions/46489603/passing-a-member-function-as-standard-function-callback
    std::function<void(char*, unsigned char*, unsigned int)> yourFunction = [=](char* topic, unsigned char* payload, unsigned int length) {
      this->mqtt_callback(topic, payload, length);
    };
    client.setCallback(yourFunction);

    client.setCallback(std::bind(&MQTT::mqtt_callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    client.setCallback([this](char *t, byte *p, unsigned int l) { mqtt_callback(t, p, l); });

    https://github.com/HobbytronicsPK/ESPMetRED
    https://hobbytronics.pk/arduino-custom-library-and-pubsubclient-call-back/
    client.setCallback([this] (char* topic, byte* payload, unsigned int length) { this->callback(topic, payload, length); });
    */

    const byte maxPoll = 2;
    byte connected = 0;
    for (byte i=0; i<maxPoll; i++) {
      // boolean connect (clientID, willTopic, willQoS, willRetain, willMessage)
      // nodename, sensorPOC, 1, true, "went offline"
      client.connect(nodename);
      DF("connecting to mqtt as nodename %s\n", nodename);
      D(".");
      if (client.connected()) {
        connected = 1;
        DL("connected");
        if (!client.subscribe(MQTT_TOPIC_IN)) {
          DL("mqtt subscription callback failed");
        }
        break;
      }
    }

    if (!connected) {
      D("failed, rc=");
      DL(client.state());
    }
  }
}

void MQTT::send_to_mqtt(char* msg) {
  mqtt_reconnect();
  DF("publishing to mqtt \"%s\": \"%s\"\n", MQTT_TOPIC, msg);

  char msg_mqtt[100] = "";
  sprintf(msg_mqtt, "%s\t%s", nodename, msg);

  client.publish(MQTT_TOPIC, msg_mqtt);
}
