/*
 * RFM69HCW control
 * see https://github.com/hallard/NodeMCU-Gateway
 * difference to RFM96 lorawan?
 *
 * Code
 * -> https://github.com/LowPowerLab/RFM69
 * -> https://github.com/bbx10/nanohab
 * -> https://github.com/flabbergast/RFM69/tree/attiny (for digispark pro)
 *
 * DEPENDENCIES (CAUTION!!!!!!!!!)
 * RFM69 library https://github.com/LowPowerLab/RFM69 with following changes:
 * @@ -290,7 +290,7 @@ void RFM69::sendACK(const void* buffer, uint8_t bufferSize) {
 * int16_t _RSSI = RSSI; // save payload received RSSI value
 * writeReg(REG_PACKETCONFIG2, (readReg(REG_PACKETCONFIG2) & 0xFB) | RF_PACKET2_RXRESTART); // avoid RX deadlocks
 * uint32_t now = millis();
 * -  while (!canSend() && millis() - now < RF69_CSMA_LIMIT_MS) receiveDone();
 * +  while (!canSend() && millis() - now < RF69_CSMA_LIMIT_MS) {receiveDone(); delay(1);}
 *
 */


#include "base.h"

// *********** radio stuff
#include <RFM69.h>

#define RECEIVER      1
#define SENDER        99
#define NODEID        RECEIVER    //unique for each node on same network
#define NETWORKID     100  //the same on all nodes that talk to each other
//Match frequency to the hardware version of the radio on your Moteino (uncomment one):
//#define FREQUENCY     RF69_433MHZ
#define FREQUENCY     RF69_868MHZ
//#define FREQUENCY     RF69_915MHZ
#define ENCRYPTKEY    "sampleEncryptKey" //exactly the same 16 characters/bytes on all nodes!


#define RFM69_CS      16 // NSS
#define RFM69_IRQ     15 // DIO0
#define IS_RFM69HCW   true // set to 'true' if you are using an RFM69HCW module
RFM69 radio = RFM69(RFM69_CS, RFM69_IRQ, IS_RFM69HCW);

typedef struct {
  int          nodeId;
  int16_t      temp;
  uint16_t     hum;
} Payload;
Payload data;

bool promiscuousMode = false;

byte ackCount=0;
uint32_t packetCount = 0;

/**************************************** main *************************************************/

void setup() {
  Serial.begin(115200);
  while (!Serial);

  // to avoid watchdog when using sendWithRetry()
  //WiFi.mode(WIFI_OFF);

  DL("");
  DL("Booting.");

  // hard reset RFM module
  /*
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, HIGH);
  delay(100);
  digitalWrite(RFM69_RST, LOW);
  delay(100);
  */

  if (radio.initialize(FREQUENCY,NODEID,NETWORKID)) {
    DF("radio initialize successful.\n");
  } else {
    DF("radio initialize failed!!!\n");
  }

  if (IS_RFM69HCW) {
    radio.setHighPower();
  }
  //radio.setPowerLevel(31);
  radio.encrypt(ENCRYPTKEY);
  radio.promiscuous(promiscuousMode);

  DF("RFM69HCW receiver @ %uMHz - Version 0x%o\n", radio.getFrequency()/1000000, radio.readReg(0x10));
}

void loop() {
  if (radio.receiveDone()) {
    if (radio.DATALEN == sizeof(Payload)) {
      data = *(Payload*)radio.DATA;

      /*
       * the following change is needed in RFM69.cpp  on RFM69::sendACK, line294:
       * while (!canSend() && millis() - now < RF69_CSMA_LIMIT_MS) {receiveDone(); delay(1);}
       */
      if (radio.ACKRequested()) {
        // send back ack with some dummy data
        const char* yes = "Hello from master!";
        unsigned long s = millis();
        //DF("rssi: %d\n", radio.readRSSI());
        radio.sendACK(yes, strlen(yes));
        DF("ACK sent to [%u] in %ums. ", radio.SENDERID, millis()-s);
      }

      DF("Got data {nodeId: %u, temp: %d, hum: %u}\n", data.nodeId, data.temp, data.hum);
    } else {
      Serial.print("Invalid payload received, not matching Payload struct!");
    }

      // When a node requests an ACK, respond to the ACK
      // and also send a packet requesting an ACK (every 3rd one only)
      // This way both TX/RX NODE functions are tested on 1 end at the GATEWAY
      /*
       * does not work -> wdt resets!
      if (ackCount++%3==0) {
        Serial.print(" Pinging node ");
        Serial.print(theNodeID);
        Serial.print(" - ACK...");
        delay(10); //need this when sending right after reception .. ?
        if (radio.sendWithRetry(theNodeID, "ACK TEST", 0, 100))  // 0 = only 1 attempt, no retries
          Serial.print("ok!");
        else Serial.print("nothing");
      }
      */
  }
}
