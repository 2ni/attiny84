/**
 * Simple terminal reply prompt
 */

#include "base.h"
#include "mqtt.h"

#include <Arduino.h>
#include <Wire.h>

#define SDA 21
#define SCL 22
#define ADDR 0x21

#define I2C_RESET     0x15
#define I2C_GET_RAW   0x14
#define I2C_GET_MOIST 0x13
#define I2C_SET_BLINK 0x12
#define I2C_GET_BLINK 0x11
#define I2C_GET_TEMP  0x10

#define DATA_ADC_NUM 3
#define SHIPPING_INTERVALL 5*60000 // 5min

unsigned long next_shipping;

MQTT mqtt;

/*
 * I2C functions
 */

char input[20] = "";
byte i = 0;
byte no_port = 0;
byte port = 0;

void readraw(byte addr, byte reg, byte size) {
  Wire.beginTransmission(addr);
  Wire.write((uint8_t)reg);
  Wire.endTransmission();
  Wire.requestFrom(addr, size);
}

byte read8(byte addr, byte reg) {
  byte value;
  readraw(addr, reg, 1);

  value = Wire.read();
  Wire.endTransmission();

  return value;
}

uint16_t read16(byte addr, byte reg) {
  uint16_t value;
  readraw(addr, reg, 2);

  byte in[2]={0};
  byte c=0;
  // h first, then l
  // ignore all except first 2 bytes
  while (Wire.available()) {
    byte b = Wire.read();
    if (c<2) {
      in[c] = b;
    }
    c++;
  }

  Wire.endTransmission();

  value = ((in[0] << 8)&0xff00) | in[1];
  return value;
}

/*
 * Read <size> uint16_t
 * and save them in values
 */
void read16(byte addr, byte reg, uint16_t *values, byte size) {
  byte c = 0;
  readraw(addr, reg, 2*size);

  while (Wire.available()) {
    if (c<size) {
      values[c] = Wire.read() << 8 | Wire.read();
    }
    c++;
  }
  Wire.endTransmission();
}

byte write8(byte addr, byte reg, byte value) {
  byte error;

  Wire.beginTransmission(addr);
  Wire.write((uint8_t)reg);
  Wire.write((uint8_t)value);
  error = Wire.endTransmission();
  return error;
}

byte write8(byte addr, byte value) {
  byte error;

  Wire.beginTransmission(addr);
  Wire.write((uint8_t)value);
  error = Wire.endTransmission();

  return error;
}

byte scan_ports() {
  byte error, address;
  for (address = 1; address < 127; address++ )  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      DF("I2C slave found: 0x%x\n", address);
      return address;
    } else if (error == 4) {
      DF("error at address: 0x%x\n", address);
    }
  } //for loop

  DL("No I2C slaves found");
  return 0;
}

void send_data() {
  if (no_port) return;

  char msg[50] = "";
  memset(msg, 0, sizeof(msg)); // really clear all data
  int16_t t = (int16_t)read16(port, I2C_GET_TEMP);
  uint16_t m = read16(port, I2C_GET_MOIST);

  char ct[10];
  dtostrf((float)t/10, 0, 1, ct);

  sprintf(msg, "measure\thum=%u,temp=%s", m, ct);
  //DF("msg: %s\n", msg);
  mqtt.send_to_mqtt(msg);
  next_shipping = millis()+SHIPPING_INTERVALL;
  write8(port, I2C_SET_BLINK, 1); // blink
}

void show_help() {
  DL("***********************************");
  DL("h - help");
  DL("s - send data to mqtt");
  DL("r - reset sensor");
  DL("d - display raw data from sensor");
  DL("t - show temperature from sensor");
  DL("m - show moisture from sensor");
  DL("c - show last blink count");
  DL("[0-9]+<enter> - blink LED on sensor");
  DL("***********************************");
  DL();
}

void setup() {
  // uart
  Serial.begin(115200);
  Serial.setTimeout(2000);
  while (!Serial) { }

  DL("Hello from outer space!");
  show_help();
  DL("Enter number of desired eye blinking...");

  pinMode(SDA, INPUT_PULLUP);
  pinMode(SCL, INPUT_PULLUP);

  Wire.setClock(200000); // 400kHz might be too much
  Wire.begin(SDA, SCL);

  // check if slave is here
  Wire.beginTransmission(ADDR);
  byte err = Wire.endTransmission();
  if (err == 0) {
    port = ADDR;
    DF("Yup. We're up and running at 0x%x!\n", port);
  } else if (err == 4) {
    DF("Hm... Unknwon error at 0x%x!\n", port);
  } else {
    DL("Outsch. Slave not found!");
    no_port = 1;
  }

  next_shipping = 0;
  send_data();
}

void loop() {
  char in;

  mqtt.loop();

  // deep sleep after 5min inactivity
  // send data for now
  if (millis() > next_shipping) {
    send_data();
  }

  if (no_port && (millis() % 2000 == 0)) {
    port = scan_ports();
    if (port) {
      next_shipping = 0; // send now
      no_port = 0;
    }
  }

  if (Serial.available()) {
    in = (char)Serial.read();
    //next_shipping = millis()+SHIPPING_INTERVALL;
    if (in == 'h') {
      show_help();
    } else if (in == 's') {
      send_data();
    } else if (in == 'r') {
      // reset
      write8(port, I2C_RESET);
      DF("resetting...\n");
    } else if (in == 'd') {
      uint16_t values[DATA_ADC_NUM] = {0};
      read16(port, I2C_GET_RAW, values, sizeof(values)/sizeof(values[0]));
      for (byte i=0; i<DATA_ADC_NUM; i++) {
        DF("raw(%u): %u\n", i, values[i]);
      }
    } else if (in == 't') {
      DF("temp: %d\n", (int16_t)read16(port, I2C_GET_TEMP));
    } else if (in == 'm') {
      DF("moisture: %u\n", read16(port, I2C_GET_MOIST));
    } else if (in == 'c') {
      if (port) {
        DF("last blink count: %i\n", read8(port, I2C_GET_BLINK));

      } else {
        DF("command not sent as no slave available!\n");
        no_port = 1;
      }

    } else if (in == '\r') {
      int num = atoi(input);
      //DF("You entered: %s\n", input);
      DF("\nYou entered: %i\n", num);
      if (!port || write8(port, I2C_SET_BLINK, num)) {
        DF("command not sent as no slave available!\n");
        no_port = 1;
      }

      i = 0;
      memset(input, 0, sizeof(input)); // really clear it
    } else if (in != '\n') {
      DF("%c", in);
      //DF("typed: %c\n", in);
      input[i] = in;
      i++;
      input[i] = '\0';
    }
  }
}

