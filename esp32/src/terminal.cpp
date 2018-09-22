/**
 * Simple terminal reply prompt
 */

#include <Arduino.h>
#include <Wire.h>

#define D(x) Serial.print(x)
#define DL(x) Serial.println(x)
#define DF(...) Serial.printf(__VA_ARGS__);

#define SDA 21
#define SCL 22
#define ADDR 0x21

#define I2C_SET_BLINK 0x12
#define I2C_GET_BLINK 0x11
#define I2C_GET_TEMP  0x10

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
  byte l;
  byte h;
  readraw(addr, reg, 2);

  h = Wire.read();
  l = Wire.read();

  Wire.endTransmission();

  value = ((h << 8)&0xff00) | l;
  return value;
}

byte write8(byte addr, byte reg, byte value) {
  byte error;

  Wire.beginTransmission(addr);
  Wire.write((uint8_t)reg);
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

void setup() {
  // uart
  Serial.begin(115200);
  Serial.setTimeout(2000);
  while (!Serial) { }

  DL("Hello from outer space!");
  DL("Enter number of desired eye blinking...");

  pinMode(SDA, INPUT_PULLUP);
  pinMode(SCL, INPUT_PULLUP);

  Wire.setClock(400000);
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
}

void loop() {
  char in;

  if (no_port && (millis() % 2000 == 0)) {
    port = scan_ports();
    if (port) {
      no_port = 0;
    }
  }

  if (Serial.available()) {
    in = (char)Serial.read();
    if (in == 't') {
      DF("temp: %u\n", read16(port, I2C_GET_TEMP));
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


