// Teensy41_SX1276_Rx_Test
// Receive by polling the frequency error/offset register
//
// This is a horrible, brute force, just see if it works program
//
// https://github.com/robojay/M17_SX1276_Testing
//
// Copyright (C) 2022  Jay Francis, KA1PQK
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along
// with this program; if not, write to the Free Software Foundation, Inc.,
// 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//
#include <Arduino.h>
#include <SPI.h>
#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>

static void timer_callback();
void sxWrite(uint8_t reg, uint8_t *buf, uint8_t count);
void sxRead(uint8_t reg, uint8_t *buf, uint8_t count);

uint8_t sxBuf[16];
SPISettings sxSpi(10000000, MSBFIRST, SPI_MODE0);

// Teensy 4.1
// NSS pin:   10
// RESET pin: 33
// DIO2 pin: 34
// DIO1 pin:  35
// DIO0 pin:  36

const int sxCs = 10;
const int sxReset = 33;
const int sxDio2 = 34;
const int sxDio1 = 35;
const int sxDio0 = 36;
const int debugPin = 37;
const int analogInPin = 14;
const int zeroBeatPin = 38;

IntervalTimer teensyTimer;

IPAddress broadcastIP;
const unsigned int broadcastPort = 8888;
EthernetUDP Udp;

volatile int16_t dataInt = 0;
volatile bool dataReady = false;
volatile int16_t frequencyOffset = 0;
const int16_t zeroBeatRange = 10; // 61Hz * 10 = +/- 610Hz

const int totalBuffers = 2;
const int bufferSize = 512;
volatile uint8_t broadcastBuf[totalBuffers][bufferSize];
volatile int bufNumber = 0;
volatile int bufIndex = 0;
volatile int sendBuf = -1;
volatile double deviation = 0.0;
volatile int deviationInt = 0.0;

const double in_hz_per_bit = 61.0;     // SX1276
const double out_bit_per_hz = 8.96;    // m17-demod expectation
const double deviationConversion = in_hz_per_bit * out_bit_per_hz;

// send either raw values from the frequency offset register (true)
// or scaled valued for m17-demod (false)
// useful for linux command line like:
// nc -u -l 8888 -k | m17-demod -l -v | play -b 16 -r 8000 -c1 -t s16 -
const bool rawDeviation = false;

static void timer_callback() {
  digitalWrite(debugPin, HIGH);

  sxRead(0x1d, sxBuf, 2);
  frequencyOffset = (int16_t)(sxBuf[0])<<8 | sxBuf[1];
  deviation = float(frequencyOffset) * deviationConversion;
  deviation = constrain(deviation, -32768.0, 32767.0);
  deviationInt = (int)round(deviation);

  if (abs(frequencyOffset) <= zeroBeatRange) {
    digitalWrite(zeroBeatPin, HIGH);
  }
  else {
    digitalWrite(zeroBeatPin, LOW);
  }

  if (rawDeviation) {
    broadcastBuf[bufNumber][bufIndex++] = sxBuf[1];
    broadcastBuf[bufNumber][bufIndex++] = sxBuf[0];
  }
  else {
    broadcastBuf[bufNumber][bufIndex++] = (uint8_t)(deviationInt & 0xff);
    broadcastBuf[bufNumber][bufIndex++] = (uint8_t)((deviationInt >> 8) & 0xff);
  }

  if (bufIndex >= bufferSize) {
    // swap buffer
    // send buffer
    bufIndex = 0;
    if (sendBuf != -1) {
      // overrun
      // double toggle to indicate overrun
      digitalWrite(debugPin, LOW);
      digitalWrite(debugPin, HIGH);
      // don't want to trash the whole buffer
      // will spin on last location
      bufIndex = bufferSize - 2;
    }
    else {
      sendBuf = bufNumber;
      bufNumber++;
      if (bufNumber >= totalBuffers) {
        bufNumber = 0;
      }
    }
  }

  digitalWrite(debugPin, LOW);
}

void loop() {
  static bool stopped = false;
  static uint8_t b;

  if (!stopped) {
    if (sendBuf != -1) {
      Udp.beginPacket(broadcastIP, broadcastPort);
      Udp.write((uint8_t *)(&broadcastBuf[sendBuf][0]), bufferSize);
      Udp.endPacket();
      sendBuf = -1;
    }        
  }

  if (Serial.available()) {
    b = Serial.read();
    if (stopped) {
      stopped = false;
      Serial.println("Streaming...");
    }
    else {
      stopped = true;
      Serial.println("Stopped");
    }
  }

}

void sxWrite(uint8_t reg, uint8_t *buf, uint8_t count) {
    reg = reg | 0x80;
    digitalWrite(sxCs, LOW);
    SPI.transfer(reg);
    SPI.transfer(buf, count);
    digitalWrite(sxCs, HIGH);
}

void sxRead(uint8_t reg, uint8_t *buf, uint8_t count) {
    memset(buf, 0, count);
    digitalWrite(sxCs, LOW);
    SPI.transfer(reg);
    SPI.transfer(buf, count);
    digitalWrite(sxCs, HIGH);
}

void setup() {
  Serial.begin(115200);
  pinMode(sxReset, INPUT);
  pinMode(sxDio2, INPUT);
  pinMode(sxDio1, INPUT);
  pinMode(sxDio0, INPUT);
  pinMode(sxCs, OUTPUT);

  pinMode(debugPin, OUTPUT);
  digitalWrite(debugPin, LOW);

  pinMode(zeroBeatPin, OUTPUT);
  digitalWrite(zeroBeatPin, LOW);

  digitalWrite(sxCs, HIGH);

  // reset according to 7.2.2
  digitalWrite(sxReset, LOW);
  pinMode(sxReset, OUTPUT);
  delay(1);
  pinMode(sxReset, INPUT);
  delay(500);

  SPI.begin();
  SPI.beginTransaction(sxSpi);
  // we're going to take and hold the SPI bus

  sxRead(0x42, sxBuf, 1);
  if (sxBuf[0] != 0x12) {
    Serial.println("[ERROR] SX1276 not found!");
    while(true);
  }
  else {
    Serial.println("SX1276 Found.");
  }

  // Power
  sxBuf[0] = 0xf0;
  sxWrite(0x09, sxBuf, 1);

  // continuous mode
  sxBuf[0] = 0x00;
  sxWrite(0x31, sxBuf, 1);

  // bit rate 300kbps (4 bit times per FEI calculation)
  // 0x00 0x6B 
  sxBuf[0] = 0x00;
  sxBuf[1] = 0x6b;
  sxWrite(0x02, sxBuf, 2);

  // (test) big deviation
  //sxBuf[0] = 0x13;
  //sxBuf[1] = 0x36;
  //sxWrite(0x04, sxBuf, 2);

  // RxBW
  // RxBwMant 00b RxBwExp 1
  sxBuf[0] = 0x01;
  sxWrite(0x12, sxBuf, 1);

  // enable receive
  sxBuf[0] = 0x0d;
  sxWrite(0x01, sxBuf, 1);

  uint8_t mac[6];
  for(uint8_t by=0; by<2; by++) mac[by]=(HW_OCOTP_MAC1 >> ((1-by)*8)) & 0xFF;
  for(uint8_t by=0; by<4; by++) mac[by+2]=(HW_OCOTP_MAC0 >> ((3-by)*8)) & 0xFF;
  Serial.printf("MAC: %02x:%02x:%02x:%02x:%02x:%02x\r\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  if (Ethernet.begin(mac) == 0) {
    Serial.println("Failed to obtain IP address");
    while(true);
  }
  Serial.print("My IP address: ");
  Serial.println(Ethernet.localIP());
  broadcastIP = Ethernet.localIP();
  broadcastIP[3] = 0xff;
  Serial.print("Broadcast IP address: ");
  Serial.println(broadcastIP);
  Udp.begin(broadcastPort);

  teensyTimer.priority(0);
  teensyTimer.begin(timer_callback, 1000.0/48.0);  // should be 1000.0/48.0 for 48kHz

  Serial.println("Hit a key to start");
  while (!Serial.available());
  while(Serial.available()) {
    uint8_t b = Serial.read();
  }
  Serial.println("Streaming...");

}
