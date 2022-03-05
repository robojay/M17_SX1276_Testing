// MetroM4_SX1276_Tx_Test
// Transmit by changing the deviation register from values
// read from the embedded SPI flash.
//
// Note: read https://learn.adafruit.com/adafruit-feather-m0-express-designed-for-circuit-python-circuitpython/using-spi-flash
// to learn how to save the M17 baseband file into the embedded SPI flash
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
#include <SdFat.h>
#include <Adafruit_SPIFlash.h>
#include "SAMDTimerInterrupt.h"

static void timer_callback();
void sxWrite(uint8_t reg, uint8_t *buf, uint8_t count);
void sxRead(uint8_t reg, uint8_t *buf, uint8_t count);

uint8_t sxBuf[16];
SPISettings sxSpi(8000000, MSBFIRST, SPI_MODE0);

// Metro M4
// SPI Pins are on SPI header
// SCK 
// MOSI
// MISO

const int sxCs = 10;
const int sxReset = 9;
const int sxDio2 = 2;
const int sxDio1 = 3;
const int sxDio0 = 4;
const int debugPin = 8;
const int analogInPin = 0;

SAMDTimer metroTimer(TIMER_TC3); 

Adafruit_FlashTransport_QSPI flashTransport;
Adafruit_SPIFlash flash(&flashTransport);
FatFileSystem fatfs;
File m17File;

const int totalBuffers = 2;
const int bufferSize = 512;
volatile uint8_t flashBuf[totalBuffers][bufferSize];
volatile bool bufReady[totalBuffers];
volatile int bufNumber = 0;
volatile int bufIndex = 0;

static void timer_callback() {
  // set pin high at start of callback (for logic analyzer use)
  digitalWrite(debugPin, HIGH);

  // if the data byte is ready, send it to the low byte of the 
  // frequency deviation register RegFdevLsb
  if (bufReady[bufNumber]) {
    sxBuf[0] = flashBuf[bufNumber][bufIndex++];
    sxWrite(0x05, sxBuf, 1);
    // at the end of the buffer?
    if (bufIndex >= bufferSize) {
      bufIndex = 0;
      bufReady[bufNumber++] = false;
      // wrap?
      if (bufNumber >= totalBuffers) {
        bufNumber = 0;
      }
    }
  }
  else {
    // double toggle to indicate missed byte
    digitalWrite(debugPin, LOW);
    digitalWrite(debugPin, HIGH);
  }
  // set pin low at end of callback
  digitalWrite(debugPin, LOW);
}

void loop() {
  static int d;
  static int getBuf = 0;
  static int getIndex = 0;

  // is the current buffer unused?
  if (!bufReady[getBuf]) {
    // yes.. are we at the end of the file?
    if (!m17File.available()) {
      // yes.. go back to the beginning
      m17File.rewind();
    }
    // read the data
    d = m17File.read();
    // any error?
    if (d != -1) {
      // all good, convert to a byte
      flashBuf[getBuf][getIndex++] = (uint8_t)d;
      if (getIndex >= bufferSize) {
        // buffer is full
        bufReady[getBuf++] = true;
        getIndex = 0;
        // wrap around?
        if (getBuf >= totalBuffers) {
          getBuf = 0;
        }
      }
    }
  }
}

// SPI write routine for the SX1276
void sxWrite(uint8_t reg, uint8_t *buf, uint8_t count) {
    reg = reg | 0x80;
    digitalWrite(sxCs, LOW);
    SPI.transfer(reg);
    SPI.transfer(buf, count);
    digitalWrite(sxCs, HIGH);
}

// SPI read routine for the SX1276
void sxRead(uint8_t reg, uint8_t *buf, uint8_t count) {
    memset(buf, 0, count);
    digitalWrite(sxCs, LOW);
    SPI.transfer(reg);
    SPI.transfer(buf, count);
    digitalWrite(sxCs, HIGH);
}

void setup() {
  Serial.begin(115200);
  // Reset pin
  pinMode(sxReset, INPUT);

  // SX1276 DIO pins
  pinMode(sxDio2, INPUT);
  pinMode(sxDio1, INPUT);
  pinMode(sxDio0, INPUT);

  // SX1276 SPI chip select
  pinMode(sxCs, OUTPUT);
  digitalWrite(sxCs, HIGH);

  // pin useful for logic analyzer triggers
  pinMode(debugPin, OUTPUT);
  digitalWrite(debugPin, LOW);

  // reset according to SX1276 datasheet section 7.2.2
  digitalWrite(sxReset, LOW);
  pinMode(sxReset, OUTPUT);
  delay(1);
  pinMode(sxReset, INPUT);
  delay(500);

  // initialize the SPI bus
  SPI.begin();
  SPI.beginTransaction(sxSpi);
  // we're going to take and hold the SPI bus, never doing an endTransaction
  // the microSD card on the Teensy 4.1 is on its own interface

  // verify the chip version
  sxRead(0x42, sxBuf, 1);
  if (sxBuf[0] != 0x12) {
    Serial.println("[ERROR] SX1276 not found!");
    while(true);
  }
  else {
    Serial.println("SX1276 Found.");
  }

  // most of the setting used are default reset values
  // very minimal configuration of the SX1276 is needed

  // power at max for the RFM96 module (uses PA_BOOST)
  sxBuf[0] = 0xf0;
  sxWrite(0x09, sxBuf, 1);

  // continuous mode
  sxBuf[0] = 0x00;
  sxWrite(0x31, sxBuf, 1);

  // enable transmit
  sxBuf[0] = 0x0b;
  sxWrite(0x01, sxBuf, 1);

  // in FSK continous mode, DIO2 controls the modulation
  // we just want it to be stable
  digitalWrite(sxDio2, LOW);
  pinMode(sxDio2, OUTPUT);

  // initialize flash library
  if (!flash.begin()) {
    Serial.println("[error] Flash failed to initialize");
    while(1);
  }

  // mount the filesystem
  if (!fatfs.begin(&flash)) {
    Serial.println("[error] Filesystem failed to mount");
    while(1);
  }
 
  // open the file
  m17File = fatfs.open("M17_TX.RAW", FILE_READ);
  if (!m17File) {
    Serial.println("[error] File open failed");
    while (1);
  }

  for (int i = 0; i < totalBuffers; i++) {
    bufReady[i] = false;
  }

  // start the timer callback at 48kHz
  // units are in microseconds, and can be floats
  metroTimer.attachInterrupt(48000.0, timer_callback);
}
