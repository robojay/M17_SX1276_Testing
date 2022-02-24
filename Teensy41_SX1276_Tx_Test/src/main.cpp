// Teensy41_SX1276_Tx_Test
// Transmit by changing the deviation register from values
// read from the micro SD card.
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

IntervalTimer teensyTimer;

//== items from SdFat examples
//  https://github.com/greiman/SdFat
//  https://github.com/greiman/SdFat/blob/master/LICENSE.md
    #include "SdFat.h"

    // SD_FAT_TYPE = 0 for SdFat/File as defined in SdFatConfig.h,
    // 1 for FAT16/FAT32, 2 for exFAT, 3 for FAT16/FAT32 and exFAT.
    #define SD_FAT_TYPE 1

    // SDCARD_SS_PIN is defined for the built-in SD on some boards.
    #ifndef SDCARD_SS_PIN
    const uint8_t SD_CS_PIN = SS;
    #else  // SDCARD_SS_PIN
    // Assume built-in SD is used.
    const uint8_t SD_CS_PIN = SDCARD_SS_PIN;
    #endif  // SDCARD_SS_PIN

    // Try to select the best SD card configuration.
    #if HAS_SDIO_CLASS
    #define SD_CONFIG SdioConfig(FIFO_SDIO)
    #elif ENABLE_DEDICATED_SPI
    #define SD_CONFIG SdSpiConfig(SD_CS_PIN, DEDICATED_SPI)
    #else  // HAS_SDIO_CLASS
    #define SD_CONFIG SdSpiConfig(SD_CS_PIN, SHARED_SPI)
    #endif  // HAS_SDIO_CLASS

    #if SD_FAT_TYPE == 0
    SdFat sd;
    File file;
    #elif SD_FAT_TYPE == 1
    SdFat32 sd;
    File32 file;
    #elif SD_FAT_TYPE == 2
    SdExFat sd;
    ExFile file;
    #elif SD_FAT_TYPE == 3
    SdFs sd;
    FsFile file;
    #else  // SD_FAT_TYPE
    #error Invalid SD_FAT_TYPE
    #endif  // SD_FAT_TYPE
//== end of items from SdFat example

volatile uint8_t dataByte = 0;
volatile bool dataReady = false;

static void timer_callback() {
  // set pin high at start of callback (for logic analyzer use)
  digitalWrite(debugPin, HIGH);

  // if the data byte is ready, send it to the low byte of the 
  // frequency deviation register RegFdevLsb
  if (dataReady) {
    sxBuf[0] = dataByte;
    sxWrite(0x05, sxBuf, 1);
    // indicate we need another byte
    dataReady = false;
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

  // does the timer callback need another byte?
  if (!dataReady) {
    // yes.. are we at the end of the file?
    if (!file.available()) {
      // yes.. go back to the beginning
      file.rewind();
    }
    // read the data
    d = file.read();
    // any error?
    if (d != -1) {
      // all good, convert to a byte
      dataByte = (uint8_t)d;
      // tell the timer callback it's ready
      dataReady = true;
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

  // initialize the SD card
  if (!sd.begin(SD_CONFIG)) {
    sd.initErrorHalt(&Serial);
    return;
  }

  // open the file
  if (!file.open("M17_TX.RAW", FILE_READ)) {
    Serial.println("open failed");
    return;
  }

  // start the timer callback at 48kHz
  // units are in microseconds, and can be floats
  teensyTimer.begin(timer_callback, 1000.0/48.0);

}
