/***************************************************************************
Title: PCA9539 sketch (DT-I/O I2C to Parallel Converter) - Output Example
Created by: Innovative Electronics - Training Division
For: Arduino Training - Interfacing with Arduino

This file is free software; you can redistribute it and/or modify
it under the terms of either the GNU General Public License version 2
or the GNU Lesser General Public License version 2.1, both as
published by the Free Software Foundation.
***************************************************************************/

#include <DTIOI2CtoParallelConverter.h>
#include <Wire.h>

DTIOI2CtoParallelConverter ioConverter(0x77);

void setup()
{
  Serial.begin(9600);
  Wire.begin();
  ioConverter.portMode0(ALLOUTPUT);
}

void _digitalWriteOperation()
{
  ioConverter.digitalWrite0(0, LOW);
  ioConverter.digitalWrite0(1, LOW);
  ioConverter.digitalWrite0(2, HIGH);
  ioConverter.digitalWrite0(3, HIGH);
  ioConverter.digitalWrite0(4, HIGH);
  ioConverter.digitalWrite0(5, HIGH);
  ioConverter.digitalWrite0(6, HIGH);
  ioConverter.digitalWrite0(7, HIGH);
  delay(500);
  ioConverter.digitalWrite0(0, HIGH);
  ioConverter.digitalWrite0(1, HIGH);
  ioConverter.digitalWrite0(2, LOW);
  ioConverter.digitalWrite0(3, LOW);
  ioConverter.digitalWrite0(4, HIGH);
  ioConverter.digitalWrite0(5, HIGH);
  ioConverter.digitalWrite0(6, HIGH);
  ioConverter.digitalWrite0(7, HIGH);
  delay(500);
  ioConverter.digitalWrite0(0, HIGH);
  ioConverter.digitalWrite0(1, HIGH);
  ioConverter.digitalWrite0(2, HIGH);
  ioConverter.digitalWrite0(3, HIGH);
  ioConverter.digitalWrite0(4, LOW);
  ioConverter.digitalWrite0(5, LOW);
  ioConverter.digitalWrite0(6, HIGH);
  ioConverter.digitalWrite0(7, HIGH);
  delay(500);
  ioConverter.digitalWrite0(0, HIGH);
  ioConverter.digitalWrite0(1, HIGH);
  ioConverter.digitalWrite0(2, HIGH);
  ioConverter.digitalWrite0(3, HIGH);
  ioConverter.digitalWrite0(4, HIGH);
  ioConverter.digitalWrite0(5, HIGH);
  ioConverter.digitalWrite0(6, LOW);
  ioConverter.digitalWrite0(7, LOW);
  delay(500);
}

void _digitalWritePortOperation()
{
  for(byte i = 0; i < 8; i++)
  {
    ioConverter.digitalWritePort0(~(1 << i));
    delay(500);
  }
}

void loop()
{
  for(int i = 0; i < 3; i++)
    _digitalWriteOperation();
  for(int i = 0; i < 3; i++)
    _digitalWritePortOperation();
}
