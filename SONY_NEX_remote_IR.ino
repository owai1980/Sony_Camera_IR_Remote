/*
   SendDemo.cpp

   Demonstrates sending IR codes in standard format with address and command

    This file is part of Arduino-IRremote https://github.com/Arduino-IRremote/Arduino-IRremote.

 ************************************************************************************
   MIT License

   Copyright (c) 2020-2021 Armin Joachimsmeyer

   Permission is hereby granted, free of charge, to any person obtaining a copy
   of this software and associated documentation files (the "Software"), to deal
   in the Software without restriction, including without limitation the rights
   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
   copies of the Software, and to permit persons to whom the Software is furnished
   to do so, subject to the following conditions:

   The above copyright notice and this permission notice shall be included in all
   copies or substantial portions of the Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
   INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
   PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
   HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
   CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
   OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

 ************************************************************************************
*/
#include <Arduino.h>

/*
   Define macros for input and output pin etc.
*/
#include "PinDefinitionsAndMore.h"

//#define EXCLUDE_EXOTIC_PROTOCOLS // saves around 240 bytes program space if IrSender.write is used
//#define SEND_PWM_BY_TIMER
//#define USE_NO_SEND_PWM

#include <IRremote.h>

long int temps = 4000;

void setup() {
  Serial.begin(115200);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(7, INPUT_PULLUP);
  pinMode(8, OUTPUT);
  digitalWrite(8, LOW);
  delay(500);
  int sensorVal = digitalRead(7);
  Serial.println(sensorVal);
  if (sensorVal == HIGH)
    temps = 4000;
  else
    temps = 36000;    
  digitalWrite(LED_BUILTIN, sensorVal);   // turn the LED on (HIGH is the voltage level)
  digitalWrite(8, LOW); // save energy :-)

#if defined(__AVR_ATmega32U4__) || defined(SERIAL_USB) || defined(SERIAL_PORT_USBVIRTUAL) || defined(ARDUINO_attiny3217)
  delay(3000);  // To be able to connect Serial monitor after reset or power up and before first printout
#endif
  // Just to know which program is running on my Arduino
  Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_IRREMOTE));

  IrSender.begin(IR_SEND_PIN, ENABLE_LED_FEEDBACK);  // Specify send pin and enable feedback LED at default feedback LED pin

  Serial.print(F("Ready to send IR signals at pin "));
  Serial.println(IR_SEND_PIN);

#if defined(USE_SOFT_SEND_PWM) && !defined(ESP32)  // for esp32 we use PWM generation by hw_timer_t for each pin
  /*
     Print internal signal generation info
  */
  IrSender.enableIROut(38);

  Serial.print(F("Send signal mark duration is "));
  Serial.print(IrSender.periodOnTimeMicros);
  Serial.print(F(" us, pulse correction is "));
  Serial.print((uint16_t)PULSE_CORRECTION_NANOS);
  Serial.print(F(" ns, total period is "));
  Serial.print(IrSender.periodTimeMicros);
  Serial.println(F(" us"));
#endif
}

/*
   Set up the data to be sent.
   For most protocols, the data is build up with a constant 8 (or 16 byte) address
   and a variable 8 bit command.
   There are exceptions like Sony and Denon, which have 5 bit address.
*/
uint16_t sAddress = 0x1E3A;
// 0x2D = shoot immediat
// 0x37 = shoot 2sec focus
// 0x38 = Menu
// 0x39 = ENTER
// 0x3A = UP (arrow)
// 0x3B = DOWN (arrow)
// 0x3C = PLAY
// 0x3E = LEFT (arrow)
// 0x3F = RIGHT (arrow)
// 0x47 = play video ?
// 0x48 = video record start/stop
// +k*0x80
uint8_t sCommand = 0x37;
uint8_t sRepeats = 1;  // 2 pour camera sony?

void loop() {
  /*
     Print values
  */
#if 0
  Serial.println();
  Serial.print(F("address=0x"));
  Serial.print(sAddress, HEX);
  Serial.print(F(" command=0x"));
  Serial.print(sCommand, HEX);
  Serial.print(F(" repeats="));
  Serial.println(sRepeats);
  Serial.println();
  Serial.flush();

  Serial.println(F("Send Sony/SIRCS with 7 command and 13 address bits"));
  Serial.flush();
#endif

  IrSender.sendSony(sAddress & 0x1FFF, sCommand & 0x7F, sRepeats, SIRCS_20_PROTOCOL);


  delay(temps);  // additional delay at the end of each loop

  // sCommand++;
}