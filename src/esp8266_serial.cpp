/**
 * @example ConnectWiFi.ino
 * @brief The ConnectWiFi demo of library WeeESP8266.
 * @author Wu Pengfei<pengfei.wu@itead.cc>
 * @date 2015.03
 *
 * @par Copyright:
 * Copyright (c) 2015 ITEAD Intelligent Systems Co., Ltd. \n\n
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version. \n\n
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#include "ESP8266.h"
#include <SoftwareSerial.h>

#define SSID        "elecom2g-fa7d7b"
#define PASSWORD    "4208401361893"
#define RX 2
#define TX 3
SoftwareSerial Serial1(RX, TX);
//ESP8266 wifi(Serial1, 9600);

void setup(void)
{
  pinMode(RX, INPUT);
  pinMode(TX, OUTPUT);

  Serial.begin(115200);
  Serial1.begin(9600);

  Serial.print("setup begin\r\n");
  Serial.print("Type in AT commands now\r\n");
  Serial.print("Make sure that carriage return are sent\r\n");
  Serial.print("setup end\r\n");

}

void loop(void)
{
  byte b;
  while(Serial.available()) {
    b=Serial.read();
    Serial1.write(b);
  }

  String content = "";
  char character;
  while (Serial1.available()) {
    character = Serial1.read();
    content.concat(character);
  }

  if (content != "") {
    Serial.print(content);
    Serial.print("\r\n");
  }

  delay(1000);
}

