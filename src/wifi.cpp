
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
#include <Arduino.h>
#include <SoftwareSerial.h>

#define SSID  "elecom2g-fa7d7b"
#define PASSWORD  "4208401361893"
#define HOST_NAME "192.168.250.225"
#define HOST_PORT   (5005)

SoftwareSerial Serial1(2, 3); // TX, RX
ESP8266 wifi(Serial1);

void setup(void)
{
  Serial.begin(115200);
  Serial.print("setup begin\r\n");

  Serial.print("Restarting WIFI: ");
  if(wifi.restart()) {
    Serial.print("OK\r\n");
  } else {
    Serial.print("FAILED");
  }
  Serial.print("FW Version: ");
  Serial.println(wifi.getVersion().c_str());

  Serial.print("To Station + SoftAP: ");
  if (wifi.setOprToStationSoftAP()) {
    Serial.print("OK\r\n");
  } else {
    Serial.print("FAILED\r\n");
  }

  if (wifi.joinAP(SSID, PASSWORD)) {
    Serial.print("Join AP: OK\r\n");
    Serial.print("IP: ");
    Serial.println(wifi.getLocalIP().c_str());
  } else {
    Serial.print("Join AP: FAILED\r\n");
  }

  Serial.print("Single Mode: ");
  if (wifi.disableMUX()) {
    Serial.print("OK\r\n");
  } else {
    Serial.print("FAILED\r\n");
  }

  while (wifi.registerUDP(HOST_NAME, HOST_PORT)) {
    Serial.print("register UDP: FAILED, retrying...\r\n");
    wifi.unregisterUDP();
    wifi.registerUDP(HOST_NAME, HOST_PORT);
  }
  Serial.print("register UDP: OK\r\n");


  Serial.print("setup end\r\n");
}

void loop(void)
{
  uint8_t buffer[128] = {0};

  const char *hello = "Hello, this is client!\n";
  wifi.send((const uint8_t *)hello, strlen(hello));

  uint32_t len = wifi.recv(buffer, sizeof(buffer), 1000);
  if (len > 0) {
    Serial.print("Received:[");
    for (uint32_t i = 0; i < 64; i++) {
      Serial.print((char)buffer[i]);
    }
    Serial.print("]\r\n");
  }

  delay(1000);
}

