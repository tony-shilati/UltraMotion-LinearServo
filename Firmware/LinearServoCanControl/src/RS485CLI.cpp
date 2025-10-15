/*
// RS485_Transmit.cpp
// For Teensy: read from USB Serial and transmit out Serial2 TX

#include <Arduino.h>

// Configure baud rates
static const uint32_t USB_BAUD = 115200;
static const uint32_t RS485_BAUD = 115200; // adjust as needed



void setup() {
  Serial.begin(USB_BAUD);
  // Wait briefly for Serial to be ready (USB)
  unsigned long start = millis();
  while (!Serial && (millis() - start) < 2000) {
    ;
  }
  Serial.println("RS485 forwarder starting...");

  // Serial2 is TX2/RX2 on Teensy (pins depend on board)
  Serial2.begin(RS485_BAUD);

}

void loop() {
  // Raw passthrough: forward bytes immediately both directions
    if (Serial.available()) {
      while (Serial.available()) {
        int b = Serial.read();
        if (b >= 0) Serial2.write((uint8_t)b);
      }
      // Don't flush here; let UART pacing handle it
    }

    while (Serial2.available()) {
      int b = Serial2.read();
      if (b >= 0) Serial.write((uint8_t)b);
    }

  // Echo any data coming from Serial2 back to USB Serial for debugging
  // Buffer incoming bytes and print full lines with Serial.println()
  static char lineBuf[256];
  static size_t lineIdx = 0;
  static unsigned long lastByteMs = 0;
  const unsigned long lineTimeoutMs = 200; // flush partial line after timeout

  while (Serial2.available()) {
    int b = Serial2.read();
    if (b < 0) continue;
    lastByteMs = millis();
    // normalize CRLF: treat '\r' as ignored, '\n' as line end
    if (b == '\r') {
      continue;
    } else if (b == '\n') {
      // terminate and print
      lineBuf[lineIdx] = '\0';
      if (lineIdx == 0) {
        // empty line -> print empty line
        Serial.println();
      } else {
        Serial.println(lineBuf);
        lineIdx = 0;
      }
    } else {
      if (lineIdx < (sizeof(lineBuf) - 1)) {
        lineBuf[lineIdx++] = (char)b;
      } else {
        // buffer full: terminate, print and reset
        lineBuf[lineIdx] = '\0';
        Serial.println(lineBuf);
        lineIdx = 0;
      }
    }
  }

  // flush partial line if timeout elapsed
  if (lineIdx > 0 && (millis() - lastByteMs) >= lineTimeoutMs) {
    lineBuf[lineIdx] = '\0';
    Serial.println(lineBuf);
    lineIdx = 0;
  }

  delay(2);
}

*/