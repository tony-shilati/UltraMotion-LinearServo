#include <FlexCAN_T4.h>
#include <stdlib.h>
// Use CAN3 interface for the 3rd CAN bus on Teensy 4.1
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> can3;

void printMessage(const CAN_message_t &m) {
  // print timestamp, ID, type, len and data
  Serial.print(millis());
  Serial.print(" ms  ");
  Serial.print("ID: 0x");
  Serial.print(m.id, HEX);
  Serial.print("  len:");
  Serial.print(m.len);
  Serial.print("  data:");
  for (uint8_t i = 0; i < m.len && i < 8; ++i) {
    if (m.buf[i] < 0x10) Serial.print('0');
    Serial.print(m.buf[i], HEX);
    Serial.print(' ');
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  // give USB serial a moment to start
  unsigned long start = millis();
  while (!Serial && (millis() - start) < 2000) {
    delay(10);
  }
  Serial.println("Ready to go");
  
  can3.begin();
  can3.setBaudRate(1000000); // 1 Mbps
}

void loop() {

  // Receive any incoming CAN messages and print them to Serial
  CAN_message_t msg;
  if (can3.read(msg)) {
    printMessage(msg);
  }

  // Read numeric input lines from Serial (USB) and transmit to CAN3.
  // Expected input: a decimal integer per line, e.g. "3750\n".
  static char inBuf[64];
  static size_t inIdx = 0;
  while (Serial.available()) {
    int c = Serial.read();
    if (c < 0) continue;
    if (c == '\r') continue;
    if (c == '\n' || inIdx >= (sizeof(inBuf) - 1)) {
      // terminate
      inBuf[inIdx] = '\0';
      if (inIdx > 0) {
        // parse number
        char *endp = NULL;
        long val = strtol(inBuf, &endp, 10);
        if (endp == inBuf) {
          Serial.print("Invalid number: ");
          Serial.println(inBuf);
        } else {
          uint16_t number = (uint16_t)val;
          CAN_message_t tx;
          tx.id = 0x3;
          tx.len = 2;
          tx.buf[0] = number & 0xFF;
          tx.buf[1] = (number >> 8) & 0xFF;
          can3.write(tx);
          Serial.print("Transmitted: ");
          Serial.println(number);
        }
      }
      inIdx = 0;
    } else {
      inBuf[inIdx++] = (char)c;
    }
  }

  // small delay for background tasks
  delay(4);
}
