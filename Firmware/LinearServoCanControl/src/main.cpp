#include <FlexCAN_T4.h>
#include <math.h>
#define SEND_EXAMPLE
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

  // Optional: example transmitter (enable by defining SEND_EXAMPLE)
#ifdef SEND_EXAMPLE
  static uint32_t lastMillis = 0;
  const uint32_t sendIntervalMs = 100; // send every 100 ms
  if (millis() - lastMillis >= sendIntervalMs) {
    lastMillis = millis();
    // Oscillate between 3500 and 4500 at 0.1 Hz (period = 10 s)
    float t = millis() / 1000.0f; // seconds
    const float freq = 0.5f; // Hz
    const float amplitude = 4000.0f; // half-range (4500-3500)/2
    const float center = 8212.0f;
    float value = center + amplitude * sinf(2.0f * M_PI * freq * t);
    uint16_t number = (uint16_t)roundf(value);

    CAN_message_t tx;
    tx.id = 0x3;
    tx.len = 2; // Sending 2 bytes
    tx.buf[0] = number & 0xFF;        // Low byte
    tx.buf[1] = (number >> 8) & 0xFF; // High byte
    can3.write(tx);

    Serial.print("Sent value: ");
    Serial.print(number);
    Serial.print("  bytes: ");
    if (tx.buf[0] < 0x10) Serial.print('0');
    Serial.print(tx.buf[0], HEX);
    Serial.print(" ");
    if (tx.buf[1] < 0x10) Serial.print('0');
    Serial.println(tx.buf[1], HEX);
  }
#endif

  // small delay for background tasks
  delay(4);
}
