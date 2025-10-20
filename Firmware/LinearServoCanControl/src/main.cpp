#include <FlexCAN_T4.h>
#include <math.h>
#define SEND_EXAMPLE
#define FREQUENCY 1.0f
// Use CAN3 interface for the 3rd CAN bus on Teensy 4.1
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> can3;

uint16_t number;
unsigned long start;
unsigned long start_micros;
unsigned long time_limit = 10 * 1000;

void printMessage(const CAN_message_t &m) {
  // print telemetry data and timestamp

  if (m.len >= 2) {
    uint16_t value = (uint16_t)m.buf[0] | ((uint16_t)m.buf[1] << 8); // low byte first
    Serial.print(value);           // decimal
    Serial.print(", ");
    Serial.println((micros() - start_micros)/1000000.0f, 4);
  } else {
    Serial.println("  (not enough data for 16-bit)");
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
  Serial.println("Ready to go");
  
  can3.begin();
  can3.setBaudRate(1000000); // 1 Mbps
  
  start = millis();
  start_micros = micros();
}

void loop() {
  if ((millis() - start) > time_limit){
    delay(3000);

    // Software reset of the teensy
    SCB_AIRCR = 0x05FA0004;

  }

  // Receive any incoming CAN messages and print them to Serial
  CAN_message_t msg;
  if (can3.read(msg)) {
    printMessage(msg);
  }

  // Optional: example transmitter (enable by defining SEND_EXAMPLE)
#ifdef SEND_EXAMPLE
  static uint32_t lastMillis = 0;
  const uint32_t sendIntervalMs = 1; // Send period

  if (millis() - lastMillis >= sendIntervalMs) {
    lastMillis = millis();

    float t = millis() / 1000.0f; // seconds
    const float freq = FREQUENCY; // Hz
    const float amplitude = 700.0f;
    const float center = 8212.0f;
    float value = center + amplitude * sinf(2.0f * M_PI * freq * t);
    number = (uint16_t)roundf(value);

    CAN_message_t tx;
    tx.id = 0x3;
    tx.len = 2; // Sending 2 bytes
    tx.buf[0] = number & 0xFF;        // Low byte
    tx.buf[1] = (number >> 8) & 0xFF; // High byte
    can3.write(tx);
    /*
    Serial.print("Sent value: ");
    Serial.print(number);
    Serial.print("  bytes: ");
    if (tx.buf[0] < 0x10) Serial.print('0');
    Serial.print(tx.buf[0], HEX);
    Serial.print(" ");
    if (tx.buf[1] < 0x10) Serial.print('0');
    Serial.println(tx.buf[1], HEX);
    */
  }
#endif

}
