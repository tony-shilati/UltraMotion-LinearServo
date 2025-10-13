#include <FlexCAN_T4.h>

FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> can3;

void setup() {
  Serial.begin(115200);
  delay(1000);

  can3.begin();
  can3.setBaudRate(1000000); // 1 Mbps
}

void loop() {
  uint16_t number = 3000; // Position command

  CAN_message_t msg;
  msg.id = 0x3;
  msg.len = 2; // Sending 2 bytes

  msg.buf[0] = number & 0xFF;        // Low byte
  msg.buf[1] = (number >> 8) & 0xFF; // High byte

  can3.write(msg);
  Serial.print("Sent 3000 as: ");
  Serial.print(msg.buf[0], HEX);
  Serial.print(" ");
  Serial.println(msg.buf[1], HEX);

  delay(1000);
}