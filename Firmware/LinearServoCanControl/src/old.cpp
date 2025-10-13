// #include <Arduino.h>
// #include <FlexCAN_T4.h>

// // Create a CAN3 object for Teensy 4.1
// FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can3;

// static CAN_message_t msg;
// static CAN_message_t rxmsg;

// void setup() {
//   Serial.begin(115200);
//   while (!Serial && millis() < 5000);  // Wait up to 5s for Serial

//   Serial.println("Starting CAN3 example...");

//   // Initialize CAN3 at 1 Mbps
//   Can3.begin();
//   Can3.setBaudRate(1000000);

//   // Optional: Enable filters or listen to all frames
//   Can3.setMaxMB(16);     // Number of mailboxes
//   Can3.enableFIFO();     // Enable FIFO receive
//   Can3.enableFIFOInterrupt();
//   // Can3.onReceive(can3_rx); // Attach receive callback

//   Serial.println("CAN3 initialized.");

//   // // Write "LK unlock\r" at startup
//   const char* unlockCommand = "LK unlock\r";

//   msg.id = 0x03;
//   msg.len = strlen(unlockCommand);

//   for (size_t i = 0; i < strlen(unlockCommand); i++) {
//       msg.buf[i] = unlockCommand[i];
//   }
//   Can3.write(msg);
//   Serial.println("Sent Unlock Command");
// }

// void loop() {
//   // Example: send a CAN message every second
//   static uint32_t lastSend = 0;
//   if (millis() - lastSend > 1000) {
//     CAN_message_t sendMsg;
//     sendMsg.id = 0x03;
//     sendMsg.len = 2;

//     uint16_t pos = 15000; // Example position value

//     for (uint8_t i = 0; i < 2; i++) {
//       // Write the lower byte, then the upper byte
//       sendMsg.buf[i] = (pos >> (8 * i)) & 0xFF;
//     }

//     Can3.write(sendMsg);
//     Serial.println("Sent CAN3 message: ID 0x03");
//     lastSend = millis();
//   }

//   // FlexCAN_T4 handles RX via callback (see below)
// }

// // Callback function for received CAN frames
// void can3_rx(const CAN_message_t &rxmsg) {
//   Serial.print("Received CAN3 frame: ID 0x");
//   Serial.print(rxmsg.id, HEX);
//   Serial.print(" Data: ");
//   for (uint8_t i = 0; i < rxmsg.len; i++) {
//     Serial.print(rxmsg.buf[i], HEX);
//     Serial.print(" ");
//   }
//   Serial.println();
// }

// #include <FlexCAN_T4.h>

// FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> can3;

// void setup() {
//   Serial.begin(115200);
//   delay(1000);

//   can3.begin();
//   can3.setBaudRate(1000000); // 1 Mbps
// }

// void loop() {
//   uint16_t number = 3000; // The number you want to send

//   CAN_message_t msg;
//   msg.id = 0x3;
//   msg.len = 2; // Sending 2 bytes

//   msg.buf[0] = number & 0xFF;        // Low byte
//   msg.buf[1] = (number >> 8) & 0xFF; // High byte

//   can3.write(msg);
//   Serial.print("Sent 3000 as: ");
//   Serial.print(msg.buf[0], HEX);
//   Serial.print(" ");
//   Serial.println(msg.buf[1], HEX);

//   delay(1000);
// }