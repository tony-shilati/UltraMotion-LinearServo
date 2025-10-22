#include <FlexCAN_T4.h>
#include <math.h>

#define FREQUENCY_1 0.0f                  // Hz
#define FREQUENCY_2 20.0f                 // Hz
#define INITIAL_AMPLITUDE 1.93f           // mm
#define INITIAL_AMPLITUDE_TICKS 2800.0f   // mm
#define FINAL_AMPLITUDE 0.75f             // mm
#define SWEEP_LENGTH 60.0f                // s
#define SEND_INTERVAL 1                   // ms

static uint32_t lastMillis = 0;

// Use CAN3 interface for the 3rd CAN bus on Teensy 4.1
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> can3;

uint16_t number;
unsigned long start;
unsigned long start_micros;
float tau_inv = -log(FINAL_AMPLITUDE/INITIAL_AMPLITUDE) / SWEEP_LENGTH; // Inverse time constant of exponential decay

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
  if ((millis() - start) > SWEEP_LENGTH * 1000){
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
  

  if (millis() - lastMillis >= (uint32_t)SEND_INTERVAL) {
    lastMillis = millis();

    float t = millis() / 1000.0f; // seconds
    const float center = 8212.0f;
    float value = center + INITIAL_AMPLITUDE_TICKS * exp(-t*tau_inv) *
      sinf(2.0f * PI * ((FREQUENCY_1 * t) + (FREQUENCY_2 - FREQUENCY_1) / (2.0f*SWEEP_LENGTH) * t*t));
    number = (uint16_t)roundf(value);

    CAN_message_t tx;
    tx.id = 0x3;
    tx.len = 2; // Sending 2 bytes
    tx.buf[0] = number & 0xFF;        // Low byte
    tx.buf[1] = (number >> 8) & 0xFF; // High byte
    can3.write(tx);

  }

}
