#include <FlexCAN_T4.h>
#include <math.h>
#include <QuadEncoder.h>

#define FREQUENCY 10.0f                  // Hz
#define AMPLITUDE 5000                  // Ticks
#define SWEEP_LENGTH 5.0f              // s
#define CENTER 8212



/*////////
 * Object Instantiation
 *////////
FlexCAN_T4        <CAN3, RX_SIZE_256, TX_SIZE_16> can3;
QuadEncoder       encoder1(3, 7, 5);  // ENC1 using pins 0 (A) and 1 (B)

IntervalTimer     servoTimer;
IntervalTimer     encoderTimer;


/*////////
 * Global Vars
 *////////
uint16_t          number;
unsigned long     start;
unsigned long     start_micros;

/*////////
 * Function Prototypes
 *////////
void readEncoder();
void printMessage(const CAN_message_t &m);
void commandServo();


/*////////
 * Setup Code
 *////////
void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
  Serial.println("Ready to go");
  
  /*////////
   * Config the linear servo comms
   *////////
  can3.begin();
  can3.setBaudRate(1000000); // 1 Mbps
  

  /*////////
   * Configure the encoder
   *////////
  encoder1.setInitConfig();   // load default settings
  encoder1.init();            // initialize hardware encoder

  // Put the Quadrature chip in RS-422 mode
  pinMode(26, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);

  digitalWrite(26, LOW);
  digitalWrite(11, HIGH);
  digitalWrite(12, LOW);

  // Start the timers
  start = millis();
  start_micros = micros();

  /*////////
   * Start the control and DAQ timers
   *////////
  encoderTimer.begin(readEncoder, 3125);  // 320 Hz timer
  delayMicroseconds(300);
  servoTimer.begin(commandServo, 1000); // Send position commands at 1kHz
}


/*////////
 * Main Loop
 *////////
void loop() {
  if ((millis() - start) > SWEEP_LENGTH * 1000){
    noInterrupts();
    delay(3000);

    // Software reset of the teensy
    SCB_AIRCR = 0x05FA0004;
    interrupts();

  }

  // Receive any incoming CAN messages and print them to Serial
  CAN_message_t msg;
  if (can3.read(msg)) {
    printMessage(msg);
  }
}


/*////////
 * Function declarations
 *////////

void printMessage(const CAN_message_t &m) {
  // print telemetry data and timestamp

  if (m.len >= 2) {
    uint16_t value = (uint16_t)m.buf[0] | ((uint16_t)m.buf[1] << 8); // low byte first
    Serial.print("T:");
    Serial.print(value);           // decimal
    Serial.print(",");
    Serial.println((micros() - start_micros)/1000000.0f, 4);
  } else {
    Serial.println("  (not enough data for 16-bit)");
  }
}

void readEncoder(){
  Serial.print("E:");
  Serial.print(encoder1.read()*1e-3, 3);
  Serial.print(", ");
  Serial.println((micros() - start_micros)/1000000.0f, 4);
}

void commandServo(){

  float t = millis() / 1000.0f; // seconds
  const float center = (float)CENTER;
  float value = center + AMPLITUDE * sinf(2.0f * PI * FREQUENCY * t);
  number = (uint16_t)roundf(value);

  CAN_message_t tx;
  tx.id = 0x3;
  tx.len = 2; // Sending 2 bytes
  tx.buf[0] = number & 0xFF;        // Low byte
  tx.buf[1] = (number >> 8) & 0xFF; // High byte
  can3.write(tx);

}