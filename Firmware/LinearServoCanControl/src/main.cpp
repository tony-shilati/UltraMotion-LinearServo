#include <FlexCAN_T4.h>
#include <math.h>
#include <QuadEncoder.h>

/*
 * Defines
 */
#define POS1 4000                      // Position 1 ticks
#define POS2 10000                     // Position 2 ticks
#define PULSE_WIDTH 2                  // s
#define TRIAL_LENGTH 7.0f             // s

/*
 * Global vars
 */
static bool position = false;                 // 0 -> POS1; 1 -> POS2   
unsigned long start;
unsigned long start_micros;

/*
 * Object instantiation
 */
FlexCAN_T4        <CAN3, RX_SIZE_256, TX_SIZE_16> can3;
QuadEncoder       encoder1(3, 7, 5);  // ENC1 using pins 0 (A) and 1 (B)
IntervalTimer     encoderTimer;
IntervalTimer     servoTimer;
IntervalTimer     flipPos;


/*
 * Prototype functions
 */
void printMessage(const CAN_message_t &m);
void commandServo();
void flipPosfcn();
void readEncoder();



void setup() {

  /*
   * Send the servo to POS1
   */ 

  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
  Serial.println("Ready to go");

  /*
   *  Start CAN
   */
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
  
  /*
   *  Start timers
   */

  start = millis();
  start_micros = micros();
  servoTimer.begin(commandServo, 1000 * 10);                // Send position commands at 100 Hz
  encoderTimer.begin(readEncoder, 1000);               // Read encoder at 100 Hz
  flipPos.begin(flipPosfcn, PULSE_WIDTH * 1000 * 1000);     // Flip the position
  

}

void loop() {
  if ((millis() - start) > TRIAL_LENGTH * 1000){
    noInterrupts();
    delay(5000);

    // Software reset of the teensy
    SCB_AIRCR = 0x05FA0004;
    interrupts();

  }

  // Receive any incoming CAN messages and print them to Serial
  noInterrupts();
  CAN_message_t msg;
  if (can3.read(msg)) {
    printMessage(msg);
  }
  interrupts();


  delay(1);
}


/*
 * Function definitions
 */

void flipPosfcn(){
  position = !position;
}

void printMessage(const CAN_message_t &m) {
  // print telemetry data and timestamp

  if (m.len >= 2) {
    uint16_t value = (uint16_t)m.buf[0] | ((uint16_t)m.buf[1] << 8); // low byte first
    Serial.print("T: ");           
    Serial.print(value);           // decimal
    Serial.print(", ");
    Serial.println((micros() - start_micros)/1000000.0f, 4);
  } else {
    Serial.println("  (not enough data for 16-bit)");
  }
}

void commandServo(){
  // Determine position
  static uint16_t pos;
  if (position){
      pos = (uint16_t)POS2;
  } else {
      pos = (uint16_t)POS1;
  }

  // Write position
  CAN_message_t tx;
  tx.id = 0x3;
  tx.len = 2; // Sending 2 bytes
  tx.buf[0] = pos & 0xFF;        // Low byte
  tx.buf[1] = (pos >> 8) & 0xFF; // High byte
  can3.write(tx);

  Serial.print("G: ");          
  Serial.print(pos); 
  Serial.print(","); 
  Serial.println((micros() - start_micros)/1000000.0f, 4);         

}

void readEncoder(){
  Serial.print("E: ");
  Serial.print(encoder1.read() * 1e-3);
  Serial.print(", ");
  Serial.println((micros() - start_micros)/1000000);
}


