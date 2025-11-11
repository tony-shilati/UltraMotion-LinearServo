#include <FlexCAN_T4.h>
#include <math.h>
#include <QuadEncoder.h>
#include <Adafruit_NAU7802.h>
#include <Wire.h>

#define FREQUENCY_1 0.1f                  // Hz
#define FREQUENCY_2 5.0f                 // Hz
#define AMPLITUDE 865                 // Ticks
#define SWEEP_LENGTH 50.0f              // s
#define CENTER 8212

#define LC_CAL 0.012          // Load cell callibration (nextowns/tick) // NAU [0.00004893370999f] 
#define LC_PIN 14
#define LC_CAL_CYCLES 500




/*////////
 * Object Instantiation
 *////////
FlexCAN_T4        <CAN3, RX_SIZE_256, TX_SIZE_16> can3;
QuadEncoder       encoder1(3, 7, 5);  // ENC1 using pins 0 (A) and 1 (B)
Adafruit_NAU7802  nau;

IntervalTimer     servoTimer;
IntervalTimer     encoderTimer;
IntervalTimer     loadcellTimer;


/*////////
 * Global Vars
 *////////
uint16_t          number;
unsigned long     start;
unsigned long     start_micros;
bool started = false;
float lc_offset;

/*////////
 * Function Prototypes
 *////////
void readEncoder();
void printMessage(const CAN_message_t &m);
void commandServo();
void readLoadCell();


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
   * Config the load cell
   */////////
   pinMode(LC_PIN, INPUT);
   analogReadResolution(12);  // 12-bit ADC (0–4095)

  for (int i = 0; i < LC_CAL_CYCLES; i++) {
    lc_offset += analogRead(LC_PIN);
    delay(1);
  }
  lc_offset = lc_offset / (float)LC_CAL_CYCLES;


  /*////////
   * Start the linear servo commands
   *////////
  servoTimer.begin(commandServo, 1000); // Send position commands at 1kHz
  delay(1000);
  

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
  started = true;

  /*////////
   * Start the control and DAQ timers
   *////////
  encoderTimer.begin(readEncoder, 3125);  // 320 Hz timer
  delayMicroseconds(50);
  loadcellTimer.begin(readLoadCell, 3125);  // Read load cell at 320 Hz
  
  
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
  /*
  CAN_message_t msg;
  if (can3.read(msg)) {
    noInterrupts();
    printMessage(msg);
    interrupts();
  }
    */
}


/*////////
 * Function declarations
 *////////

void printMessage(const CAN_message_t &m) {
  // print telemetry data and timestamp

  if (m.len >= 2) {
    uint16_t value = (uint16_t)m.buf[0] | ((uint16_t)m.buf[1] << 8); // low byte first
  // Label CAN telemetry as load-cell formatted 'LC:data,time'
  Serial.print("LC:");
  Serial.print(value);           // decimal raw value
  Serial.print(",");
  Serial.println((micros() - start_micros)/1000000.0f, 6);
  } else {
    Serial.println("  (not enough data for 16-bit)");
  }
}

void readLoadCell(){
  Serial.print("LC: ");
  Serial.print((analogRead(LC_PIN) - lc_offset)*LC_CAL);
  Serial.print(", ");
  Serial.println((micros()-start_micros)/1000000.0f, 6);
}

void readEncoder(){
  Serial.print("E:");
  Serial.print(encoder1.read()*1e-3, 3);
  Serial.print(", ");
  Serial.println((micros() - start_micros)/1000000.0f, 6);
}

void commandServo(){

  if (!started){
    number = (float)CENTER;
  } else {

    float t = (millis() - start) / 1000.0f; // seconds
    float value = CENTER + AMPLITUDE *
        sinf(2.0f * PI * FREQUENCY_1 * SWEEP_LENGTH * ((pow(FREQUENCY_2/FREQUENCY_1, t/SWEEP_LENGTH) - 1) / (log(FREQUENCY_2/FREQUENCY_1))));
      number = (uint16_t)roundf(value);

    Serial.print("GND:");
    Serial.print(number);
    Serial.print(", ");
    Serial.println((micros() - start_micros)/1000000.0f, 6);
  }

  CAN_message_t tx;
  tx.id = 0x3;
  tx.len = 2; // Sending 2 bytes
  tx.buf[0] = number & 0xFF;        // Low byte
  tx.buf[1] = (number >> 8) & 0xFF; // High byte
  can3.write(tx);

  

}