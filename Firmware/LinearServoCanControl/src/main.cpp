#include <FlexCAN_T4.h>
#include <math.h>
#include <QuadEncoder.h>
#include <Adafruit_NAU7802.h>
#include <Wire.h>

#define FREQUENCY_1 0.1f                  // Hz
#define FREQUENCY_2 20.0f                 // Hz
#define INITIAL_AMPLITUDE_TICKS 2595.0f   // ticks
#define CENTER 8212                       // ticks
#define INITIAL_AMPLITUDE 1.75f            // mm
#define FINAL_AMPLITUDE 0.5f              // mm
#define SWEEP_LENGTH 420.0f               // s
#define LC_PIN 14
#define NUM_FILTER_SAMPLES 40
#define NAU_CAL 0.00001125444 // 1kg // 0.00004893370999f 10 kg          // Load cell callibration (nextowns/tick)


/*////////
 * Object Instantiation
 *////////
FlexCAN_T4        <CAN3, RX_SIZE_256, TX_SIZE_16> can3;
QuadEncoder       encoder1(3, 7, 5);  // ENC1 using pins 0 (A) and 1 (B)
Adafruit_NAU7802  nau;

IntervalTimer     servoTimer;
IntervalTimer     sensorsTimer;


/*////////
 * Global Vars
 *////////
uint16_t          number;
unsigned long     start;
unsigned long     start_micros;
bool started = false;
float tau_inv = -log(FINAL_AMPLITUDE/INITIAL_AMPLITUDE) / SWEEP_LENGTH; // Inverse time constant of exponential decay

long lc_sum = 0;
uint16_t lc_index = 0;
uint16_t lc_samples[NUM_FILTER_SAMPLES];

/*////////
 * Function Prototypes
 *////////
void readSensors();
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
  
  /*////////
   * Config the linear servo comms
   *////////
  can3.begin();
  can3.setBaudRate(1000000); // 1 Mbps

  /*////////
   * Config the load cell amp
   */////////
   Wire.begin();                        // Initialize I2C
  Wire.setClock(400000);              // Use I2C (400 kHz)

  if (!nau.begin(&Wire)) {
    Serial.println("NAU7802 not found!");
    while (1) {}
  }


  nau.setLDO(NAU7802_3V3);             // Match Teensy 3.3V supply
  nau.setGain(NAU7802_GAIN_128);       // Max gain for small load cell signals
  nau.setRate(NAU7802_RATE_320SPS);    // Maximum sample rate

  // Take 500 readings to flush out readings
  for (uint16_t i=0; i<500; i++) {
    while (! nau.available()) delay(1);
    nau.read();
  }

  while (! nau.calibrate(NAU7802_CALMOD_INTERNAL)) {
    delay(1000);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
  }

  while (! nau.calibrate(NAU7802_CALMOD_OFFSET)) {
    delay(1000);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
  }

  Serial.println("Ready to go");
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
   * Start the DAQ timers
   *////////
  sensorsTimer.begin(readSensors, 3125);  // 500 Hz timer
  delayMicroseconds(150);

  
  
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

}


/*////////
 * Function declarations
 *////////

void readSensors(){
  // Collect sensor readings
  double lc_reading = nau.read()*NAU_CAL;
  float enc_reading = encoder1.read();

  // Timestampe the sensor readings
  unsigned long sensors_time = micros();

  // Indicate sensor readings
  Serial.print("S:");

  // Print the load cell reading
  Serial.print(lc_reading, 4);
  Serial.print(",");

  // Print the encoder reading
  Serial.print(enc_reading*1e-3, 3);
  Serial.print(",");

  // Print the timestamp
  Serial.println((sensors_time-start_micros)/1000000.0f, 6);
}

void commandServo(){

  if (!started){
    number = (float)CENTER;
  } else {

    float t = (millis() - start) / 1000.0f; // seconds
    float value = CENTER + INITIAL_AMPLITUDE_TICKS * exp(-t*tau_inv) *
      sinf(2.0f * PI * FREQUENCY_1 * SWEEP_LENGTH * ((pow(FREQUENCY_2/FREQUENCY_1, t/SWEEP_LENGTH) - 1) / (log(FREQUENCY_2/FREQUENCY_1))));
      number = (uint16_t)roundf(value);

    Serial.print("G:");
    Serial.print(number);
    Serial.print(",");
    Serial.println((micros() - start_micros)/1000000.0f, 6);
  }

  CAN_message_t tx;
  tx.id = 0x3;
  tx.len = 2; // Sending 2 bytes
  tx.buf[0] = number & 0xFF;        // Low byte
  tx.buf[1] = (number >> 8) & 0xFF; // High byte
  can3.write(tx);


}