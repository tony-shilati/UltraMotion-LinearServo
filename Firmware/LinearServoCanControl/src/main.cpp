#include <FlexCAN_T4.h>
#include <math.h>
#include <QuadEncoder.h>

#define FREQUENCY_1 1.0f                  // Hz
#define FREQUENCY_2 15.0f                 // Hz
#define INITIAL_AMPLITUDE_TICKS 2595.0f   // ticks
#define INITIAL_AMPLITUDE 2.0f           // mm
#define FINAL_AMPLITUDE 1.0f             // mm
#define SWEEP_LENGTH 120.0f              // s
#define CENTER 8212

#define LC_PIN 14
#define NUM_FILTER_SAMPLES 40


/*////////
 * Object Instantiation
 *////////
FlexCAN_T4        <CAN3, RX_SIZE_256, TX_SIZE_16> can3;
QuadEncoder       encoder1(3, 7, 5);  // ENC1 using pins 0 (A) and 1 (B)

IntervalTimer     LCBufferTimer;
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
void updateBuffer();


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
   * Config the load cell amp
   */////////

  // Analog read pin
  pinMode(LC_PIN, INPUT);

  // Initialize buffer
  for (int i = 0; i < NUM_FILTER_SAMPLES; i++) {
    lc_samples[i] = analogRead(LC_PIN);
  }


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
  sensorsTimer.begin(readSensors, 2000);  // 500 Hz timer
  LCBufferTimer.begin(updateBuffer, 50);      // 20 kHz buffer update

  
  
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

void updateBuffer(){
  uint16_t newSample = analogRead(LC_PIN);
  lc_sum -= lc_samples[lc_index];
  lc_samples[lc_index] = newSample;

  // Increment and wrap index
  lc_index++;
  if (lc_index >= NUM_FILTER_SAMPLES) lc_index = 0;

  lc_sum += newSample;
}

void readSensors(){
  Serial.print("S:");



  // Compute averaged loadcell reading
  Serial.print((lc_sum / NUM_FILTER_SAMPLES), 4);
  Serial.print(",");

  // Sample the encoder
  Serial.print(encoder1.read()*1e-3, 3);
  Serial.print(",");

  // Time stamp
  Serial.println((micros()-start_micros)/1000000.0f, 6);
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