#include <FlexCAN_T4.h>
#include <math.h>
#include <QuadEncoder.h>
#include <SPI.h>


#define FREQUENCY_1 0.1f                 // Hz
#define FREQUENCY_2 40.0f                 // Hz
#define INITIAL_AMPLITUDE_TICKS 2595.0f   // ticks
#define CENTER 8212                       // ticks
#define INITIAL_AMPLITUDE 1.75f            // mm
#define FINAL_AMPLITUDE 0.20f             // mm
#define SWEEP_LENGTH 380.0f                // s

#define LC_CAL 0.0000165527f                     // N/tick

#define SPI_BAUDRATE 6000000

#define CS1 0
#define DRDY 15


/*////////
 * Object Instantiation
 *////////
FlexCAN_T4        <CAN3, RX_SIZE_256, TX_SIZE_16> can3;
QuadEncoder       encoder1(3, 7, 5);  // ENC1 using pins 0 (A) and 1 (B)

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
byte outputBuffer[3];

long lc_sum = 0;
uint16_t lc_index = 0;

/*////////
 * Function Prototypes
 *////////
void readSensors();
void printMessage(const CAN_message_t &m);
void commandServo();
int32_t lcRead();



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

  /*////////
   * Config the load cell amp
   */////////
  // Initialize SPI1
  SPI1.begin();
  pinMode(CS1, OUTPUT);
  digitalWrite(CS1, HIGH);
  SPI1.beginTransaction(SPISettings(SPI_BAUDRATE, MSBFIRST, SPI_MODE1));
  
  // Format the ADC configuration data
  uint8_t config_msg[] = {0b01000001, 0b01101110, 0b11000100};
  uint8_t sync_byte = 0b00001000;

  // Write the configuration to the ADC
  digitalWrite(CS1, LOW);
  SPI1.transfer(config_msg, 3);
  digitalWrite(CS1, HIGH);

  // Sync the timer of the ADC and wait a specified time
  digitalWrite(CS1, LOW);
  SPI1.transfer(sync_byte);
  delayMicroseconds(200);
  digitalWrite(CS1, HIGH);

  Serial.println("Ready to go");

  // Start the timers
  start = millis();
  start_micros = micros();
  started = true;

  /*////////
   * Start the DAQ timers
   *////////
  //sensorsTimer.begin(readSensors, 1000);  // 1000 Hz timer
  attachInterrupt(DRDY, readSensors, FALLING);
  
  
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

void readSensors(){   // Total loop time with prints is 29 microseconds
  // put your main code here, to run repeatedly:
  int32_t lc_reading = lcRead();                       // 25.2 microseconds required   
  float enc_reading = encoder1.read();              // 112 nano seconds required (67 clock cycles)

  // Timestampe the sensor readings
  unsigned long sensors_time = micros();

  // Indicate sensor readings
  Serial.print("S:");

  // Print the load cell reading
  Serial.print(lc_reading*LC_CAL, 4);
  Serial.print(",");

  // Print the encoder reading
  Serial.print(enc_reading*1e-3, 3);
  Serial.print(",");

  // Print the timestamp
  Serial.println((sensors_time-start_micros)/1000000.0f, 6);
}

void commandServo(){ // Total loop time with prints is 25.7 microseconds
  // unsigned long local_timer = ARM_DWT_CYCCNT;

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

  // Serial.println((ARM_DWT_CYCCNT - local_timer));
}

int32_t lcRead(){
  uint8_t b0, b1, b2;

  digitalWrite(CS1, LOW);
  b0 = SPI1.transfer(0x00);
  b1 = SPI1.transfer(0x00);
  b2 = SPI1.transfer(0x00);
  digitalWrite(CS1, HIGH);

  int32_t adc = (int32_t)b0 << 16 | (int32_t)b1 << 8 | (int32_t)b2;

  // Sign-extend 24-bit to 32-bit
  if (adc & 0x800000) {  
    adc |= 0xFF000000;
  }

  return adc;
}