#include <FlexCAN_T4.h>
#include <math.h>
#include <QuadEncoder.h>
#include <ADS1256.h>

#define USE_SPI SPI1
#define FREQUENCY_1 10.0f                 // Hz
#define FREQUENCY_2 60.0f                 // Hz
#define INITIAL_AMPLITUDE_TICKS 1700.0f //2595.0f   // ticks
#define CENTER 8212                       // ticks
#define INITIAL_AMPLITUDE 1.0f // 1.75f            // mm
#define FINAL_AMPLITUDE 0.05f             // mm
#define SWEEP_LENGTH 120.0f                // s
#define LC_CAL 32.7f                     // Kg/V

#define CS1 0
#define DRDY 38


/*////////
 * Object Instantiation
 *////////
FlexCAN_T4        <CAN3, RX_SIZE_256, TX_SIZE_16> can3;
QuadEncoder       encoder1(3, 7, 5);  // ENC1 using pins 0 (A) and 1 (B)

IntervalTimer     servoTimer;
IntervalTimer     sensorsTimer;
ADS1256           ADS(DRDY, ADS1256::PIN_UNUSED, 37, CS1, 2.500, &USE_SPI); //DRDY, RESET, SYNC(PDWN), CS, VREF(float).    //Teensy 4.0 - OK


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
float lcRead();



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
  ADS.InitializeADC();
  ADS.setPGA(PGA_1);
  ADS.setMUX(DIFF_0_1);
  ADS.setDRATE(DRATE_1000SPS);

  Serial.println("Ready to go");

  // Start the timers
  start = millis();
  start_micros = micros();
  started = true;

  /*////////
   * Start the DAQ timers
   *////////
  ADS.sendDirectCommand(SELFCAL);
  sensorsTimer.begin(readSensors, 1000);  // 1000 Hz timer
  // attachInterrupt(DRDY, dataReadyISR, FALLING);
  
  
}


/*////////
 * Main Loop
 *////////
void loop() {
  if ((millis() - start) > SWEEP_LENGTH * 1000){
    noInterrupts();
    ADS.stopConversion();
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
  long lc_reading = lcRead();                       // 25.2 microseconds required   
  float enc_reading = encoder1.read();              // 112 nano seconds required (67 clock cycles)

  // Timestampe the sensor readings
  unsigned long sensors_time = micros();

  // Indicate sensor readings
  Serial.print("S:");

  // Print the load cell reading
  Serial.print((ADS.convertToVoltage(lc_reading)) * 1000.0f, 4);
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
    /*
    Serial.print("G:");
    Serial.print(number);
    Serial.print(",");
    Serial.println((micros() - start_micros)/1000000.0f, 6);
    */
  }

  CAN_message_t tx;
  tx.id = 0x3;
  tx.len = 2; // Sending 2 bytes
  tx.buf[0] = number & 0xFF;        // Low byte
  tx.buf[1] = (number >> 8) & 0xFF; // High byte
  can3.write(tx);

  // Serial.println((ARM_DWT_CYCCNT - local_timer));
}

float lcRead(){
  // put your main code here, to run repeatedly:
  SPI1.beginTransaction(SPISettings(1920000, MSBFIRST, SPI_MODE1));
  digitalWrite(0, LOW);
  SPI1.transfer(0b00000001); //Issue RDATA (0000 0001) command
  delayMicroseconds(7);

  outputBuffer[0] = SPI1.transfer(0); // MSB
  outputBuffer[1] = SPI1.transfer(0); // Mid-byte
  outputBuffer[2] = SPI1.transfer(0); // LSB    

  //Shifting and combining the above three items into a single, 24-bit number
  long lc_reading = ((long)outputBuffer[0]<<16) | ((long)outputBuffer[1]<<8) | (outputBuffer[2]);

  digitalWrite(0, HIGH);
  SPI1.endTransaction(); 

  // sign extension from 24-bit to resolution of long
  return ((lc_reading) & (1l << 23) ? (lc_reading) - 0x1000000 : lc_reading);
}