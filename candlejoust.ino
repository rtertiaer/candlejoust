// https://github.com/ElectronicCats/mpu6050
#include "MPU6050_6Axis_MotionApps20.h"

#include "I2Cdev.h"
#include <Wire.h>
#include <Adafruit_NeoPixel.h>

const int MPU_addr=0x68;  // I2C address of the MPU-6050
const int ACCEL_ADDR = 0x3B; 
const int POWER_ADDR = 0x6B; // PWR_MGMT_1 register
const int LED_DATA_PIN = 16; // Pin to communicate with LED; old values of 16, 5
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
const int BUTTON_PIN = 6; // button pin used to restart game
const double WARN_ACCEL = 2800;
const double MAX_ACCEL = 3500;
const int16_t BRIGHTNESS = 200;

bool running;

Adafruit_NeoPixel strip(1, LED_DATA_PIN, NEO_GRB + NEO_KHZ800);

// I'm colorblind lol. fetched from here: https://rgbcolorcode.com/color/flame
const uint32_t FLAME = strip.gamma32(strip.Color(88, 226, 34));
const uint32_t BLUE = strip.Color(0, 0, 255);
const uint32_t BLANK = strip.Color(0, 0, 0);


// These chunks were copied from examples in https://github.com/ElectronicCats/mpu6050
MPU6050 accelerometer;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// A class to represent the acceleration vectors and interact with the accelerometer. This covers way too many concerns - imo accelerometer logic shouldn't be baked into here - but hey, this entire project is a hack and it's small & functional ;)
class xyz {
  public:
    int16_t X, Y, Z;
    double totalAccel;

    xyz(void) {
      X = Y = Z = 0;
    }

    xyz get_values() {
      // display initial world-frame acceleration, adjusted to remove gravity
      // and rotated based on known orientation from quaternion
      accelerometer.dmpGetQuaternion(&q, fifoBuffer);
      accelerometer.dmpGetAccel(&aa, fifoBuffer);
      accelerometer.dmpGetGravity(&gravity, &q);
      accelerometer.dmpGetLinearAccel(&aaReal, &aa, &gravity);
      accelerometer.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);

      Serial.print("aaWorld\t");
      Serial.print(aaWorld.x);
      Serial.print("\t");
      Serial.print(aaWorld.y);
      Serial.print("\t");
      Serial.println(aaWorld.z);
      
      X = aaWorld.x;
      Y = aaWorld.y;
      Z = aaWorld.z;
      

      // display real acceleration, adjusted to remove gravity
      /*accelerometer.dmpGetQuaternion(&q, fifoBuffer);
      accelerometer.dmpGetAccel(&aa, fifoBuffer);
      accelerometer.dmpGetGravity(&gravity, &q);
      accelerometer.dmpGetLinearAccel(&aaReal, &aa, &gravity);
      Serial.print("areal\t");
      Serial.print(aaReal.x);
      Serial.print("\t");
      Serial.print(aaReal.y);
      Serial.print("\t");
      Serial.println(aaReal.z);
     
      X = aaReal.x;
      Y = aaReal.y;
      Z = aaReal.z;
      */
      
      totalAccel = total_accel();

      return *this;
    }
    
    xyz& operator=(xyz volatile& rhs) {
      X = rhs.X;
      Y = rhs.Y;
      Z = rhs.Z;
      return *this;
    }

    // de gua's theorem, applied to acceleration vectors
    double total_accel() {
      double t;
      t = sqrt(pow(X, 2) + pow(Y, 2) + pow(Z, 2));
      //Serial.print("total_accel(): ");
      //Serial.println(t);
      return t;
    }

    bool warn() {
      if(totalAccel > WARN_ACCEL) {
        Serial.print("totalAccel: ");
        Serial.println(totalAccel);
        return true;
      }
      else
        return false;
    }
    
    bool fail() {
      if(totalAccel > MAX_ACCEL) {
        Serial.print("totalAccel: ");
        Serial.println(totalAccel);
        return true;
      }
      else
        return false;
    }
};

xyz accel;

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

// a great deal of this code was copied & pasted from the MPU-6050 examples in the ElectronicCats library: https://github.com/ElectronicCats/mpu6050
void init_gyro(){
  Serial.print("init_gyro() called\n");
  Wire.begin();

  accelerometer.initialize();

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelerometer.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = accelerometer.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  accelerometer.setXGyroOffset(220);
  accelerometer.setYGyroOffset(76);
  accelerometer.setZGyroOffset(-85);
  accelerometer.setZAccelOffset(1688); // 1688 factory default for my test chip
  
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    accelerometer.CalibrateAccel(6);
    accelerometer.CalibrateGyro(6);
    accelerometer.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    accelerometer.setDMPEnabled(true);
        
    // enable Arduino interrupt detection
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = accelerometer.getIntStatus();
    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;
    // get expected DMP packet size for later comparison
    packetSize = accelerometer.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  Serial.print("MPU-6050 initialized\n");
}

// stop the game.
// TODO: set gyro/accel to low power mode, provided
//   we don't have to re-calibrate
void stop(){
  Serial.println("stop() called\n");
  running = false;
  strip.show();
  strip.setPixelColor(0, BLUE);
  strip.show();
  Serial.print("Ending. Press the reset button to continue\n");
}

// use the LED to warn the player that you're close
// to being too fast. there are some timing oddities -
// the game is effectively paused when this is happening.
void warning(){
  Serial.println("warning() called");
  strip.show();
  for (int i=0; i < 4; i++) {
    strip.setPixelColor(0, BLANK);
    strip.show();
    delay(50);
    strip.setPixelColor(0, FLAME);
    strip.show();
    delay(50);
  }
}

void start() {
  Serial.println("start() called");
  running = true;
  strip.setPixelColor(0, FLAME);
  strip.show();
}

void setup(){
  Serial.begin(9600); // this is for the board output for testing
  Serial.print("started serial port in setup();\n");

  strip.begin();
  strip.setBrightness(BRIGHTNESS);
  strip.setPixelColor(0, BLANK);
  strip.show();
  
  init_gyro();

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  
  start();

  delay(150); // initial readings for the Z axis take some time to normalize

  Serial.print("setup() exiting\n");
}

void loop(){
  if (!dmpReady) return;

  // if the button has been pressed, restart the game (without all the pesky
  // hardware calibration)
  if (!running && digitalRead(BUTTON_PIN) == LOW) {
    start();
  }
  
  // the timing here is still a bit wacky (there are edge cases) but should be good enough
  if(running && accelerometer.dmpGetCurrentFIFOPacket(fifoBuffer)) { // gets current packet from DMP
    accel.get_values();

    if(accel.fail()){
      stop();
      return;
    }
    if(accel.warn()){
      warning();
      // discard latest data, as it seems cumulative over the course of the warning sequence
      accelerometer.dmpGetCurrentFIFOPacket(fifoBuffer);
      return;
    }

    // scale brightness down, like a real candle
    strip.setPixelColor(0, FLAME); // https://github.com/adafruit/Adafruit_NeoPixel/blob/7f3ebe002a270ebf5298200c411085bba6ad131d/Adafruit_NeoPixel.cpp#L3359
    strip.setBrightness(BRIGHTNESS - ( BRIGHTNESS * (accel.total_accel() / MAX_ACCEL )));
    strip.show();
  }
  
}
