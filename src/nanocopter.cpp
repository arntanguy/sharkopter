#include <Arduino.h>
#include <Wire.h>
#include "I2Cdev/I2Cdev.h"
#include "MPU6050/MPU6050_6Axis_MotionApps20.h"

MPU6050 mpu;

bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// ======== RAW VALUES =======
int16_t ax, ay, az;
int16_t gx, gy, gz;
double timeStep, time, timePrev;
double arx, ary, arz, grx, gry, grz, gsx, gsy, gsz, rx, ry, rz;
double gyroScale = 131;
int i;

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

// ================================================================
// ==                 MOTOR STUFF                                ==
// ================================================================
#define DEBUG_LED 13
#define TOPLEFT 3
#define BOTTOMRIGHT 10 
#define TOPRIGHT 11 
#define BOTTOMLEFT 9 


void shutdown_motors()
{
  analogWrite(TOPLEFT, 0); 
  analogWrite(BOTTOMRIGHT, 0); 
  analogWrite(TOPRIGHT, 0); 
  analogWrite(BOTTOMLEFT, 0); 
}

void motor_check_sequential(int speed=125) {
  digitalWrite(DEBUG_LED, HIGH);
  Serial.print("Testing motor 1 at fullspeed\n");
  analogWrite(TOPLEFT, speed); 
  delay(1000);
  Serial.print("Testing motor 2 at fullspeed\n");
  analogWrite(TOPLEFT, 0); 
  analogWrite(BOTTOMRIGHT, speed); 
  delay(1000);
  Serial.print("Testing motor 3 at fullspeed\n");
  analogWrite(BOTTOMRIGHT, 0); 
  analogWrite(TOPRIGHT, speed); 
  delay(1000);
  Serial.print("Testing motor 4 at fullspeed\n");
  analogWrite(TOPRIGHT, 0); 
  analogWrite(BOTTOMLEFT, speed); 
  delay(1000);
  analogWrite(BOTTOMLEFT, 0); 
  digitalWrite(DEBUG_LED, LOW);
}

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
  pinMode(TOPLEFT, OUTPUT);
  pinMode(BOTTOMRIGHT, OUTPUT);
  pinMode(TOPRIGHT, OUTPUT);
  pinMode(BOTTOMLEFT, OUTPUT);
  pinMode(DEBUG_LED, OUTPUT);

  Serial.begin(115200);

  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
  // 400KHZ
  //Fastwire::setup(400, true);

  //while (!Serial); // wait for Leonardo enumeration, others continue immediately

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity

  // Calibrated offsets
  float offsets[] =   {	-1503,	1153,	5128,	57,	26,	62};
  mpu.setXGyroOffset(offsets[0]);
  mpu.setXGyroOffset(offsets[1]);
  mpu.setZAccelOffset(offsets[2]);
  mpu.setXGyroOffset(offsets[3]);
  mpu.setYGyroOffset(offsets[4]);
  mpu.setZGyroOffset(offsets[5]);

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  digitalWrite(DEBUG_LED, LOW);
  shutdown_motors();

  digitalWrite(DEBUG_LED, HIGH);
  delay(6000);
  //motor_check_sequential();
  digitalWrite(DEBUG_LED, LOW);

  time = millis();
  i = 1;
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {
    // other program behavior stuff here
    // .
    // .
    // .
    // if you are really paranoid you can frequently test in between other
    // stuff to see if mpuInterrupt is true, and if so, "break;" from the
    // while() loop to immediately process the MPU data
    // .
    // .
    // .
  }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    //while(fifoCount > packetSize) {
    //  fifoCount = mpu.getFIFOCount();
    //  mpu.getFIFOBytes(fifoBuffer, packetSize);
    //  fifoCount -= packetSize;
    //}
    //mpu.getFIFOBytes(fifoBuffer, packetSize);


    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;
    if(fifoCount>packetSize) mpu.resetFIFO();

    // set up time for integration
    timePrev = time;
    time = millis();
    timeStep = (time - timePrev) / 1000; // time-step in s

    // collect readings
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // apply gyro scale from datasheet
    gsx = gx/gyroScale;   gsy = gy/gyroScale;   gsz = gz/gyroScale;

    // calculate accelerometer angles
    arx = (180.f/3.141592) * atan(ax / sqrt(square(ay) + square(az))); 
    ary = (180.f/3.141592) * atan(ay / sqrt(square(ax) + square(az)));
    arz = (180.f/3.141592) * atan(sqrt(square(ay) + square(ax)) / az);

    // set initial values equal to accel values
    if (i == 1) {
      grx = arx;
      gry = ary;
      grz = arz;
    }
    // integrate to find the gyro angle
    else{
      grx = grx + (timeStep * gsx);
      gry = gry + (timeStep * gsy);
      grz = grz + (timeStep * gsz);
    }  

    // apply filter
    rx = (0.1 * arx) + (0.9 * grx);
    ry = (0.1 * ary) + (0.9 * gry);
    rz = (0.1 * arz) + (0.9 * grz);

    // print result
    Serial.print(i);   Serial.print("\t");
    Serial.print(timePrev);   Serial.print("\t");
    Serial.print(time);   Serial.print("\t");
    Serial.print(timeStep, 5);   Serial.print("\t\t");
    Serial.print(ax);   Serial.print("\t");
    Serial.print(ay);   Serial.print("\t");
    Serial.print(az);   Serial.print("\t\t");
    Serial.print(gx);   Serial.print("\t");
    Serial.print(gy);   Serial.print("\t");
    Serial.print(gz);   Serial.print("\t\t");
    Serial.print(arx);   Serial.print("\t");
    Serial.print(ary);   Serial.print("\t");
    Serial.print(arz);   Serial.print("\t\t");
    Serial.print(grx);   Serial.print("\t");
    Serial.print(gry);   Serial.print("\t");
    Serial.print(grz);   Serial.print("\t\t");
    Serial.print(rx);   Serial.print("\t");
    Serial.print(ry);   Serial.print("\t");
    Serial.println(rz);

    int rest_lift = 125;

    int left_control =  255-map(rx, -20, 0, 0, 255);
    int right_control = map(rx, 0, 20, 0, 255);
    int top_control = 255-map(ry, -20, 0, 0, 255);
    int bottom_control = map(ry, 0, 20, 0, 255);

    int topleft     = constrain(rest_lift + left_control + top_control    , 0, 255);
    int topright    = constrain(rest_lift + right_control + top_control   , 0, 255);
    int bottomleft  = constrain(rest_lift + left_control + bottom_control , 0, 255);
    int bottomright = constrain(rest_lift + right_control + bottom_control, 0, 255);

    analogWrite(TOPLEFT, topleft); 
    analogWrite(BOTTOMRIGHT, bottomright); 
    analogWrite(TOPRIGHT, topright); 
    analogWrite(BOTTOMLEFT, bottomleft); 

    blinkState = !blinkState;
    digitalWrite(DEBUG_LED, blinkState);
  }
}
