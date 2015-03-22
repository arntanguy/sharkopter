#include <Arduino.h>
#include <Wire.h>
#include "I2Cdev/I2Cdev.h"
#include "MPU6050/MPU6050_6Axis_MotionApps20.h"
#include "PID_v1/PID_v1.h"

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
double arx, ary, arz, grx, gry, grz, gsx, gsy, gsz, ypr[3];
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


// =================================================================
// ==                   Flight controller                         ==
// =================================================================

/**
 * Controllers
 */
double nsOut, nsSet = 0;
::PID* nsController = new PID(&ypr[0], &nsOut, &nsSet, 7, 0.0, 1.1, REVERSE);

double ewOut, ewSet = 0;
::PID* ewController = new PID(&ypr[1], &ewOut, &ewSet, 10.0, 0.06, 1.5, REVERSE);

double yawOut, yawSet = 0;
::PID* yawController = new PID(&ypr[0], &yawOut, &yawSet, 2.0, 0.0, 0.0, DIRECT);

/*
 * Flight variables
 */

int initSpeed = 1000;
long lastWatchdog = 0;

/**
 * Debug
 **/
void printPIDGains(){
	Serial.print("NS] P :: ");
	Serial.print(nsController->GetKp());
	Serial.print("   I :: ");
	Serial.print(nsController->GetKi());
	Serial.print("   D :: ");
	Serial.print(nsController->GetKd());
	Serial.print("   EW] P :: ");
	Serial.print(ewController->GetKp());
	Serial.print("   I :: ");
	Serial.print(ewController->GetKi());
	Serial.print("   D :: ");
	Serial.print(ewController->GetKd());
	Serial.print("   YAW] P :: ");
	Serial.print(yawController->GetKp());
	Serial.print("   I :: ");
	Serial.print(yawController->GetKi());
	Serial.print("   D :: ");
	Serial.print(yawController->GetKd());
}

void printHelp() {
  Serial.println("Z - Enter run mode");
  Serial.println("I - Increase base speed");
  Serial.println("K - Decrease base speed");
  Serial.println("O - Increase init speed");
  Serial.println("L - Decrease init speed");
  Serial.println("Q - Increase P gain");
  Serial.println("A - Decrease P gain");
  Serial.println("W - Increase I gain");
  Serial.println("S - Decrease I gain");
  Serial.println("E - Increase D gain");
  Serial.println("D - Decrease D gain");
}


/*
 * Initializer functions
 */

void initControllers()
{
	nsController->SetOutputLimits(-1000, 1000);
	nsController->SetMode(AUTOMATIC);
	ewController->SetOutputLimits(-1000, 1000);
	ewController->SetMode(AUTOMATIC);
	yawController->SetOutputLimits(-300, 300);
	yawController->SetMode(AUTOMATIC);
}

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================
// MPU uses interuptions to indicate when data is available in the FIFO buffer
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


  Serial.println(F("Initializing flight controllers..."));
  initControllers();


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

  printPIDGains();
  Serial.println();
  printHelp();
  Serial.println(F("Press X to start"));
	//while(true){
	//	if(Serial.available()) { // Wait for initialization command from user
	//		if(Serial.read() == 'X') break;
	//	}
	//}
  digitalWrite(DEBUG_LED, HIGH);
  //motor_check_sequential();
  digitalWrite(DEBUG_LED, LOW);

  time = millis();
  lastWatchdog = millis();
  i = 1;
  delay(6000);
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

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

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
    ypr[0] = (0.1 * arx) + (0.9 * grx);
    ypr[1] = (0.1 * ary) + (0.9 * gry);
    ypr[2] = (0.1 * arz) + (0.9 * grz);

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
    Serial.print(ypr[0]);   Serial.print("\t");
    Serial.print(ypr[1]);   Serial.print("\t");
    Serial.println(ypr[2]);

    int rest_lift = 125;

    int left_control =  255-map(ypr[0], -20, 0, 0, 255);
    int right_control = map(ypr[0], 0, 20, 0, 255);
    int top_control = 255-map(ypr[1], -20, 0, 0, 255);
    int bottom_control = map(ypr[1], 0, 20, 0, 255);

    int topleft     = constrain(rest_lift + left_control + top_control    , 0, 255);
    int topright    = constrain(rest_lift + right_control + top_control   , 0, 255);
    int bottomleft  = constrain(rest_lift + left_control + bottom_control , 0, 255);
    int bottomright = constrain(rest_lift + right_control + bottom_control, 0, 255);

		//nsController->Compute();
		//ewController->Compute();
		//yawController->Compute();

    analogWrite(TOPLEFT, topleft); 
    analogWrite(BOTTOMRIGHT, bottomright); 
    analogWrite(TOPRIGHT, topright); 
    analogWrite(BOTTOMLEFT, bottomleft); 

    blinkState = !blinkState;
    digitalWrite(DEBUG_LED, blinkState);
  }
}
