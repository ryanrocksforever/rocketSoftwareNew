

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "Servo.h"
#include <PID_v1.h>
#include <Arduino.h>
#include <SPI.h>              // include libraries
#include <SD.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include <ArduinoJson.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <iostream>
#include <sstream>
#include <cmath>
#include <stdlib.h>     /* srand, rand */
#include <Entropy.h>
//#include <./projectileMotionQuadraticDrag/mainAltiProject.h>
using namespace std;
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high


//Start custom stuff for the tvc. 
//uncomment #define USE_TVC if you want to use

//#define ACTUAL_LAUNCH
#define USE_SERVOS

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

#define MPU6050_ACCEL_FS_2          0x00

uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}
//Servo Setup vars
Servo yawServo; //top servo
Servo pitchServo; // bottom servo
int posYaw = 0;
int posPitch = 0;


int fullOpen = 100; //100 percent open - total degrees of motion from offset

double dragConst = 0.561; // average drag coefficient
double mass = 0.61; // mass of rocket
double g = 9.80665;
const double pi = 3.14159;

float gain = 2.463; //gain got from fusion

//PID Loop setup stuff
double SetpointY, InputY, OutputY;
double SetpointP, InputP, OutputP;
float consKp = 8.6, consKi = 0.7, consKd = 0;
PID upPID(&InputY, &OutputY, &SetpointY, consKp, consKi, consKd, REVERSE);
PID downPID(&InputP, &OutputP, &SetpointP, consKp, consKi, consKd, DIRECT);
int yawOffset = 60; //offset of pos from 90 degrees
int yawMax = 180;
int pitchOffset = 60; //offset of pos from 90 degrees
int pitchMax = 180;


//Altitude projection 
double baseAltitude = 0;

//projectedAlt finalAlt {  };



// Singleton instance of the radio driver
//RH_RF95 rf95(RFM95_CS, RFM95_INT);


//altimeter
Adafruit_BMP3XX bmp;
#define SEALEVELPRESSURE_HPA (1013.25)
//#define USELORA


// Datalogging
//const int chipSelect = BUILTIN_SDCARD; 









// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    delay(1000);
    Entropy.Initialize();
     // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

   
        Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    pinMode(13, OUTPUT); //2 is buzzer pin

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    delay(2000);

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
    

    // supply your own gyro offsets here, scaled for min sensitivity
    // calibrated 3-14-23
    
    mpu.setXGyroOffset(99);
    mpu.setYGyroOffset(67);
    mpu.setZGyroOffset(118);
    mpu.setXAccelOffset(-2911); // 1
    mpu.setYAccelOffset(203); // 
    mpu.setZAccelOffset(1713); // 

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        //mpu.CalibrateAccel(6);
        //mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
        mpu.dmpGetCurrentFIFOPacket(fifoBuffer);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        Serial.println(ypr[2]);
        Serial.println(ypr[0]);
        Serial.println(ypr[1]);

    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

        //altimeter
        if (!bmp.begin_I2C()) {   // hardware I2C mode, can pass in address & alt Wire
        //if (! bmp.begin_SPI(BMP_CS)) {  // hardware SPI mode  
        //if (! bmp.begin_SPI(BMP_CS, BMP_SCK, BMP_MISO, BMP_MOSI)) {  // software SPI mode
            Serial.println("Could not find a valid BMP3 sensor, check wiring!");
            //while (1);
        } else {

            bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
        bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
        bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
        bmp.setOutputDataRate(BMP3_ODR_50_HZ);
        if (! bmp.performReading()) {
            Serial.println("Failed to perform reading :(");
            
        } else {
            Serial.print("Temperature = ");
        Serial.print(bmp.temperature);
        Serial.println(" *C");

        Serial.print("Pressure = ");
        Serial.print(bmp.pressure / 100.0);
        Serial.println(" hPa");

        Serial.print("Approx. Altitude = ");
        Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
        Serial.println(" m");

        Serial.println();
        baseAltitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);



        }
        

        }
        


        
        



        StaticJsonDocument<500> doc;
        
        doc["yawservo"] = 0 ;
        doc["pitchservo"] = 0 ;
        doc["altitude"] = 0;
        doc["tempurature"] = 0;
        doc["message"] = "None" ;
        doc["yawPID"] = 0 ;
        doc["pitchPID"] = 0 ;
        doc["speed"] = 0;
        serializeJson(doc, Serial);

   



    
  

    

    

    
    //config pyro
    

    
    //config servos
    
      
        // initialize device
    




      yawServo.attach(20);
      pitchServo.attach(21);
      yawServo.write(yawOffset);  // 140 is 90
      pitchServo.write(pitchOffset);
      SetpointY = 252;
      SetpointP = 250;
      upPID.SetOutputLimits(0, 100);
      downPID.SetOutputLimits(0, 100);
      upPID.SetMode(AUTOMATIC);
      downPID.SetMode(AUTOMATIC);

    


    Serial.print("\nInitializing SD card...");

    if (!SD.begin(BUILTIN_SDCARD)) {
      Serial.println("initialization failed. Things to check:");
      Serial.println("1. is a card inserted?");
      Serial.println("2. is your wiring correct?");
      Serial.println("3. did you change the chipSelect pin to match your shield or module?");
      Serial.println("Note: press reset button on the board and reopen this Serial Monitor after fixing your issue!");
      while (true);
    }
    
    Serial.println("initialization done.");


    
}






float newaltitude = 0;
float oldaltitude = 0;
double altitude = 0;
float temperature = 0;
double speed = 0;
int lastSendTime = 0;
int interval = 1000;
bool do50hz(DynamicJsonDocument doc) { 
	if (millis() - lastSendTime > 200) {
    
    lastSendTime = millis();            // timestamp the message
    doc["altitude"] = ceil((bmp.readAltitude(SEALEVELPRESSURE_HPA))*1000)/1000;
    doc["tempurature"] = ceil((bmp.temperature)*1000)/1000;
    doc["message"] = "Nominal" ;
    altitude = doc["altitude"];
    temperature = doc["tempurature"];
    if (altitude > 0)
    {
      newaltitude = altitude;
      speed = ((newaltitude-oldaltitude)/200)*1000;
      doc["speed"] = ceil(speed*1000)/1000 ;
      Serial.println("SPeed");
      Serial.println(speed);
      Serial.println(((newaltitude-oldaltitude)));
      oldaltitude = newaltitude;
      if ((speed <-2) && (altitude < 260))
      {
        return false; // IS appogee
      }
      else {
        return true;
      }
    }
  
    }
    else
    {
      return true;
    }
    return true;
  
}


String dataOut = "";
String dataName;


void log2hz(DynamicJsonDocument doc) { 
	if (millis() - lastSendTime > 500) {
    Serial.println("Logging");
    lastSendTime = millis();            // timestamp the message
    File dataFile = SD.open(dataName.c_str(), FILE_WRITE);
    Serial.println("Logging2");
    if (dataFile) {
                serializeJson(doc, dataOut);
                dataFile.println(dataOut);
                dataFile.close();
                // print to the serial port too:
                Serial.println(dataOut);
              } else {
                Serial.println("error opening datalog.txt");
              }
    }
  
}
int lastServoWrite = 0;
void writeServo10hz(int pos) { 
	if (millis() - lastServoWrite > 100) {
      yawServo.write(pos);
      lastServoWrite = millis();
    }
  
}
int lastBeep;
void beep2sec() { 
	if (millis() - lastBeep > 2000 || blinkState == true) {
      blinkState = !blinkState;
      digitalWrite(LED_PIN, blinkState);
      lastBeep = millis();
    }
  
}

int lastSendTimeBurn = 0;
bool waitForBurnout() {
  if (millis() - lastSendTimeBurn > 200) {
    lastSendTimeBurn = millis();
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
                     mpu.dmpGetQuaternion(&q, fifoBuffer);
                    mpu.dmpGetAccel(&aa, fifoBuffer);
                    mpu.dmpGetGravity(&gravity, &q);
                    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
                    mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
                    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
                  if (! bmp.performReading()) {
                    Serial.println("Failed to perform reading :(");
                    
                } else {
                    altitude = ceil((bmp.readAltitude(SEALEVELPRESSURE_HPA))*1000)/1000;
                    if (altitude > 0)
                    {
                      newaltitude = altitude;
                      speed = ((newaltitude-oldaltitude)/200)*1000;
                      //doc["speed"] = ceil(speed*1000)/1000 ;
                      Serial.println("SPeed");
                      Serial.println(speed);
                      Serial.println("Accel");
                      Serial.println(aaReal.z );
                      Serial.println("Altitude");
                      Serial.println(altitude);
                      Serial.println(((newaltitude-oldaltitude)));
                      oldaltitude = newaltitude;
                      
                    }

                }
                 
                  
                  if(aaReal.z < -100 && altitude >(160 +baseAltitude )&& (speed >0.5 || speed < 0.5)) {
                    return true;
                  }

                }
    } else {
      return false;
    }
    
  
  
  return false;
}




int lastSendTime2 = 0;
int counttTen = 0;
bool countTen(DynamicJsonDocument doc) { 
  
	if (millis() - lastSendTime2 > 10000) {
    
    lastSendTime2 = millis();            // timestamp the message
    Serial.println("Countdown 1 sec");
    if(counttTen >= 10 ){
      return true;
    }
    counttTen++;
} else {
  return false;
}
}










  const double dryMass=0.61;//dry mass in Kg
  const double thereGoesGravity=9.8;
  const double crossArea=0.003412;//cross section area in m^2
  double estCoeffDrag = 0.56;
  
  // double currentSpeed = 60;
  // double currentHeight = 114;
double getAirDensity(double groundTemperature, double groundPressure, double currentHeight){ 
 groundTemperature=groundTemperature+273.15;
 const double effectiveConst=0.3;//Constant for estimating the effictive average density for drag
 const double lapseRate=0.00649;
 double Temperature=groundTemperature-(lapseRate*currentHeight);
 double TemperatureApo=groundTemperature-(lapseRate*3048);//calculates air temperature at max height, may be calculated once in main()
 const double gasConst=8.3144598;
 const double airMolarMass=28.7484273;//@2% H2O content
 const double thereGoesGravity=9.8;
 double pressureChangeExp=thereGoesGravity*airMolarMass/(gasConst*currentHeight);
 double pressureChangeExpApo=thereGoesGravity*airMolarMass/(gasConst*3048);//calculates pressure exponant at max height, may be calculated once in main()
 double pressure=groundPressure*pow((1-(lapseRate*currentHeight/groundTemperature)),pressureChangeExp);
 double pressureApo=groundPressure*pow((1-(lapseRate*currentHeight/groundTemperature)),pressureChangeExpApo);//calculates pressure at max height, may be calculated once in main()
 double density=pressure*airMolarMass/(gasConst*Temperature);
 double densityApo=pressureApo*airMolarMass/(gasConst*TemperatureApo);//calculates density at max height, may be calculated once in main()
 double effectiveAveDensity=effectiveConst*(density-densityApo)+densityApo;
 return effectiveAveDensity;
}
  int getMaxHeight(double currentHeight, double currentSpeed) {
    double estAirDensity = getAirDensity(23.3, 135.4, currentHeight);
    
    double terminalVelocitySquared=(2*dryMass*thereGoesGravity/(estCoeffDrag*crossArea*estAirDensity));//simplified math by not taking the square root here
    double maxHeight=terminalVelocitySquared/(2*thereGoesGravity)*log((pow(currentSpeed,2)+terminalVelocitySquared)/terminalVelocitySquared) + currentHeight;
  

    return maxHeight;

  }

int getDescentTime(double currentHeight, double currentSpeed){
  return round(currentHeight/currentSpeed);

}
double descentVel;
double descentTime;
double paraCd = 0.75;
double paraDiam = 0.8636; // m 34 in
double descentMass = 0.61; // kilos
bool shouldOpenChute(double currentHeight, double timeSinceLaunch){
  double estAirDensity = getAirDensity(23.3, 135.4, currentHeight);
  descentVel = sqrt((8*descentMass*g) / (pi*estAirDensity*paraCd*paraDiam));
  Serial.println("descentVel");
  Serial.println(descentVel);
  descentTime = getDescentTime(abs(currentHeight-baseAltitude), descentVel);
  Serial.println("descentTime");
  Serial.println(descentTime);
  if ((descentTime+timeSinceLaunch) > 40 && (descentTime+timeSinceLaunch) <48){
    return true;
  } else{

  if((descentTime+timeSinceLaunch) <=40){
    return true;
  }

  }
   
  
  return false;
}


double getTimeSinceLaunch(double timeOfLaunch){

  return millis() - timeOfLaunch;
}






double projectedApogee ;
float lastAngle = 0;
double angle;
int timeOfLaunch;
int filewritetime;
void launchtime() {
  
  

  //digitalWrite(PYRO_PINL, HIGH);
  //delay(1000);
  //digitalWrite(PYRO_PINL, LOW);
  pitchServo.write(80);
  bool notappoge = true;
  timeOfLaunch = millis();
  //dataName = "datalog5InFlight.txt";
  while(notappoge == true)
  {
    StaticJsonDocument<500> doc;
    //Serial.print("start loop 2-2");


                if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
                     mpu.dmpGetQuaternion(&q, fifoBuffer);
                    mpu.dmpGetAccel(&aa, fifoBuffer);
                    mpu.dmpGetGravity(&gravity, &q);
                    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
                    mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
                    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

                    // angle = arccosine(w) * 2
                    angle = acos(q.w) * 2;
                    lastAngle = angle;
                  

                } else {
                  angle = lastAngle;
                }
                 if (! bmp.performReading()) {
                    Serial.println("Failed to perform reading :(");
                    
                } else {
                    doc["altitude"], altitude = ceil((bmp.readAltitude(SEALEVELPRESSURE_HPA))*1000)/1000;
                    doc["tempurature"] = ceil((bmp.temperature)*1000)/1000;
                    doc["message"] = "In Flight" ;
                    doc["speed"], speed = ceil(speed*1000)/1000 ;

                }



               
            
                
                InputY = getMaxHeight(abs(altitude-baseAltitude), speed);
                Serial.print("ProjectedApogee: ");
                Serial.println(InputY);
                
                upPID.Compute();
                if (projectedApogee < 100 || projectedApogee > 400)
                {
                  Serial.println("ProjectedApogee Failed");
                  Serial.println(projectedApogee);
                  //projectedApogee = 250;
                  
                }
                
            
                //yawPID.Compute();
                //pitchPID.Compute();
                #ifdef USE_SERVOS
                
                
                int yservopos = round(yawMax - ((yawMax- yawOffset)*(OutputY/100.0)));
                if ((yservopos < (50)) | (yservopos >(180)))
                {
                    Serial.print("Yaw out of range!!");
                } else
                {
                    writeServo10hz(yservopos);
                    //pitchServo.write(yservopos);
                }
                
                
                
              #endif
                
                //Serial.print("pitch"); // limits 81 left and 109 right
                //Serial.print(pservopos);
                //Serial.print("\n");
               
                doc["yawservo"] = ceil(yservopos*1000)/1000 ;
                doc["pitchservo"] = ceil(yservopos*1000)/1000 ;
                doc["yawPID"] = ceil(OutputY*1000)/1000 ;
                //doc["pitchPID"] = ceil(OutputP*1000)/1000 ;
                doc["altitude"] = altitude;
                doc["tempurature"] = temperature;
                doc["speed"] = ceil(speed*1000)/1000 ;
                doc["message"] = "in flight";
                doc["projectedAltitude"] = projectedApogee;
                //String loraoutput = "fff";
                serializeJson(doc, Serial);
                log2hz(doc);
              //   if ((millis() - filewritetime) > 500 ) {
              //     Serial.println("Checked Time2");
              //     filewritetime = millis();
              //     serializeJson(doc, dataOut);
              //     serializeJson(doc, Serial);
              //     // while(SD.exists(dataName)) {
              //     //   dataName = (((char)dataCount) +  'datalog.txt');
              //     //   dataCount ++;
              //     //   Serial.println(dataName);
              //     // }
              //     //Serial.println(dataName);
              //     File dataFile = SD.open(dataName.c_str(), FILE_WRITE);
              //     Serial.println("Checked Time3");
              //     // if the file is available, write to it:
              //     if (dataFile) {
              //       dataFile.println(dataOut + "\n");
              //       dataFile.close();
              //       // print to the serial port too:
              //       Serial.println(dataOut);
              //     } else {
              //       Serial.println("error opening datalog.txt");
              //     }
              // }
                //serializeJson(doc, loraoutput);
                //sendData("Helloooo");
                
                notappoge = do50hz(doc);
                delay(10);

  }
  //appogee hit, deploy chute
  
  while (true)
  {
    StaticJsonDocument<500> doc;
            //get data
            
                if (!bmp.performReading()) {
                    Serial.println("Failed to perform reading :(");
                    
                } else {
                    doc["altitude"], altitude = ceil((bmp.readAltitude(SEALEVELPRESSURE_HPA))*1000)/1000;
                    doc["tempurature"] = ceil((bmp.temperature)*1000)/1000;
                    
                    doc["tempurature"] = bmp.temperature;
                    doc["message"] = "Descending" ;
                    do50hz(doc);
                    doc["speed"] = ceil(speed*1000)/1000;

                }

                if ((millis() - filewritetime) > 500 ) {
                  Serial.println("Checked Time2");
                  filewritetime = millis();
                  
                  if(shouldOpenChute(altitude, millis()-timeOfLaunch)){
                    pitchServo.write(0);
                    doc["message"] = "Second Chute Deployed" ;
                  } else {
                    pitchServo.write(80);
                  }
              }
                // Serial.print("ypr\t");
                // Serial.print(ypr[0] * 180/M_PI);
                // Serial.print("\t");
                // Serial.print(ypr[1] * 180/M_PI);
                // Serial.print("\t");
                // Serial.print(ypr[2] * 180/M_PI);

                //Serial.print("ypr\t");
                
               
                //String preoutput = "";
                serializeJson(doc, Serial);
                log2hz(doc);

                
                
            
            
          

            delay(100);
  }
  
}
int findClosest(double arr[], double n, double target)
{
    int left = 0, right = n - 1;
    while (left < right) {
        if (abs(arr[left] - target)
            <= abs(arr[right] - target)) {
            right--;
        }
        else {
            left++;
        }
    }
    return left;
}

double getAcceleration(double zAccel, double angle) {//event is a struct of type sensors_event_t
	
	double theta = angle;
	//using spherical coordinates
	double a = cos(theta)*zAccel;
	return a;
}

/*
This function uses the average of two accelerometer readings multiplied by
the elapsed time, summed with the previous velocity reading.
Thus it integrates over the acceleration to find velocity.

velocity is a global variable that is initialized to 0 on the pad
*/
// double getVelocity(void) {
// 	time_t start,end;
// 	double acc_first,acc_second;
// 	start = time(NULL); //get begin time
// 	acc_first = getAcceleration();//get first acceleration
// 	delay(100);//delay for 100ms
// 	acc_second = getAcceleration();//get acceleration again
// 	end = time(NULL);//get finish time
// 	velocity=velocity + (end-start)*(acc_first+acc_second)/(2);//Vo + at
// 	return velocity;
// }

// /*
// Using spherical coordinates, get angle of rocket to vertical
// event is a structure of type sensors_event_t which holds information from the
// gyro sensor used to derive the angles
// */
// double getAngleToVertical(double angle){
// 	// hardware read
// 	sensors_event_t event;//orient event
// 	bno.getEvent(&event, Adafruit_BNO055::VECTOR_GYROSCOPE);//get orientation
// 	double x=event.orientation.x*180/pi;//get angle to x in radians, convert to deg
// 	double y=event.orientation.y*180/pi;//get angle to y in radians, convert to deg
	
// 	// conversion to spherical
// 	double theta=atan(y/x);//rectangular to spherical conversion
// 	return theta;
// }

double getDrag(double velocity, double a) {
	//double a = getAcceleration();
	// from mg+cv^2=a, we rearrange to solve for c
	double c = (a-mass*g)/(velocity*velocity); 
	return c;
}

double referenceArea = 0.35;
double airDensity = 1.2;  // air density
double getProjectedAltitude(double Cd,double v,double alt) {
  double y = (mass/Cd)*log(cos(atan(v/sqrt(mass*g/Cd)))); // Calculate additional altitude projected
  double deltaHeight = (v * v) / (2 * g) * (mass / (Cd * referenceArea)) +
                        (v / g) * log(mass / (mass - (airDensity * referenceArea * Cd * v * v) / (2 * g)));
  
  double ypro = deltaHeight + alt;  // Projected altitude is the additional + current alt
  return ypro;
}




















void demoLaunch() {
  //demolaunch with preset values
  Serial.println("DEMO LAUNCh");

  //digitalWrite(PYRO_PINL, HIGH);
  //delay(1000);
  //digitalWrite(PYRO_PINL, LOW);
  double angles[] = {80.267, 80.319, 80.382, 80.345, 80.179, 79.972, 79.795, 79.754, 79.818, 79.871, 79.828, 79.635, 79.367, 79.104, 78.858, 78.672, 78.57, 78.567, 78.655, 78.776, 78.845, 78.773, 78.537, 78.174, 77.775, 77.448, 77.254, 77.159, 77.113, 77.115, 77.147, 77.15, 77.08, 76.892, 76.551, 76.117, 75.712, 75.408, 75.227, 75.166, 75.204, 75.282, 75.323, 75.262, 75.062, 74.712, 74.246, 73.701, 73.125, 72.587, 72.137, 71.789, 71.538, 71.397, 71.359, 71.367, 71.352, 71.27, 71.104, 70.849, 70.517, 70.137, 69.714, 69.221, 68.663, 68.079, 67.484, 66.88, 66.274, 65.665, 65.054, 64.437, 63.802, 63.153, 62.483, 61.778, 61.041, 60.251, 59.407, 58.519, 57.58, 56.585, 55.532, 54.42, 53.251, 52.025, 50.747, 49.425, 48.076, 46.712, 45.319, 43.879, 42.386, 40.832, 39.212, 37.516, 35.737, 33.868, 31.905, 29.849, 27.693, 25.43, 23.058, 20.584, 18.011};
  double altitudes[] = {114.416, 115.81, 117.883, 120.862, 123.8, 126.697, 129.553, 132.369, 135.146, 137.883, 140.581, 143.242, 145.864, 148.449, 150.996, 153.507, 155.981, 158.419, 160.82, 163.187, 165.518, 167.814, 170.077, 172.305, 174.499, 176.659, 178.786, 180.88, 182.941, 184.969, 186.964, 188.928, 190.86, 192.761, 194.63, 196.468, 198.275, 200.052, 201.798, 203.513, 205.198, 206.853, 208.478, 210.074, 211.641, 213.179, 214.688, 216.168, 217.62, 219.043, 220.438, 221.804, 223.142, 224.451, 225.733, 226.987, 228.214, 229.413, 230.585, 231.73, 232.848, 233.94, 235.004, 236.043, 237.054, 238.039, 238.998, 239.931, 240.837, 241.718, 242.572, 243.401, 244.203, 244.98, 245.731, 246.457, 247.157, 247.832, 248.482, 249.106, 249.706, 250.28, 250.829, 251.354, 251.853, 252.328, 252.778, 253.204, 253.604, 253.98, 254.332, 254.659, 254.961, 255.239, 255.493, 255.723, 255.929, 256.112, 256.27, 256.405, 256.517, 256.605, 256.67, 256.713, 256.732};
  double speeds[] = {60.965, 60.576, 59.998, 59.167, 58.345, 57.53, 56.723, 55.923, 55.134, 54.355, 53.586, 52.826, 52.07, 51.322, 50.58, 49.845, 49.116, 48.394, 47.679, 46.973, 46.276, 45.586, 44.902, 44.222, 43.546, 42.872, 42.204, 41.543, 40.888, 40.237, 39.593, 38.956, 38.324, 37.697, 37.076, 36.456, 35.837, 35.222, 34.61, 34.002, 33.4, 32.803, 32.213, 31.628, 31.047, 30.469, 29.893, 29.32, 28.747, 28.176, 27.606, 27.04, 26.475, 25.914, 25.356, 24.804, 24.256, 23.713, 23.172, 22.633, 22.096, 21.56, 21.028, 20.498, 19.969, 19.441, 18.915, 18.39, 17.868, 17.347, 16.827, 16.31, 15.795, 15.281, 14.769, 14.259, 13.751, 13.246, 12.741, 12.238, 11.736, 11.236, 10.738, 10.24, 9.744, 9.249, 8.755, 8.261, 7.767, 7.273, 6.783, 6.294, 5.807, 5.323, 4.84, 4.36, 3.882, 3.407, 2.934, 2.464, 1.998, 1.535, 1.075, 0.616, 0.161};
  double accels[] = {1, 1.27, 1.536, 1.482, 1.251, 1.077, 0.805, 0.835, 1.086, 1.257, 1.368, 1.194, 0.975, 0.954, 0.841, 0.769, 0.712, 0.698, 0.803, 0.972, 1.164, 1.208, 1.116, 0.895, 0.637, 0.473, 0.542, 0.653, 0.634, 0.644, 0.765, 0.853, 0.94, 1.026, 0.887, 0.58, 0.398, 0.337, 0.331, 0.356, 0.481, 0.639, 0.783, 0.852, 0.881, 0.788, 0.691, 0.591, 0.413, 0.294, 0.242, 0.258, 0.2, 0.206, 0.314, 0.473, 0.556, 0.587, 0.577, 0.549, 0.464, 0.439, 0.502, 0.493, 0.387, 0.343, 0.335, 0.314, 0.307, 0.299, 0.292, 0.323, 0.304, 0.308, 0.34, 0.322, 0.364, 0.369, 0.333, 0.343, 0.356, 0.355, 0.353, 0.347, 0.338, 0.33, 0.304, 0.26, 0.226, 0.247, 0.29, 0.301, 0.315, 0.329, 0.349, 0.372, 0.393, 0.403, 0.403, 0.422, 0.436, 0.439, 0.418, 0.396, 0.385};
  //note acceleration is lateral accel ex what the sensor returns
  double angles1[] = {80.267, 80.319, 80.382, 80.345, 80.179};
  double altitudes1[] = {114.416, 115.81, 117.883, 120.862, 123.8};
  double speeds1[] = {60.965, 60.576, 59.998, 59.167, 58.345};
  double accels1[] = {1, 1.27, 1.536, 1.482, 1.251};
  
  double alt, v, Cd;
  // projectedApogee = calcApogee(0, dragConst, 80, 58);
  // Serial.println(projectedApogee);
  // projectedApogee = calcApogee(0, dragConst, 78, 56);
  // Serial.println(projectedApogee);
  bool notappoge = true;
  int index = 0;
  int targetVel;
  int actualVel;
  for(double loopangle : altitudes) {
    StaticJsonDocument<500> doc;
    //Serial.print("start loop 2-2");

                Serial.println(index);
                // if (true) {
                     

                //     // angle = arccosine(w) * 2
                     angle = angles[index];
                //     lastAngle = angle;
                  

                // } else {
                //   angle = lastAngle;
                // }
                 
                    doc["altitude"] = altitudes[index];
                    doc["tempurature"] = 0;
                    doc["message"] = "In Flight DEMO" ;
                    doc["speed"] = speeds[index];

                

                //InputP = (pitch);
                Serial.println("Data");
                Serial.println(altitudes[index]);
                Serial.println(dragConst);
                Serial.println(angle);
                Serial.println(speeds[index]);
                //projectedApogeeList = calcApogeeList(0, dragConst, angle, speeds[index]);
                
                //Serial.println((char*)(projectedApogeeList));
                
                //projectedApogee = calcApogee(0, dragConst, angle, speeds[index]);
                
                


                alt = altitudes[index];//get data from altimeter
                Serial.println(alt);
                v = speeds[index];//get velocity from altimeter do50hz funct
                Serial.println(v);
                Serial.println(getAcceleration(accels[index], angle));
                //Cd = getDrag(v, getAcceleration(accels[index], angle));
                
                //Serial.println(Cd);
                Cd = 0.5;
                //InputY = getProjectedAltitude(Cd, v, alt);//calculate Input to PID
                InputY = getMaxHeight(altitudes[index], speeds[index]);
                Serial.print("ProjectedApogee: ");
                Serial.println(InputY);
                // if (projectedApogee < 100 || projectedApogee > 400)
                // {
                //   Serial.println("ProjectedApogee Failed");
                //   Serial.println(projectedApogee);
                //   //projectedApogee = 250;
                  
                // }
                //SetpointY = targetVel
                //InputY = actualVel;
                upPID.Compute();
                //yawPID.Compute();
                //pitchPID.Compute();
                #ifdef USE_SERVOS
                int yservopos = round(yawMax - ((yawMax- yawOffset)*(OutputY/100.0)));
                if ((yservopos < (50)) | (yservopos >(180)))
                {
                    Serial.print("Yaw out of range!!");
                } else
                {
                  writeServo10hz(yservopos);
                }
                
                //Serial.print("yaw"); // limits 153 front and 100 back
                //Serial.print(yservopos);
                //Serial.print("\n");
                
                int pservopos = round(pitchMax - ((pitchMax- pitchOffset)*(OutputY/100.0)));
                if ((pservopos < (50)) | (pservopos >(180)))
                {
                    Serial.print("Pitch out of range!!");
                } else
                {
                    pitchServo.write(pservopos); 
                }
                
              #endif
               
                doc["yawservo"] = ceil(yservopos*1000)/1000 ;
                doc["pitchservo"] = ceil(pservopos*1000)/1000 ;
                doc["yawPID"] = ceil(OutputY*1000)/1000 ;
                //doc["pitchPID"] = ceil(OutputP*1000)/1000 ;
                doc["altitude"] = altitude;
                doc["tempurature"] = index;
                doc["speed"] = ceil(speed*1000)/1000 ;
                doc["message"] = "in flight";
                doc["projectedAltitude"] = InputY;
                String loraoutput = "fff";
                serializeJson(doc, Serial);
                

                index ++;
                delay(100);
  }
  //appogee hit, deploy chute
  
  

}



class Timer {
public:
    Timer(unsigned long interval) : interval_(interval), lastMillis_(millis()) {}

    bool hasElapsed() {
        unsigned long currentMillis = millis();
        if (currentMillis - lastMillis_ >= interval_) {
            lastMillis_ = currentMillis;
            return true;
        }
        return false;
    }

private:
    unsigned long interval_;
    unsigned long lastMillis_;
};





//String dataOut = "";

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

bool launch = false;
//char* dataName = '00datalog.txt';
int dataCount = 0;
void loop() {
    
    Serial.println(Entropy.random(0, 1));
    dataName = "datalog"+ ((String)(Entropy.random(0, 1000))) + ".txt";
    Serial.println(dataName);
    //demoLaunch();
    //ACTUAL LAUNCH TIME
    
        
        static Timer timer(10000);
        while (launch == false)
        {
          
            StaticJsonDocument<500> doc;
            beep2sec();

            if (! bmp.performReading()) {
                    Serial.println("Failed to perform reading :(");
                    
                } else {
                    doc["altitude"] = bmp.readAltitude(SEALEVELPRESSURE_HPA);
                    doc["tempurature"] = bmp.temperature;
                    doc["message"] = "Ready for Launch" ;
                    
                }
              #ifdef ACTUAL_LAUNCH
                launch=waitForBurnout();
                //launch=timer.hasElapsed();
              #else
                launch=timer.hasElapsed();
                  yawServo.write(60);
              //pitchServo.write(0); // 0 IS OPEN THE PARACHUTE
                  delay(1000); 
                  yawServo.write(180);
                  //pitchServo.write(180);
                  delay(1000);
                  //yawServo.write(60);
              //pitchServo.write(0); // 0 IS OPEN THE PARACHUTE
              delay(1000); 
              yawServo.write(180);
              //pitchServo.write(180);
              delay(1000);
              #endif
              
              Serial.println("Checked Time");
              
              if ((millis() - filewritetime) > 500 || launch == true) {
                Serial.println("Checked Time2");
                filewritetime = millis();
              serializeJson(doc, dataOut);
              serializeJson(doc, Serial);
              // while(SD.exists(dataName)) {
              //   dataName = (((char)dataCount) +  'datalog.txt');
              //   dataCount ++;
              //   Serial.println(dataName);
              // }
              //Serial.println(dataName);
              File dataFile = SD.open(dataName.c_str(), FILE_WRITE);
              Serial.println("Checked Time3");
              // if the file is available, write to it:
              if (dataFile) {
                dataFile.println(dataOut + "\n");
                dataFile.close();
                // print to the serial port too:
                Serial.println(dataOut);
              } else {
                Serial.println("error opening datalog.txt");
              }
              }
             // digitalWrite(13, HIGH);
              delay(100);
              //digitalWrite(13, LOW);
        }
        StaticJsonDocument<500> doc;
        doc["message"] = "START";
        serializeJson(doc, dataOut);
        File dataFile = SD.open(dataName.c_str(), FILE_WRITE);

        // if the file is available, write to it:
        if (dataFile) {
          dataFile.println(dataOut);
          dataFile.close();
          // print to the serial port too:
          Serial.println(dataOut);
        }
        // if the file isn't open, pop up an error:
        else {
          Serial.println("error opening datalog.txt");
        }
        Serial.println("waiting 3");
        #ifdef ACTUAL_LAUNCH
          launchtime();
        #else
          delay(3000);
          demoLaunch();
        #endif
        //launchtime();

        
        
    
  }
  

