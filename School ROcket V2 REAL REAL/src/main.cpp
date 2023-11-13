

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "Servo.h"
#include PID_v1.h

#include <SPI.h>              // include libraries
#include <SD.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include <ArduinoJson.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <iostream>
#include <sstream>
#include <cmath>
#include <./projectileMotionQuadraticDrag/mainAltiProject.h>
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
#define USE_TVC
#define USE_TVCLOOP
#define ROCKET_PROCEDURES
#define ACTUAL_LAUNCH


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

#define MPU6050_ACCEL_FS_2          0x00
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}


//Servo Setup vars
Servo yawServo; //top servo
Servo pitchServo; // bottom servo
int posYaw = 0;
int posPitch = 0;


int fullOpen = 100; //100 percent open - offset

flaot dragConst = 0.561; // average drag coefficient

float gain = 2.463; //gain got from fusion

//PID Loop setup stuff
double SetpointY, InputY, OutputY;
double SetpointP, InputP, OutputP;
float consKp = 0.6, consKi = 0.1, consKd = 0.5;
PID upPID(&InputY, &OutputY, &SetpointY, consKp, consKi, consKd, DIRECT);
PID downPID(&InputP, &OutputP, &SetpointP, consKp, consKi, consKd, DIRECT);
int yawOffset = 82; //offset of pos from 90 degrees
int pitchOffset = 106; //offset of pos from 90 degrees



//Altitude projection 


projectedAlt finalAlt {  };



// Singleton instance of the radio driver
//RH_RF95 rf95(RFM95_CS, RFM95_INT);


//altimeter
Adafruit_BMP3XX bmp;
#define SEALEVELPRESSURE_HPA (1013.25)
//#define USELORA












// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    delay(1000);


    #ifdef ROCKET_PROCEDURES
        Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

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

    #endif



    
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    

    

    
    //config pyro
    

    
    //config servos
    #ifdef USE_TVC
      
        // initialize device
    




      yawServo.attach(20);
      pitchServo.attach(21);
      yawServo.write(yawOffset);  // 140 is 90
      pitchServo.write(pitchOffset);
      SetpointY = 250;
      SetpointP = 250;
      upPID.SetOutputLimits(0, 100);
      downPID.SetOutputLimits(0, 100);
      upPID.SetMode(AUTOMATIC);
      downPID.SetMode(AUTOMATIC);

    #endif


    Serial.print("Initializing SD card...");

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
float altitude = 0;
float temperature = 0;
float speed = 0;
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
      if ((speed > (2)))
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



float projectedApogee ;
float lastAngle = 0;
flaot angle;

void launchtime() {
  
  }

  //digitalWrite(PYRO_PINL, HIGH);
  //delay(1000);
  //digitalWrite(PYRO_PINL, LOW);

  bool notappoge = true;

  while (notappoge == true)
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
                    doc["altitude"] = ceil((bmp.readAltitude(SEALEVELPRESSURE_HPA))*1000)/1000;
                    doc["tempurature"] = ceil((bmp.temperature)*1000)/1000;
                    doc["message"] = "In Flight" ;
                    doc["speed"] = ceil(speed*1000)/1000 ;

                }

                //InputP = (pitch);
                projectedApogee = finalAlt(altitude, dragConst, angle, speed);
                InputY = (projectedAlt);
                yawPID.Compute();
                //pitchPID.Compute();
                float yservopos = yawOffset +fullOpen*(OutputY/100.0);
                if ((yservopos < (yawOffset-50)) | (yservopos >(yawOffset+70)))
                {
                    Serial.print("Yaw out of range!!");
                } else
                {
                    yawServo.write(yservopos);
                }
                
                //Serial.print("yaw"); // limits 153 front and 100 back
                //Serial.print(yservopos);
                //Serial.print("\n");
                
                float pservopos = pitchOffset +gain*(OutputP);
                if ((pservopos < (pitchOffset-50)) | (pservopos >(pitchOffset+50)))
                {
                    Serial.print("Pitch out of range!!");
                } else
                {
                    pitchServo.write(pservopos);
                }
                
                //Serial.print("pitch"); // limits 81 left and 109 right
                //Serial.print(pservopos);
                //Serial.print("\n");
               
                doc["yawservo"] = ceil(yservopos*1000)/1000 ;
                doc["pitchservo"] = ceil(pservopos*1000)/1000 ;
                doc["yawPID"] = ceil(OutputY*1000)/1000 ;
                doc["pitchPID"] = ceil(OutputP*1000)/1000 ;
                doc["altitude"] = altitude;
                doc["tempurature"] = temperature;
                doc["speed"] = ceil(speed*1000)/1000 ;
                doc["message"] = "in flight";
                doc["projectedAltitude"] = projectedAlt;
                String loraoutput = "fff";
                serializeJson(doc, Serial);
                //serializeJson(doc, loraoutput);
                //sendData("Helloooo");
                notappoge = do50hz(doc);

  }
  //appogee hit, deploy chute
  
  while (true)
  {
    StaticJsonDocument<500> doc;
            //get data
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
                    doc["altitude"] = ceil((bmp.readAltitude(SEALEVELPRESSURE_HPA))*1000)/1000;
                    doc["tempurature"] = ceil((bmp.temperature)*1000)/1000;
                    doc["altitude"] = bmp.readAltitude(SEALEVELPRESSURE_HPA);
                    doc["tempurature"] = bmp.temperature;
                    doc["message"] = "Descending" ;
                    do50hz(doc);
                    doc["speed"] = ceil(speed*1000)/1000 ;

                }
                // Serial.print("ypr\t");
                // Serial.print(ypr[0] * 180/M_PI);
                // Serial.print("\t");
                // Serial.print(ypr[1] * 180/M_PI);
                // Serial.print("\t");
                // Serial.print(ypr[2] * 180/M_PI);

                //Serial.print("ypr\t");
                float yaw = ypr[2] * 180/M_PI;
                
                float pitch = ypr[1] * 180/M_PI;
                
                float roll = ypr[1] * 180/M_PI;


                doc["yaw"] = ceil(yaw*1000)/1000 ;
                doc["pitch"] = ceil(pitch*1000)/1000;
                doc["roll"] = ceil(roll*1000)/1000;
                doc["xccel"] = aaWorld.x ;
                doc["yaccel"] = aaWorld.y ;
                doc["zaccel"] = aaWorld.z ;
                String preoutput = "";
                serializeJson(doc, preoutput);
                sendData(preoutput);
            }
            
            if (rf95.waitAvailableTimeout(100))
            {
              
                handleMessage();
                
                
                
                
                

                
                
            } else
            {
              Serial.println("No message available");
            }
            

            delay(100);
  }
  
}












String dataOut = "";

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
int filewritetime ;
void loop() {

    //demoLaunch();
    //ACTUAL LAUNCH TIME
    #ifdef ACTUAL_LAUNCH
        
        
        while (launch == false)
        {
            StaticJsonDocument<500> doc;
            if (! bmp.performReading()) {
                    Serial.println("Failed to perform reading :(");
                    
                } else {
                    doc["altitude"] = bmp.readAltitude(SEALEVELPRESSURE_HPA);
                    doc["tempurature"] = bmp.temperature;
                    doc["message"] = "Ready for Launch" ;
                    launch=do50hz();
                }
              if (millis() - filewritetime > 500 || launch = true) {
              serializeJson(doc, dataOut);
              File dataFile = SD.open("datalog.txt", FILE_WRITE);

              // if the file is available, write to it:
              if (dataFile) {
                dataFile.println(dataString);
                dataFile.close();
                // print to the serial port too:
                Serial.println(dataString);
              }
              // if the file isn't open, pop up an error:
              else {
                Serial.println("error opening datalog.txt");
              }
              }
        }
        doc["message"] = "START" ;
        serializeJson(doc, dataOut);
        File dataFile = SD.open("datalog.txt", FILE_WRITE);

        // if the file is available, write to it:
        if (dataFile) {
          dataFile.println(dataString);
          dataFile.close();
          // print to the serial port too:
          Serial.println(dataString);
        }
        // if the file isn't open, pop up an error:
        else {
          Serial.println("error opening datalog.txt");
        }
        delay(3000);
        launchtime();

        
        
    #endif
  }
  

