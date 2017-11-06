// Arduino sketch that returns calibration offsets for MPU6050 //   Version 1.1  (31th January 2014)
// Done by Luis Ródenas <luisrodenaslorda@gmail.com>
// Based on the I2Cdev library and previous work by Jeff Rowberg <jeff@rowberg.net>
// Updates (of the library) should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib

// These offsets were meant to calibrate MPU6050's internal DMP, but can be also useful for reading sensors. 
// The effect of temperature has not been taken into account so I can't promise that it will work if you 
// calibrate indoors and then use it outdoors. Best is to calibrate and use at the same room temperature.

/* ==========  LICENSE  ==================================
 I2Cdev device library code is placed under the MIT license
 Copyright (c) 2011 Jeff Rowberg
 
 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:
 
 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.
 
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 THE SOFTWARE.
 =========================================================
 */

// I2Cdev and MPU6050 must be installed as libraries
#include <I2Cdev.h>
#include <EEPROMex.h> // include EEPROM library to write the calibrated offset values into the EEPROM
#include <Arduino.h>
#include <DFPlayer.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <OneButton.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include <Wire.h>
#endif

#define MEMORYBASEMPUCALIBOFFSET 200 // start storing the calibration values starting at this address
#define DEBUGVERBOSE
#define DFPLAYER_RX      8
#define DFPLAYER_TX     7
#define SPK1        20 //A6
#define SPK2        21 //A7


#define MAIN_BUTTON     12
#define LOCKUP_BUTTON   4
///////////////////////////////////   CONFIGURATION   /////////////////////////////
//Change this 3 variables if you want to fine tune the skecth to your needs.
int buffersize=1000;     //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
int discardfirstmeas=100;  // Amount of initial measurements to be discarded
int acel_deadzone=10;     //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
int giro_deadzone=10;     //Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)
int accel_offset_divisor=8; //8;
int gyro_offset_divisor=4; //4;
// deadzone: amount of variation between 2 consecutive measurements

// default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high
volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus; // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint8_t fifoBuffer[64]; // FIFO storage buffer
uint16_t mpuFifoCount;     // count of all bytes currently in FIFO
// calibrated acc/gyro values
int16_t ax_zero, ay_zero, az_zero;
int16_t gx_zero, gy_zero, gz_zero;
// orientation/motion vars
Quaternion quaternion;           // [w, x, y, z]         quaternion container
VectorInt16 aaWorld; // [x, y, z]            world-frame accel sensor measurements
static Quaternion quaternion_last;  // [w, x, y, z]         quaternion container
static Quaternion quaternion_reading; // [w, x, y, z]         quaternion container
static VectorInt16 aaWorld_last; // [x, y, z]            world-frame accel sensor measurements
static VectorInt16 aaWorld_reading; // [x, y, z]            world-frame accel sensor measurements

int16_t ax, ay, az,gx, gy, gz;


int mean_ax,mean_ay,mean_az,mean_gx,mean_gy,mean_gz,state=0;
int ax_offset,ay_offset,az_offset,gx_offset,gy_offset,gz_offset;
int ax_initoffset,ay_initoffset,az_initoffset,gx_initoffset,gy_initoffset,gz_initoffset;
int16_t ax_offsetEEPROM,ay_offsetEEPROM,az_offsetEEPROM,gx_offsetEEPROM,gy_offsetEEPROM,gz_offsetEEPROM;
unsigned int calibratedOffsetAdress = 0;
bool forceCalibration = false;
bool CalibResult=false;
bool BlasterBoard=false;
unsigned long sndRepeat = millis();

unsigned int loopcount=0;

/***************************************************************************************************
 * LED String variables
 */
#define LEDSTRING1       3
#define LEDSTRING2      5
#define LEDSTRING3      6
#define LEDSTRING4      9
#define LEDSTRING5      10
#define LEDSTRING6      11
uint8_t ledPins[] = { LEDSTRING1, LEDSTRING2, LEDSTRING3, LEDSTRING4,
LEDSTRING5, LEDSTRING6 };
uint8_t LEDbrightness;
int8_t signSoundFile, signVolume, signBrightness;

/***************************************************************************************************
 * DFPLAYER variables
 */
DFPlayer dfplayer;
uint8_t dfplayer_volume;
uint8_t SoundFile;

///////////////////////////////////   SETUP   ////////////////////////////////////
void setup() {



  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz). Comment this line if having compilation difficulties with TWBR.
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
#endif

// Serial line for debug
Serial.begin(115200);

  /***** DF PLAYER INITIALISATION  *****/
  dfplayer.setSerial(DFPLAYER_TX, DFPLAYER_RX);
  dfplayer.setVolume(15);
  delay(500);
  pinMode(SPK1, INPUT);
  pinMode(SPK2, INPUT);

  //setup finished. Boot ready. We notify !
  dfplayer.playPhysicalTrack(1);
  delay(200);
  // initialise global loop variables
  LEDbrightness=0;
  SoundFile=1;
  signBrightness=1;
  signSoundFile=1;
  signVolume=1;

  BlasterBoard=false; // assume a Stardust/Prime board with MPU
  // initialize device
  mpu.initialize();

  if (mpu.testConnection() ) {
  /*  // wait for ready
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available()){
      Serial.println(F("Send any character to start sketch.\n"));
      delay(1500);
    }                
    while (Serial.available() && Serial.read()); // empty buffer again
  
    // start message
    Serial.println("\nMPU6050 Calibration Sketch");
    delay(2000);
    Serial.println("\nYour MPU6050 should be placed in horizontal position, with package letters facing up. \nDon't touch it until you see a finish message.\n");
    delay(3000);
    // verify connection
    Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
    delay(1000);
    */
    // reset offsets
    ax_initoffset=0;
    ay_initoffset=0;
    az_initoffset=0;
    gx_initoffset=0;
    gy_initoffset=0;
    gz_initoffset=0;
  
    mpu.setXAccelOffset(ax_initoffset);
    mpu.setYAccelOffset(ay_initoffset);
    mpu.setZAccelOffset(az_initoffset);
    mpu.setXGyroOffset(gx_initoffset);
    mpu.setYGyroOffset(gy_initoffset);
    mpu.setZGyroOffset(gz_initoffset);
    
            // set the fll scale range of the gyro- and accelerometer respectively
          mpu.setFullScaleGyroRange(0); //0: 250deg/s | 1: 500deg/s | 2: 1000deg/s | 3: 2000deg/s
          mpu.setFullScaleAccelRange(0); //0: 2g | 1: 4g | 2: 8g | 3: 16g
          
    // Get config from EEPROM if there is one
    // or initialise value with default ones set in StoreStruct
    EEPROM.setMemPool(MEMORYBASEMPUCALIBOFFSET, EEPROMSizeATmega328); //Set memorypool base, assume Arduino Uno board
  }
  else {
    BlasterBoard=true;
  }
}

///////////////////////////////////   LOOP   ////////////////////////////////////
void loop() {
  
// variable declaration
int16_t mpu_caliboffset_AccX, mpu_caliboffset_AccY, mpu_caliboffset_AccZ, mpu_caliboffset_GyroX, mpu_caliboffset_GyroY, mpu_caliboffset_GyroZ;
  
if (!BlasterBoard)  {
  if (state==0){
    // Check if the device is already calibrated (EEPROM filled) and not forceCalibration
    bool execCalib=true;
    int addressInt=MEMORYBASEMPUCALIBOFFSET;
    for (uint8_t i = MEMORYBASEMPUCALIBOFFSET; i < MEMORYBASEMPUCALIBOFFSET+12; (i=i+2)) {
      Serial.print(i);Serial.print(" : EEPROM Data ");Serial.println(EEPROM.readInt(i));
      if ((EEPROM.readInt(i) != 65535 ) and forceCalibration==false) {
        execCalib=false;
      }
    }
    if (execCalib==false) {
      state=3; // jump to test sequence
    }
    else {
      Serial.println("\nReading sensors for first time...");
      meansensors();
      //if (abs(mean_ax)>=32000){ax_initoffset=-mean_ax;Serial.println("\nRemove X-axis deadlock...");}
      //if (abs(mean_ay)>=32000){ay_initoffset=-mean_ay;Serial.println("\nRemove Y-axis deadlock...");}
      //if (mean_az<-32000){az_initoffset=-mean_az;Serial.println("\nRemove Z-axis deadlock...");}
      //if (abs(mean_gx)>=32000){gx_initoffset=-mean_gx;Serial.println("\nRemove Gyro X-axis deadlock...");}
      //if (abs(mean_gy)>=32000){gy_initoffset=-mean_gy;Serial.println("\nRemove Gyro Y-axis deadlock...");}
      //if (mean_gz<-32000){gz_initoffset=-mean_gz;Serial.println("\nRemove Gyro Z-axis deadlock...");}
      state++;
      delay(1000);
    }
  }

  if (state==1) {
    Serial.println("\nCalculating offsets...");
    CalibResult=calibration();
    if (CalibResult) {
      Serial.println("\nCalibration successful!");
    }
    else {
      Serial.println("\nCalibration failed!");
    }
    state++;
    delay(1000);
  }

  if (state==2) {
    meansensors();
    ax_offsetEEPROM=ax_offset;
    ay_offsetEEPROM=ay_offset;
    az_offsetEEPROM=az_offset;
    gx_offsetEEPROM=gx_offset;
    gy_offsetEEPROM=gy_offset;
    gz_offsetEEPROM=gz_offset;
    
    Serial.println("\nFINISHED!");
    Serial.print("\nSensor readings with offsets:\t");
    Serial.print(mean_ax); 
    Serial.print("\t");
    Serial.print(mean_ay); 
    Serial.print("\t");
    Serial.print(mean_az); 
    Serial.print("\t");
    Serial.print(mean_gx); 
    Serial.print("\t");
    Serial.print(mean_gy); 
    Serial.print("\t");
    Serial.println(mean_gz);
    Serial.print("Your offsets:\t");
    Serial.print(ax_offset+ax_initoffset); 
    Serial.print("\t");
    Serial.print(ay_offset+ay_initoffset); 
    Serial.print("\t");
    Serial.print(az_offset+az_initoffset); 
    Serial.print("\t");
    Serial.print(gx_offset+gx_initoffset); 
    Serial.print("\t");
    Serial.print(gy_offset+gy_initoffset); 
    Serial.print("\t");
    Serial.println(gz_offset+gz_initoffset); 
    Serial.println("\nData is printed as: acelX acelY acelZ giroX giroY giroZ");
    Serial.println("Check that your sensor readings are close to 0 0 16384 0 0 0");
    
        // wait for ready
      while (Serial.available() && Serial.read()); // empty buffer
      if (CalibResult==false) {
        while (!Serial.available()){
          Serial.println(F("If you want to store the calibrated offset values into EEPROM, press \"y\" \n"));
          delay(1500);
        }
        int inByte=0; 
        if (Serial.available() > 0) {
        //while (Serial.available() && Serial.read()); // empty buffer again
          inByte = Serial.read();
          #ifdef DEBUGVERBOSE
            Serial.print("Pressed key: ");Serial.println(inByte);
          #endif
        }
        if (inByte==121) { // "y"
          CalibResult=true;
        }
        else {
           Serial.println("Exiting without storing calibrated offset values into EEPROM");
           state=10;  // do nothing
        }
      }
    
      if (CalibResult==true) {
    
            int addressInt = MEMORYBASEMPUCALIBOFFSET;
            int output;
            addressInt       = EEPROM.getAddress(sizeof(int));
            EEPROM.updateInt(addressInt,ax_offsetEEPROM+ax_initoffset);
            #ifdef DEBUGVERBOSE
              output = EEPROM.readInt(addressInt);
              Serial.print("address: ");Serial.println(addressInt);Serial.print("output: ");Serial.println(output);Serial.println("");
            #endif
            // 2
            addressInt       = EEPROM.getAddress(sizeof(int));
            EEPROM.updateInt(addressInt,ay_offsetEEPROM+ay_initoffset);
            #ifdef DEBUGVERBOSE
              output = EEPROM.readInt(addressInt);
              Serial.print("address: ");Serial.println(addressInt);Serial.print("output: ");Serial.println(output);Serial.println("");
            #endif
            // 3
            addressInt       = EEPROM.getAddress(sizeof(int));
            EEPROM.updateInt(addressInt,az_offsetEEPROM+ax_initoffset);
            #ifdef DEBUGVERBOSE
              output = EEPROM.readInt(addressInt);
              Serial.print("address: ");Serial.println(addressInt);Serial.print("output: ");Serial.println(output);Serial.println("");
            #endif
           // 4
            addressInt       = EEPROM.getAddress(sizeof(int));
            EEPROM.updateInt(addressInt,gx_offsetEEPROM+gx_initoffset);
            #ifdef DEBUGVERBOSE
              output = EEPROM.readInt(addressInt);
              Serial.print("address: ");Serial.println(addressInt);Serial.print("output: ");Serial.println(output);Serial.println("");
            #endif
            // 5
            addressInt       = EEPROM.getAddress(sizeof(int));
            EEPROM.updateInt(addressInt,gy_offsetEEPROM+gy_initoffset);
            #ifdef DEBUGVERBOSE
              output = EEPROM.readInt(addressInt);
              Serial.print("address: ");Serial.println(addressInt);Serial.print("output: ");Serial.println(output);Serial.println("");
            #endif
           // 6
            addressInt       = EEPROM.getAddress(sizeof(int));
            EEPROM.updateInt(addressInt,gz_offsetEEPROM+gz_initoffset);
            #ifdef DEBUGVERBOSE
              output = EEPROM.readInt(addressInt);
              Serial.print("address: ");Serial.println(addressInt);Serial.print("output: ");Serial.println(output);Serial.println("");
            #endif
            Serial.println("Calibrated offset values stored in EEPROM!");
            state++;
      }
  }
      
 if (state==3) {  //  start of the test routine
          // retreive MPU6050 calibrated offset values from EEPROM
         EEPROM.setMemPool(MEMORYBASEMPUCALIBOFFSET, EEPROMSizeATmega328);
         int addressInt=MEMORYBASEMPUCALIBOFFSET;
         mpu_caliboffset_AccX=EEPROM.readInt(addressInt);
         mpu.setXAccelOffset(mpu_caliboffset_AccX);
         Serial.print("EEPROM address: ");Serial.println(addressInt);Serial.print("Calibrated AcceleroX Offset: ");Serial.println(mpu_caliboffset_AccX);Serial.println("");
         
         addressInt = addressInt + 2; //EEPROM.getAddress(sizeof(int));        
         mpu_caliboffset_AccY=EEPROM.readInt(addressInt);
         mpu.setYAccelOffset(mpu_caliboffset_AccY);
         Serial.print("EEPROM address: ");Serial.println(addressInt);Serial.print("Calibrated AcceleroY Offset: ");Serial.println(mpu_caliboffset_AccY);Serial.println("");
         
         addressInt = addressInt + 2; //EEPROM.getAddress(sizeof(int));
         mpu_caliboffset_AccZ=EEPROM.readInt(addressInt);
         mpu.setZAccelOffset(mpu_caliboffset_AccZ);
         Serial.print("EEPROM address: ");Serial.println(addressInt);Serial.print("Calibrated AcceleroZ Offset: ");Serial.println(mpu_caliboffset_AccZ);Serial.println("");
         
         addressInt = addressInt + 2; //EEPROM.getAddress(sizeof(int));
         mpu_caliboffset_GyroX=EEPROM.readInt(addressInt);
         mpu.setXGyroOffset(mpu_caliboffset_GyroX);
         Serial.print("EEPROM address: ");Serial.println(addressInt);Serial.print("Calibrated GyroX Offset: ");Serial.println(mpu_caliboffset_GyroX);Serial.println("");
         
         addressInt = addressInt + 2; //EEPROM.getAddress(sizeof(int)); 
         mpu_caliboffset_GyroY=EEPROM.readInt(addressInt);
         mpu.setYGyroOffset(mpu_caliboffset_GyroY);
         Serial.print("EEPROM address: ");Serial.println(addressInt);Serial.print("Calibrated GyroY Offset: ");Serial.println(mpu_caliboffset_GyroY);Serial.println("");

         addressInt = addressInt + 2; //EEPROM.getAddress(sizeof(int));
         mpu_caliboffset_GyroZ=EEPROM.readInt(addressInt);
         mpu.setZGyroOffset(mpu_caliboffset_GyroZ);
         Serial.print("EEPROM address: ");Serial.println(addressInt);Serial.print("Calibrated GyroZ Offset: ");Serial.println(mpu_caliboffset_GyroZ);Serial.println("");

          dfplayer.playPhysicalTrack(1);
        
          if (devStatus == 0) {
            // turn on the DMP, now that it's ready
            Serial.println(F("Enabling DMP..."));
            mpu.setDMPEnabled(true);
        
            // enable Arduino interrupt detection
            Serial.println(
                F(
                    "Enabling interrupt detection (Arduino external interrupt 0)..."));
        //    attachInterrupt(0, dmpDataReady, RISING);
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
        
        
                // set the fll scale range of the gyro- and accelerometer respectively
                mpu.setFullScaleGyroRange(0); //0: 250deg/s | 1: 500deg/s | 2: 1000deg/s | 3: 2000deg/s
                mpu.setFullScaleAccelRange(0); //0: 2g | 1: 4g | 2: 8g | 3: 16g
        
          // configure the motion interrupt for clash recognition
          // INT_PIN_CFG register
          // in the working code of MPU6050_DMP all bits of the INT_PIN_CFG are false (0)
        
          mpu.setInterruptMode(false); // INT_PIN_CFG register INT_LEVEL (0-active high, 1-active low)
          mpu.setInterruptDrive(false); // INT_PIN_CFG register INT_OPEN (0-push/pull, 1-open drain)
          mpu.setInterruptLatch(false); // INT_PIN_CFG register LATCH_INT_EN (0 - emits 50us pulse upon trigger, 1-pin is held until int is cleared)
          mpu.setInterruptLatchClear(false); // INT_PIN_CFG register INT_RD_CLEAR (0-clear int only on reading int status reg, 1-any read clears int)
          mpu.setFSyncInterruptLevel(false);
          mpu.setFSyncInterruptEnabled(false);
          mpu.setI2CBypassEnabled(false);
          // Enable/disable interrupt sources - enable only motion interrupt
          mpu.setIntFreefallEnabled(false);
          mpu.setIntMotionEnabled(true);
          mpu.setIntZeroMotionEnabled(false);
          mpu.setIntFIFOBufferOverflowEnabled(false);
          mpu.setIntI2CMasterEnabled(false);
          mpu.setIntDataReadyEnabled(false);
          mpu.setIntMotionEnabled(true); // INT_ENABLE register enable interrupt source  motion detection
          mpu.setMotionDetectionThreshold(10); // 1mg/LSB
          mpu.setMotionDetectionDuration(2); // number of consecutive samples above threshold to trigger int
          mpuIntStatus = mpu.getIntStatus();
          /***** MP6050 MOTION DETECTOR INITIALISATION  *****/ 
        
          /***** LED SEGMENT INITIALISATION  *****/
          // initialize ledstrings segments
          DDRD |= B01101000;
          DDRB |= B00001110;
        
          //We shut off all pins that could wearing leds,just to be sure
          PORTD &= B10010111;
          PORTB &= B11110001;    

          state++;
 }
}// if board with MPU
else{
  state=4;
}
 if (state==4) { // execute test routine
   if (millis() - sndRepeat > 3000) { // repeat first sound file every 3 secs
      dfplayer.playPhysicalTrack(1);
      sndRepeat=millis();
   }
  if (LEDbrightness==50) {signBrightness=-1;}
  if (LEDbrightness==0) {signBrightness=1;}
  if (dfplayer_volume==31) {signVolume=-1;}
  if (dfplayer_volume==0) {signVolume=1;}
  if (SoundFile==30) {signSoundFile=-1;}
  if (SoundFile==0) {signSoundFile=1;}

  if (!BlasterBoard) {
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
             // display tab-separated accel/gyro x/y/z values
            Serial.print("Acceleration- and Gyro values:\t");
            Serial.print(ax); Serial.print("\t");
            Serial.print(ay); Serial.print("\t");
            Serial.print(az); Serial.print("\t");
            Serial.print(gx); Serial.print("\t");
            Serial.print(gy); Serial.print("\t");
            Serial.println(gz);
  }          
  for (int i = 0; i < sizeof(ledPins); i++) {
    analogWrite(ledPins[i], constrain(LEDbrightness,0,50));
    //delay(5);
  }  
 if (LEDbrightness==50) {

  //delay(20);
 }
 
  LEDbrightness=LEDbrightness+signBrightness;
  //dfplayer_volume=dfplayer_volume+signVolume;
  //SoundFile=SoundFile+signSoundFile;
   
 if (!BlasterBoard) {
 
  mpuIntStatus = mpu.getIntStatus();
    if (mpuIntStatus > 60 and mpuIntStatus < 70) {
      /*
       * THIS IS A CLASH  !
       */
                  Serial.println("*");Serial.println("*");Serial.println("*");Serial.println("*");Serial.println("*");
                  Serial.println("*");Serial.println("*");Serial.println("*");Serial.println("*");Serial.println("*");
                  Serial.println("                       CLASH!                                    ");
                  Serial.println("*");Serial.println("*");Serial.println("*");Serial.println("*");Serial.println("*");
                  Serial.println("*");Serial.println("*");Serial.println("*");Serial.println("*");Serial.println("*");



                } 

 }
 }
}

///////////////////////////////////   FUNCTIONS   ////////////////////////////////////
void meansensors(){
  long i=0,buff_ax=0,buff_ay=0,buff_az=0,buff_gx=0,buff_gy=0,buff_gz=0;
  
  while (i<(buffersize+discardfirstmeas+1)){
    // read raw accel/gyro measurements from device
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    if (i>discardfirstmeas && i<=(buffersize+discardfirstmeas)){ //First 100 measures are discarded
      buff_ax=buff_ax+ax;
      buff_ay=buff_ay+ay;
      buff_az=buff_az+az;
      buff_gx=buff_gx+gx;
      buff_gy=buff_gy+gy;
      buff_gz=buff_gz+gz;
    }
    if (i==(buffersize+100)){
      mean_ax=buff_ax/buffersize;
      mean_ay=buff_ay/buffersize;
      mean_az=buff_az/buffersize;
      mean_gx=buff_gx/buffersize;
      mean_gy=buff_gy/buffersize;
      mean_gz=buff_gz/buffersize;
    }
    i++;
    delay(2); //Needed so we don't get repeated measures
  }
  
  Serial.print("Results of measurements a/g:\t");
  Serial.print(mean_ax); Serial.print("\t");
  Serial.print(mean_ay); Serial.print("\t");
  Serial.print(mean_az); Serial.print("\t");
  Serial.print(mean_gx); Serial.print("\t");
  Serial.print(mean_gy); Serial.print("\t");
  Serial.println(mean_gz);
}

bool calibration(){
  ax_offset=-mean_ax/accel_offset_divisor;
  ay_offset=-mean_ay/accel_offset_divisor;
  //az_offset=-mean_az/accel_offset_divisor;
  az_offset=(16384-mean_az)/8;

  gx_offset=-mean_gx/gyro_offset_divisor;
  gy_offset=-mean_gy/gyro_offset_divisor;
  gz_offset=-mean_gz/gyro_offset_divisor;
  while (1){
    int ready=0;
    mpu.setXAccelOffset(ax_offset+ax_initoffset);
    mpu.setYAccelOffset(ay_offset+ay_initoffset);
    mpu.setZAccelOffset(az_offset+az_initoffset);

    mpu.setXGyroOffset(gx_offset+gx_initoffset);
    mpu.setYGyroOffset(gy_offset+gy_initoffset);
    mpu.setZGyroOffset(gz_offset+gz_initoffset);

    meansensors();
    Serial.println("...");

    if (abs(mean_ax)<=acel_deadzone) ready++;
    else ax_offset=ax_offset-mean_ax/acel_deadzone;

    if (abs(mean_ay)<=acel_deadzone) ready++;
    else ay_offset=ay_offset-mean_ay/acel_deadzone;

    //if (abs(mean_az)<=acel_deadzone) ready++;
    //else az_offset=az_offset-mean_az/acel_deadzone;

    if (abs(16384-mean_az)<=acel_deadzone) ready++;
    else az_offset=az_offset+(16384-mean_az)/acel_deadzone;

    if (abs(mean_gx)<=giro_deadzone) ready++;
    else gx_offset=gx_offset-mean_gx/(giro_deadzone+1);

    if (abs(mean_gy)<=giro_deadzone) ready++;
    else gy_offset=gy_offset-mean_gy/(giro_deadzone+1);

    if (abs(mean_gz)<=giro_deadzone) ready++;
    else gz_offset=gz_offset-mean_gz/(giro_deadzone+1);

    if (ready==6) {
     return true;   
     break;
    }

    Serial.print("Resulting offset calibration value a/g:\t");
  Serial.print(ax_offset+ax_initoffset); Serial.print("\t");
  Serial.print(ay_offset+ay_initoffset); Serial.print("\t");
  Serial.print(az_offset+az_initoffset); Serial.print("\t");
  Serial.print(gx_offset+gx_initoffset); Serial.print("\t");
  Serial.print(gy_offset+gy_initoffset); Serial.print("\t");
  Serial.println(gz_offset+gz_initoffset);
  loopcount=loopcount+1;
  Serial.print("Loop Cnt: ");Serial.println(loopcount);
  if (loopcount==10) {
     return false;   
     break; // exit the calibration routine if no stable results can be obtained after 10 calibration loops
    }
  }
}
