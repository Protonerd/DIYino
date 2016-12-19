

#include <Arduino.h>
#include <DFPlayer.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <EEPROMex.h>
#include <OneButton.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include <Wire.h>
#endif

#define XYZ

#define MPUCALIBEEPROM
#ifdef MPUCALIBEEPROM
#define MEMORYBASEMPUCALIBOFFSET 96 // start storing the calibration values starting at this address
#endif
#ifndef MPUCALIBEEPROM
#define MPUCALIBMANUAL
#endif
#define DEBUGVERBOSE

#define DFPLAYER_RX			8
#define DFPLAYER_TX			7
#define SPK1				20 //A6
#define SPK2				21 //A7


#define MAIN_BUTTON			13
#define LOCKUP_BUTTON		4
/***************************************************************************************************
 * Motion detection Variables
 */
MPU6050 mpu;
// MPU control/status vars
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

/***************************************************************************************************
 * LED String variables
 */
#define LEDSTRING1 			3
#define LEDSTRING2 			5
#define LEDSTRING3 			6
#define LEDSTRING4 			9
#define LEDSTRING5 			10
#define LEDSTRING6 			11
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

void setup() {
  
// variable declaration
int16_t mpu_caliboffset_AccX, mpu_caliboffset_AccY, mpu_caliboffset_AccZ, mpu_caliboffset_GyroX, mpu_caliboffset_GyroY, mpu_caliboffset_GyroZ;

  
  
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
        dfplayer_volume=25;
	dfplayer.setVolume(dfplayer_volume);
	delay(200);
	pinMode(SPK1, INPUT);
	pinMode(SPK2, INPUT);

	//setup finished. Boot ready. We notify !
	dfplayer.playPhysicalTrack(18);
	delay(20);


               // dump config values stored in EEPROM
                //saveConfig();
                Serial.println("EEPROM Dump (first 256 bytes): ");
                for (uint8_t i = 0; i < 255; i++) {
                  Serial.println(EEPROM.readByte(i));
                }




	/***** MP6050 MOTION DETECTOR INITIALISATION  *****/

	// initialize device
	Serial.println(F("Initializing I2C devices..."));
	mpu.initialize();

	// verify connection
	Serial.println(F("Testing device connections..."));
	Serial.println(
			mpu.testConnection() ?
					F("MPU6050 connection successful") :
					F("MPU6050 connection failed"));

	// load and configure the DMP
	Serial.println(F("Initializing DMP..."));
	devStatus = mpu.dmpInitialize();

#ifdef MPUCALIBEEPROM
  //mpu.setXAccelOffset(0);
  //mpu.setYAccelOffset(0);
  //mpu.setZAccelOffset(20000);
  //mpu.setXGyroOffset(0);
  //mpu.setYGyroOffset(0);
  //mpu.setZGyroOffset(20000);
	/*
	 * Those offsets are specific to each MPU6050 device.
	 * they are found via calibration process.
	 * See this script http://www.i2cdevlib.com/forums/index.php?app=core&module=attach&section=attach&attach_id=27
	 */
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

#endif
#ifdef XYZ
	if (devStatus == 0) {
		// turn on the DMP, now that it's ready
		Serial.println(F("Enabling DMP..."));
		mpu.setDMPEnabled(true);

		// enable Arduino interrupt detection
		Serial.println(
				F(
						"Enabling interrupt detection (Arduino external interrupt 0)..."));
//		attachInterrupt(0, dmpDataReady, RISING);
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
	Serial.println("MPU6050 register setup:");
	Serial.print("INT_PIN_CFG\t");
	Serial.print(mpu.getInterruptMode());
	Serial.print("\t");
	Serial.print(mpu.getInterruptDrive());
	Serial.print("\t");
	Serial.print(mpu.getInterruptLatch());
	Serial.print("\t");
	Serial.print(mpu.getInterruptLatchClear());
	Serial.print("\t");
	Serial.print(mpu.getFSyncInterruptLevel());
	Serial.print("\t");
	Serial.print(mpu.getFSyncInterruptEnabled());
	Serial.print("\t");
	Serial.println(mpu.getI2CBypassEnabled());
	// list INT_ENABLE register contents
	Serial.print("INT_ENABLE\t");
	Serial.print(mpu.getIntFreefallEnabled());
	Serial.print("\t");
	Serial.print(mpu.getIntMotionEnabled());
	Serial.print("\t");
	Serial.print(mpu.getIntZeroMotionEnabled());
	Serial.print("\t");
	Serial.print(mpu.getIntFIFOBufferOverflowEnabled());
	Serial.print("\t");
	Serial.print(mpu.getIntI2CMasterEnabled());
	Serial.print("\t");
	Serial.println(mpu.getIntDataReadyEnabled());
	/***** MP6050 MOTION DETECTOR INITIALISATION  *****/ 

	/***** LED SEGMENT INITIALISATION  *****/
	// initialize ledstrings segments
	DDRD |= B01101000;
	DDRB |= B00001110;

	//We shut off all pins that could wearing leds,just to be sure
	PORTD &= B10010111;
	PORTB &= B11110001;
#endif

  //mpu.setXAccelOffset(0);
  //mpu.setYAccelOffset(0);
  //mpu.setZAccelOffset(32767);
  //mpu.setXGyroOffset(0);
  //mpu.setYGyroOffset(0);
  //mpu.setZGyroOffset(20000);
  
        // initialise global loop variables
        LEDbrightness=0;
        SoundFile=1;
        signBrightness=1;
        signSoundFile=1;
        signVolume=1;
        
        dfplayer.setVolume(dfplayer_volume);
	delay(200);
	dfplayer.playPhysicalTrack(20);
}

void loop() {
  
  uint8_t i;
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  
  if (LEDbrightness==50) {signBrightness=-1;}
  if (LEDbrightness==0) {signBrightness=1;}
  if (dfplayer_volume==31) {signVolume=-1;}
  if (dfplayer_volume==0) {signVolume=1;}
  if (SoundFile==30) {signSoundFile=-1;}
  if (SoundFile==0) {signSoundFile=1;}
  
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
             // display tab-separated accel/gyro x/y/z values
            Serial.print("Acceleration- and Gyro values:\t");
            Serial.print(ax); Serial.print("\t");
            Serial.print(ay); Serial.print("\t");
            Serial.print(az); Serial.print("\t");
            Serial.print(gx); Serial.print("\t");
            Serial.print(gy); Serial.print("\t");
            Serial.println(gz);
            
  for (int i = 0; i < sizeof(ledPins); i++) {
    analogWrite(ledPins[i], constrain(LEDbrightness,0,50));
    delay(20);
  }  
 if (LEDbrightness==50) {

	//delay(20);
 }
 
  LEDbrightness=LEDbrightness+signBrightness;
  dfplayer_volume=dfplayer_volume+signVolume;
  SoundFile=SoundFile+signSoundFile;
  
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
