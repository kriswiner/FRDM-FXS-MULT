/* FXOS8700CQ Example Code
 Original sketch by: Kris Winer
 with pieces borrowed from Jim Linblom of sparkfun.com
 date: May 31, 2014
 license: Beerware - Use this code however you'd like. If you 
 find it useful you can buy me a beer some time.
 
 Includes reset, initialization, accelerometer calibration, sleep mode, motion threshold, portrait/lanscape detection
 and tap detection function. Tried to get the acceleration magnitude detection to work but it seems to be always on!
 Added LCD functions to allow data display and motion detection to on-breadboard monitor.
 
 This code provides example usage for most features of
 the FXOS8700CQ 3-axis, I2C 14-bit accelerometer/16-bit magnetometer. 
 Play around with the settings in the various functions and consult the data sheet.
 
 Hardware setup:
 FXOS8700CQ Breakout ------ Arduino
 3.3V --------------------- 3.3V
 SDA ----------------------- A4
 SCL ----------------------- A5
 INT2 ---------------------- D5 or any digital pin
 INT1 ---------------------- D4 or any other digital pin
 GND ---------------------- GND
 
 SDA and SCL should have external pull-up resistors (to 3.3V).
 10k resistors are on the FRDM-FXS-MULTI breakout board.
 
 Note: The FXOS8700CQ is an I2C sensor; here we make use of the Arduino Wire library.
 Because the sensor is not 5V tolerant, we are using a 3.3 V 8 MHz Pro Mini or a 3.3 V Teensy 3.1.
 We have disabled the internal pull-ups used by the Wire library in the Wire.h/twi.c utility file.
 We are also using the 400 kHz fast I2C mode by setting the TWI_FREQ  to 400000L /twi.h utility file.
 */
 
#include "Wire.h"  
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>

// Using NOKIA 5110 monochrome 84 x 48 pixel display
// pin 9 - Serial clock out (SCLK)
// pin 8 - Serial data out (DIN)
// pin 7 - Data/Command select (D/C)
// pin 5 - LCD chip select (CS)
// pin 6 - LCD reset (RST)
Adafruit_PCD8544 display = Adafruit_PCD8544(9, 8, 7, 5, 6);

// Define registers per FXOS8700CQ, Document Number: FXOS8700CQ
// Data Sheet: Technical Data Rev. 2.0, 02/2013 3-Axis, 12-bit/8-bit Digital Accelerometer
// Freescale Semiconductor Data Sheet
#define STATUS           0x00
#define DR_STATUS        0x00
#define F_STATUS         0x00
#define OUT_X_MSB        0x01    
#define OUT_X_LSB        0x02
#define OUT_Y_MSB        0x03
#define OUT_Y_LSB        0x04
#define OUT_Z_MSB        0x05
#define OUT_Z_LSB        0x06
#define F_SETUP          0x09
#define TRIG_CFG         0x0A
#define SYSMOD           0x0B
#define INT_SOURCE       0x0C
#define WHO_AM_I         0x0D   
#define XYZ_DATA_CFG     0x0E
#define HP_FILTER_CUTOFF 0x0F
#define PL_STATUS        0x10
#define PL_CFG           0x11
#define PL_COUNT         0x12
#define PL_BF_ZCOMP      0x13
#define P_L_THS_REG      0x14
#define A_FFMT_CFG       0x15
#define A_FFMT_SRC       0x16
#define A_FFMT_THS       0x17
#define A_FFMT_COUNT     0x18
#define TRANSIENT_CFG    0x1D
#define TRANSIENT_SRC    0x1E
#define TRANSIENT_THS    0x1F
#define TRANSIENT_COUNT  0x20
#define PULSE_CFG        0x21
#define PULSE_SRC        0x22
#define PULSE_THSX       0x23
#define PULSE_THSY       0x24
#define PULSE_THSZ       0x25
#define PULSE_TMLT       0x26
#define PULSE_LTCY       0x27
#define PULSE_WIND       0x28
#define ASLP_COUNT       0x29
#define CTRL_REG1        0x2A
#define CTRL_REG2        0x2B
#define CTRL_REG3        0x2C
#define CTRL_REG4        0x2D
#define CTRL_REG5        0x2E
#define OFF_X            0x2F
#define OFF_Y            0x30
#define OFF_Z            0x31
#define M_DR_STATUS      0x32
#define M_OUT_X_MSB      0x33    
#define M_OUT_X_LSB      0x34
#define M_OUT_Y_MSB      0x35
#define M_OUT_Y_LSB      0x36
#define M_OUT_Z_MSB      0x37
#define M_OUT_Z_LSB      0x38
#define CMP_OUT_X_MSB    0x39    
#define CMP_OUT_X_LSB    0x3A
#define CMP_OUT_Y_MSB    0x3B
#define CMP_OUT_Y_LSB    0x3C
#define CMP_OUT_Z_MSB    0x3D
#define CMP_OUT_Z_LSB    0x3E
#define M_OFF_X_MSB      0x3F    
#define M_OFF_X_LSB      0x40
#define M_OFF_Y_MSB      0x41
#define M_OFF_Y_LSB      0x42
#define M_OFF_Z_MSB      0x43
#define M_OFF_Z_LSB      0x44
#define MAX_X_MSB        0x45   
#define MAX_X_LSB        0x46
#define MAX_Y_MSB        0x47
#define MAX_Y_LSB        0x48
#define MAX_Z_MSB        0x49
#define MAX_Z_LSB        0x4A
#define MIN_X_MSB        0x4B   
#define MIN_X_LSB        0x4C
#define MIN_Y_MSB        0x4D
#define MIN_Y_LSB        0x4E
#define MIN_Z_MSB        0x4F
#define MIN_Z_LSB        0x50
#define TEMP             0x51
#define M_THS_CFG        0x52
#define M_THS_SRC        0x53
#define M_THS_X_MSB      0x54   
#define M_THS_X_LSB      0x55
#define M_THS_Y_MSB      0x56
#define M_THS_Y_LSB      0x57
#define M_THS_Z_MSB      0x58
#define M_THS_Z_LSB      0x59
#define M_THS_COUNT      0x5A
#define M_CTRL_REG1      0x5B
#define M_CTRL_REG2      0x5C
#define M_CTRL_REG3      0x5D
#define M_INT_SRC        0x5E
#define A_VECM_CFG       0x5F
#define A_VECM_THS_MSB   0x60
#define A_VECM_THS_LSB   0x61
#define A_VECM_CNT       0x62
#define A_VECM_INITX_MSB 0x63   
#define A_VECM_INITX_LSB 0x64
#define A_VECM_INITY_MSB 0x65
#define A_VECM_INITY_LSB 0x66
#define A_VECM_INITZ_MSB 0x67
#define A_VECM_INITZ_LSB 0x68
#define M_VECM_CFG       0x69
#define M_VECM_THS_MSB   0x6A
#define M_VECM_THS_LSB   0x6B
#define M_VECM_CNT       0x6C
#define M_VECM_INITX_MSB 0x6D   
#define M_VECM_INITX_LSB 0x6E
#define M_VECM_INITY_MSB 0x6F
#define M_VECM_INITY_LSB 0x70
#define M_VECM_INITZ_MSB 0x71
#define M_VECM_INITZ_LSB 0x72
#define A_FFMT_THS_X_MSB 0x73
#define A_FFMT_THS_X_LSB 0x74
#define A_FFMT_THS_Y_MSB 0x75
#define A_FFMT_THS_Y_LSB 0x76
#define A_FFMT_THS_Z_MSB 0x77
#define A_FFMT_THS_Z_LSB 0x78

#define SA0 0
#if SA0
#define FXOS8700CQ_ADDRESS 0x1F  // SA0 is high, 0x1E if low
#else
#define FXOS8700CQ_ADDRESS 0x1E
#endif

// Set initial input parameters
enum accelFSR {
  AFS_2g = 0,
  AFS_4g,
  AFS_8g
};

enum accelODR {
  AODR_800HZ = 0, // 200 Hz
  AODR_400HZ,
  AODR_200HZ,
  AODR_100HZ,
  AODR_50HZ,
  AODR_12_5HZ, // 12.5 Hz, etc.
  AODR_6_25HZ,
  AODR_1_56HZ
};

enum magOSR {
  MOSR_0 = 0,  // oversample ratio 2 at 50 and 200 Hz ODR
  MOSR_1,
  MOSR_2,
  MOSR_3,
  MOSR_4,
  MOSR_5,  
  MOSR_6,
  MOSR_7      // oversample ratio 8 at 200 Hz ODR, 32 at 50 HZ ODR
};

// Specify sensor full scale
uint8_t accelFSR = AFS_2g;     // Set the scale below either 2, 4 or 8
uint8_t accelODR = AODR_200HZ; // In hybrid mode, accel/mag data sample rates are half of this value
uint8_t   magOSR = MOSR_5;     // Choose magnetometer oversample rate
float aRes, mRes;              // Scale resolutions per LSB for the sensor

// Pin definitions
int int1Pin = 12;  // These can be changed, 2 and 3 are the Arduinos ext int pins
int ledPin  = 13;  // Pro Mini led

int16_t magCount[3], accelCount[3];  // Stores the 12-bit signed value
float ax, ay, az;       // Stores the real accel value in g's
float mx, my, mz;       // Stores the real mag value in G's
int8_t tempCount;
float temperature;
uint32_t count = 0;
boolean sleepMode = false;

void setup()
{
  Serial.begin(38400);

//  lcd.begin(16, 2);// Initialize the LCD with 16 characters and 2 lines
 
  // Set up the interrupt pins, they're set as active high, push-pull
  pinMode(int1Pin, INPUT);
  digitalWrite(int1Pin, LOW);
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
  
  display.begin(); // Initialize the display
  display.setContrast(58); // Set the contrast
  display.setRotation(2); //  0 or 2) width = width, 1 or 3) width = height, swapped etc.
  
// Start device display with ID of sensor
  display.clearDisplay();
  display.setCursor(0, 0); display.print("FXOS8700CQ");
  display.setCursor(0, 8); display.print("3-axis 14-bit");
  display.setCursor(0, 16); display.print("accelerometer");
  display.setCursor(0, 24); display.print("3-axis 16-bit");
  display.setCursor(0, 32); display.print("magnetometer");
  display.setCursor(0, 40); display.print("0.25 mg, 1 mG ");
  display.display();
  delay(2000);

// Set up for data display
  display.setTextSize(1); // Set text size to normal, 2 is twice normal etc.
  display.setTextColor(BLACK); // Set pixel color; 1 on the monochrome screen
  display.clearDisplay();   // clears the screen and buffer
  
  // Read the WHO_AM_I register, this is a good test of communication
  byte c = readByte(FXOS8700CQ_ADDRESS, WHO_AM_I);  // Read WHO_AM_I register
  display.clearDisplay();
  display.setCursor(0,0); display.print("FXOS8700CQ");  
  display.setCursor(0,10); display.print("I Am");
  display.setCursor(0, 20); display.print("Ox");display.print(c, HEX);  
  display.setCursor(0, 30); display.print("I Should be"); 
  display.setCursor(0, 40); display.print("Ox");display.print(0xC7, HEX);  
  display.display();
  delay(1000);

  if (c == 0xC7) // WHO_AM_I should always be 0xC7
  {  
    FXOS8700CQReset();           // Start by resetting sensor device to default settings
    calibrateFXOS8700CQ();       // Calibrate the accelerometer
    FXOS8700CQMagOffset();       // Apply user-determined magnetometer offsets-currently se to calculate the offsets dynamically
    initFXOS8700CQ();            // Initialize the accelerometer and magnetometer if communication is OK
    accelMotionIntFXOS8700CQ();  // Configure motion interrupts
    sleepModeFXOS8700CQ();       // Configure sleep mode
    Serial.println("FXOS8700CQQ is online...");

    display.clearDisplay();
    display.setCursor(0,0); display.print("FXOS8700CQ");
    display.setCursor(0, 20); display.print("ax bias "); display.print((int8_t)2*readByte(FXOS8700CQ_ADDRESS, OFF_X)); display.print(" mg");
    display.setCursor(0, 30); display.print("ay bias "); display.print((int8_t)2*readByte(FXOS8700CQ_ADDRESS, OFF_Y)); display.print(" mg");
    display.setCursor(0, 40); display.print("az bias "); display.print((int8_t)2*readByte(FXOS8700CQ_ADDRESS, OFF_Z)); display.print(" mg");
    display.display();
    delay(1000);
    display.clearDisplay();
    display.setCursor(0,0); display.print("FXOS8700CQ");
    int16_t mxbias = (int16_t)readByte(FXOS8700CQ_ADDRESS, M_OFF_X_MSB) << 8 | readByte(FXOS8700CQ_ADDRESS, M_OFF_X_LSB);
    int16_t mybias = (int16_t)readByte(FXOS8700CQ_ADDRESS, M_OFF_Y_MSB) << 8 | readByte(FXOS8700CQ_ADDRESS, M_OFF_Y_LSB);
    int16_t mzbias = (int16_t)readByte(FXOS8700CQ_ADDRESS, M_OFF_Z_MSB) << 8 | readByte(FXOS8700CQ_ADDRESS, M_OFF_Z_LSB);
    display.setCursor(0, 20); display.print("mx bias "); display.print((int16_t)(((float)mxbias)*10./32.768)); display.print(" mG");
    display.setCursor(0, 30); display.print("my bias "); display.print((int16_t)(((float)mybias)*10./32.768)); display.print(" mG");
    display.setCursor(0, 40); display.print("mz bias "); display.print((int16_t)(((float)mzbias)*10./32.768)); display.print(" mG");
    display.display();
    delay(1000);
  }
  else
  {
    Serial.print("Could not connect to FXOS8700CQQ: 0x");
    Serial.println(c, HEX);
    while(1) ; // Loop forever if communication doesn't happen
  }
}

void loop()
{  
  // One can use the interrupt pins to detect a data ready condition; here we just check the STATUS register for a data ready bit
  if(readByte(FXOS8700CQ_ADDRESS, DR_STATUS) & 0x08)  // When this bit set, all accel axes have new data
  {
    readAccelData(accelCount);  // Read the x/y/z adc values
    getAres();                  // get accelerometer sensitivity
    ax = (float)accelCount[0]*aRes;  // get actual g value, this depends on scale being set
    ay = (float)accelCount[1]*aRes;  // also subtract averaged accelerometer biases
    az = (float)accelCount[2]*aRes;  
    
    tempCount = readTempData();  // Read the x/y/z adc values
    temperature = (float) tempCount * 0.96 + 4.; // Temperature in degrees Centigrade
  }
  if(readByte(FXOS8700CQ_ADDRESS, M_DR_STATUS) & 0x08)  // When this bit set, all mag axes have new data
  {
    readMagData(magCount);      // Read the x/y/z adc values
    mRes = 10./32768.;
    mx = (float)magCount[0]*mRes;  // get actual Gauss value 
    my = (float)magCount[1]*mRes;   
    mz = (float)magCount[2]*mRes;  
  }
  
   
   uint32_t deltat = millis() - count;
   if (deltat > 500) { // update LCD once per half-second independent of read rate

    // Print out accelerometer data in mgs
    Serial.print("x-acceleration = "); Serial.print(1000.*ax); Serial.print(" mg");   
    Serial.print("y-acceleration = "); Serial.print(1000.*ay); Serial.print(" mg");   
    Serial.print("z-acceleration = "); Serial.print(1000.*az); Serial.print(" mg");

    // Print out magnetometer data in mG
    Serial.print("x-magnetic field = "); Serial.print(1000.*mx); Serial.print(" mG");   
    Serial.print("y-magnetic field = "); Serial.print(1000.*my); Serial.print(" mG");   
    Serial.print("z-magnetic field = "); Serial.print(1000.*mz); Serial.print(" mG");  
 
    display.clearDisplay();
     
    display.setCursor(0, 0); display.print("FXOS8700CQ");
    display.setCursor(0, 8); display.print(" x   y   z  ");

    display.setCursor(0,  16); display.print((int)(1000*ax)); 
    display.setCursor(24, 16); display.print((int)(1000*ay)); 
    display.setCursor(48, 16); display.print((int)(1000*az)); 
    display.setCursor(72, 16); display.print("mg");
    
    display.setCursor(0,  24); display.print((int)(1000.*mx)); 
    display.setCursor(24, 24); display.print((int)(1000.*my)); 
    display.setCursor(48, 24); display.print((int)(1000.*mz)); 
    display.setCursor(72, 24); display.print("mG");   
   
    display.setCursor(0,  32); display.print("T "); 
    display.setCursor(40, 32); display.print(temperature, 1); display.print(" C");
    display.display();
    
    count = millis();
    digitalWrite(ledPin, !digitalRead(ledPin));
    }
 

  // One can use the interrupt pins to detect a motion/tap condition; 
  // here we just check the INT_SOURCE register to interpret the motion interrupt condition
    byte source = readByte(FXOS8700CQ_ADDRESS, INT_SOURCE);  // Read the interrupt source register
  // Manage sleep/wake interrupts
  if(source & 0x80) {  // Check if interrupt source is sleep/wake interrupt
   if(!sleepMode) {
    Serial.println("entering sleep mode");
    display.setCursor(0, 40); display.print("sleep on");
    sleepMode = true;
    }
    else {
    Serial.println("exiting sleep mode");
    display.setCursor(0, 40); display.print("active");
    sleepMode = false;
    }
    
    readByte(FXOS8700CQ_ADDRESS, SYSMOD); // Clear sleep interrupt
    display.display();                    // Write message to display
    delay(1000);                          // Delay a while so we can see the message
  }

  // If interrupt is due to motion control trigger...
  if (source & 0x10)         // If the p/l bit is set, go check those registers
      portraitLandscapeHandler();
    else if (source & 0x08)  // If tap register is set go check that
      tapHandler();
    else if (source & 0x04)  // If motion detection is set go check that
      motionDetect();
 //   else if (source & 0x02)  // If acceleration vector magnitude interrupt
 //     display.setCursor(0,40); display.print("Amagnit!"); 
//      display.display();     // Write message to display
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////Useful functions
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

void getAres() {
  switch (accelFSR)
  {
 	// Possible accelerometer scales (and their register bit settings) are:
	// 2 gs (00), 4 gs (01), 8 gs (10). 
    case AFS_2g:
          aRes = 2.0/8192.0;
          break;
    case AFS_4g:
          aRes = 4.0/8192.0;
          break;
    case AFS_8g:
          aRes = 8.0/8192.0;
          break;
  }
}

void readAccelData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  readBytes(FXOS8700CQ_ADDRESS, OUT_X_MSB, 6, &rawData[0]);  // Read the six raw data registers into data array
  destination[0] = ((int16_t) rawData[0] << 8 | rawData[1]) >> 2;
  destination[1] = ((int16_t) rawData[2] << 8 | rawData[3]) >> 2;
  destination[2] = ((int16_t) rawData[4] << 8 | rawData[5]) >> 2;
}

void readMagData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  readBytes(FXOS8700CQ_ADDRESS, M_OUT_X_MSB, 6, &rawData[0]);  // Read the six raw data registers into data array
  destination[0] = ((int16_t) rawData[0] << 8 | rawData[1]);
  destination[1] = ((int16_t) rawData[2] << 8 | rawData[3]);
  destination[2] = ((int16_t) rawData[4] << 8 | rawData[5]);
}

int8_t readTempData()
{
  return (int8_t) readByte(FXOS8700CQ_ADDRESS, TEMP);  // Read the 8-bit 2's complement data register 
}
 
// This function will read the status of the tap source register.
// Print if there's been a single or double tap, and on what axis.
void tapHandler()
{
  byte source = readByte(FXOS8700CQ_ADDRESS, PULSE_SRC);  // Reads the PULSE_SRC register
  if (source & 0x10)  // If AxX bit is set
  {
    if (source & 0x08)  // If DPE (double puls) bit is set
      Serial.print("    Double Tap (2) on X");  // tabbing here for visibility
    else
      Serial.print("Single (1) tap on X");

    if (source & 0x01)  { // If PoIX is set
      Serial.println(" -");
      display.setCursor(0,40); display.print("tap on -x"); }
    else {
      Serial.println(" +");
        display.setCursor(0,40); display.print("tap on +x"); }
  }
  if (source & 0x20)  // If AxY bit is set
  {
    if (source & 0x08)  // If DPE (double pulse) bit is set
      Serial.print("    Double Tap (2) on Y");
    else
      Serial.print("Single (1) tap on Y");

    if (source & 0x02) { // If PoIY is set
      Serial.println(" -");
      display.setCursor(0,40); display.print("tap on -y"); }
    else {
      Serial.println(" +");
      display.setCursor(0,40); display.print("tap on +y"); }
}
  if (source & 0x40)  // If AxZ bit is set
  {
    if (source & 0x08)  // If DPE (double puls) bit is set
      Serial.print("    Double Tap (2) on Z");
    else
      Serial.print("Single (1) tap on Z");
    if (source & 0x04) { // If PoIZ is set
      Serial.println(" -"); 
      display.setCursor(0,40); display.print("tap on -z"); }
    else {
      Serial.println(" +");
      display.setCursor(0,40); display.print("tap on +z"); }
   }
    display.display(); // Write message to display
    delay(1000); // Delay a while so we can see the message
}

// This function will read the p/l source register and
// print what direction the sensor is now facing
void portraitLandscapeHandler()
{
  byte pl = readByte(FXOS8700CQ_ADDRESS, PL_STATUS);  // Reads the PL_STATUS register
  switch((pl & 0x06)>>1)  // Check on the LAPO[1:0] bits
  {
  case 0:
    Serial.print("Portrait Up, ");
    display.setCursor(70,10); display.print("PU");
    display.display();
    delay(1000);
    break;
  case 1:
    Serial.print("Portrait Down, ");
    display.setCursor(70,10); display.print("PD");
    display.display();
    delay(1000);
    break;
  case 2:
    Serial.print("Landscape Right, ");
    display.setCursor(70,10); display.print("LR");
    display.display();
    delay(1000);
    break;
  case 3: 
    Serial.print("Landscape Left, ");
    display.setCursor(70,10); display.print("LL");
    display.display();
    delay(1000);
    break;
  }
  
  if (pl & 0x01) { // Check the BAFRO bit
    Serial.print("Back");
    display.setCursor(70,10); display.print("BK");
    display.display();
  } else {
    Serial.print("Front");
    display.setCursor(70,10); display.print("FT");
    display.display();
  }
  if (pl & 0x40) { // Check the LO bit
    Serial.println(", Z-tilt!");
    display.setCursor(70,10); display.print("Z!");
    display.display();
  }
}

// This function will read the motion detection source register and
// print motion direction
void motionDetect()
{
    byte source = readByte(FXOS8700CQ_ADDRESS, A_FFMT_SRC);
  if((source >> 7) == 1) {  // If Event Active flag set in the FF_MT_SRC register

   if (source & 0x02)  // If XHE bit is set, x-motion detected
  {
    if (source & 0x01)  { // If XHP is 1, x event was negative g
      Serial.println(" -");
      display.setCursor(0,40); display.print("motion to -x"); }
    else {
      Serial.println(" +");
      display.setCursor(0,40); display.print("motion to +x"); }
  }
  if ((source & 0x08)==0x08)  // If YHE bit is set, y-motion detected
  {
    if (source & 0x04) { // If YHP is set, y event was negative g
      Serial.println(" -");
      display.setCursor(0,40); display.print("motion to -y"); }
    else {
      Serial.println(" +");
      display.setCursor(0,40); display.print("motion to +y"); }
  }
  if (source & 0x20)  // If ZHE bit is set, z-motion detected
  {
    if (source & 0x10) { // If ZHP is set
      Serial.println(" -"); 
      display.setCursor(0,40); display.print("motion to -z"); }
    else {
      Serial.println(" +");
      display.setCursor(0,40); display.print("motion to +z"); }
  }
  display.display();  // Display motion message
  delay(1000); // Wait a while so we can the message
}
} 

void calibrateFXOS8700CQ()
{
  int32_t accel_bias[3] = {0, 0, 0};
  uint16_t ii, fcount;
  int16_t temp[3];
  
  // Clear all interrupts by reading the data output and F_STATUS registers
  readAccelData(temp);
  readByte(FXOS8700CQ_ADDRESS, STATUS);
  
  FXOS8700CQStandby();  // Must be in standby to change registers

  writeByte(FXOS8700CQ_ADDRESS, CTRL_REG1, 0x03 << 3); // select 100 Hz ODR
  fcount = 100;                                        // sample for 1 second
  writeByte(FXOS8700CQ_ADDRESS, XYZ_DATA_CFG, 0x00);   // select 2 g full scale
  uint16_t accelsensitivity = 4096;                    // 4096 LSB/g

  FXOS8700CQActive();  // Set to active to start collecting data
   
  uint8_t rawData[6];  // x/y/z FIFO accel data stored here
  for(ii = 0; ii < fcount; ii++)   // construct count sums for each axis
  {
  readBytes(FXOS8700CQ_ADDRESS, OUT_X_MSB, 6, &rawData[0]);  // Read the FIFO data registers into data array
  temp[0] = ((int16_t) rawData[0] << 8 | rawData[1]) >> 2;
  temp[1] = ((int16_t) rawData[2] << 8 | rawData[3]) >> 2;
  temp[2] = ((int16_t) rawData[4] << 8 | rawData[5]) >> 2;
  
  accel_bias[0] += (int32_t) temp[0];
  accel_bias[1] += (int32_t) temp[1];
  accel_bias[2] += (int32_t) temp[2];
  
  delay(25);  // wait for a new data reading (100 Hz)
  }
 
  accel_bias[0] /= (int32_t) fcount; // get average values
  accel_bias[1] /= (int32_t) fcount;
  accel_bias[2] /= (int32_t) fcount;
  
  if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
  else {accel_bias[2] += (int32_t) accelsensitivity;}

  rawData[0] = (-accel_bias[0]/8) & 0xFF; // get average values
  rawData[1] = (-accel_bias[1]/8) & 0xFF; // get average values
  rawData[2] = (-accel_bias[2]/8) & 0xFF; // get average values
  
  FXOS8700CQStandby();  // Must be in standby to change registers
  
  writeByte(FXOS8700CQ_ADDRESS, OFF_X, rawData[0]); // X-axis compensation  
  writeByte(FXOS8700CQ_ADDRESS, OFF_Y, rawData[1]); // Y-axis compensation  
  writeByte(FXOS8700CQ_ADDRESS, OFF_Z, rawData[2]); // z-axis compensation 

  FXOS8700CQActive();  // Set to active to start reading
}
  
  
  // Set up sensor software reset
void FXOS8700CQMagOffset() 
{
  FXOS8700CQStandby();  // Must be in standby to change registers
  
  writeByte(FXOS8700CQ_ADDRESS, M_OFF_X_MSB, 0x00); 
  writeByte(FXOS8700CQ_ADDRESS, M_OFF_X_LSB, 0x00); 
  writeByte(FXOS8700CQ_ADDRESS, M_OFF_Y_MSB, 0x00); 
  writeByte(FXOS8700CQ_ADDRESS, M_OFF_Y_LSB, 0x00); 
  writeByte(FXOS8700CQ_ADDRESS, M_OFF_Z_MSB, 0x00); 
  writeByte(FXOS8700CQ_ADDRESS, M_OFF_Z_LSB, 0x00); 

  FXOS8700CQActive();  // Set to active to start reading
}


  void sleepModeFXOS8700CQ()
{
    FXOS8700CQStandby();  // Must be in standby to change registers

  // These settings have to do with setting up the sleep mode
  // Set Auto-WAKE sample frequency when the device is in sleep mode
     writeByte(FXOS8700CQ_ADDRESS, ASLP_COUNT, 0x40 ); // sleep after ~36 seconds of inactivity at 6.25 Hz ODR

     writeByte(FXOS8700CQ_ADDRESS, CTRL_REG1, readByte(FXOS8700CQ_ADDRESS, CTRL_REG1) & ~(0xC0)); // clear bits 7 and 8
     writeByte(FXOS8700CQ_ADDRESS, CTRL_REG1, readByte(FXOS8700CQ_ADDRESS, CTRL_REG1) |  (0xC0)); // select 1.56 Hz sleep mode sample frequency for low power

  // set sleep power mode scheme
     writeByte(FXOS8700CQ_ADDRESS, CTRL_REG2, readByte(FXOS8700CQ_ADDRESS, CTRL_REG2) & ~(0x18)); // clear bits 3 and 4
     writeByte(FXOS8700CQ_ADDRESS, CTRL_REG2, readByte(FXOS8700CQ_ADDRESS, CTRL_REG2) |  (0x18)); // select low power mode
     
  // Enable auto SLEEP
     writeByte(FXOS8700CQ_ADDRESS, CTRL_REG2, readByte(FXOS8700CQ_ADDRESS, CTRL_REG2) & ~(0x04)); // clear bit 2
     writeByte(FXOS8700CQ_ADDRESS, CTRL_REG2, readByte(FXOS8700CQ_ADDRESS, CTRL_REG2) |  (0x04)); // enable auto sleep mode

  // set sleep mode interrupt scheme
     writeByte(FXOS8700CQ_ADDRESS, CTRL_REG3, readByte(FXOS8700CQ_ADDRESS, CTRL_REG3) & ~(0x7C)); // clear bits 2, 3, 4, 5, and 6
     // select wake on transient, orientation change, pulse, freefall/motion detect, or acceleration vector magnitude
     writeByte(FXOS8700CQ_ADDRESS, CTRL_REG3, readByte(FXOS8700CQ_ADDRESS, CTRL_REG3) |  (0x7C)); 
     
   // Enable Auto-SLEEP/WAKE interrupt
     writeByte(FXOS8700CQ_ADDRESS, CTRL_REG4, readByte(FXOS8700CQ_ADDRESS, CTRL_REG4) & ~(0x80)); // clear bit 7
     writeByte(FXOS8700CQ_ADDRESS, CTRL_REG4, readByte(FXOS8700CQ_ADDRESS, CTRL_REG4) |  (0x80)); // select  Auto-SLEEP/WAKE interrupt enable
 
     FXOS8700CQActive();  // Set to active to start reading
}
  
  
// Set up sensor software reset
void FXOS8700CQReset() 
{
writeByte(FXOS8700CQ_ADDRESS,   CTRL_REG2, 0x40); // set reset bit to 1 to assert accel software reset to zero at end of boot process
writeByte(FXOS8700CQ_ADDRESS, M_CTRL_REG2, 0x40); // set reset bit to 1 to assert mag software reset to zero at end of boot process
}


// Initialize the FXOS8700CQ registers 
// See the many application notes for more info on setting all of these registers:
// http://www.freescale.com/webapp/sps/site/prod_summary.jsp?code=FXOS8700CQQ
// Feel free to modify any values, these are settings that work well for me.
void initFXOS8700CQ()
{
    FXOS8700CQStandby();  // Must be in standby to change registers

    // Configure the accelerometer
    writeByte(FXOS8700CQ_ADDRESS, XYZ_DATA_CFG, accelFSR);  // Choose the full scale range to 2, 4, or 8 g.
    writeByte(FXOS8700CQ_ADDRESS, CTRL_REG1, readByte(FXOS8700CQ_ADDRESS, CTRL_REG1) & ~(0x38)); // Clear the 3 data rate bits 5:3
    if (accelODR <= 7) writeByte(FXOS8700CQ_ADDRESS, CTRL_REG1, readByte(FXOS8700CQ_ADDRESS, CTRL_REG1) | (accelODR << 3));      
    writeByte(FXOS8700CQ_ADDRESS, CTRL_REG2, readByte(FXOS8700CQ_ADDRESS, CTRL_REG2) & ~(0x03)); // clear bits 0 and 1
    writeByte(FXOS8700CQ_ADDRESS, CTRL_REG2, readByte(FXOS8700CQ_ADDRESS, CTRL_REG2) |  (0x02)); // select normal(00) or high resolution (10) mode
    
    // Configure the magnetometer
    writeByte(FXOS8700CQ_ADDRESS, M_CTRL_REG1, 0x80 | magOSR << 2 | 0x03); // Set auto-calibration, set oversampling, enable hybrid mode 
    
    // Configure interrupts 1 and 2
    writeByte(FXOS8700CQ_ADDRESS, CTRL_REG3, readByte(FXOS8700CQ_ADDRESS, CTRL_REG3) & ~(0x02)); // clear bits 0, 1 
    writeByte(FXOS8700CQ_ADDRESS, CTRL_REG3, readByte(FXOS8700CQ_ADDRESS, CTRL_REG3) |  (0x02)); // select ACTIVE HIGH, push-pull interrupts    
    writeByte(FXOS8700CQ_ADDRESS, CTRL_REG4, readByte(FXOS8700CQ_ADDRESS, CTRL_REG4) & ~(0x1D)); // clear bits 0, 3, and 4
    writeByte(FXOS8700CQ_ADDRESS, CTRL_REG4, readByte(FXOS8700CQ_ADDRESS, CTRL_REG4) |  (0x1D)); // DRDY, Freefall/Motion, P/L and tap ints enabled  
    writeByte(FXOS8700CQ_ADDRESS, CTRL_REG5, 0x01);  // DRDY on INT1, P/L and taps on INT2
   
    FXOS8700CQActive();  // Set to active to start reading
}


void accelMotionIntFXOS8700CQ()
{
    FXOS8700CQStandby();  // Must be in standby to change registers
  // This is adapted from Jim Lindblom's sketch originally found on http://www.sparkfun.com
  // Set up portrait/landscape registers - 4 steps:
  // 1. Enable P/L
  // 2. Set the back/front angle trigger points (z-lock)
  // 3. Set the threshold/hysteresis angle
  // 4. Set the debouce rate
  // For more info check out this app note: http://cache.freescale.com/files/sensors/doc/app_note/AN4068.pdf
  writeByte(FXOS8700CQ_ADDRESS, PL_CFG, 0x40);         // 1. Enable P/L
//  writeByte(FXOS8700CQ_ADDRESS, PL_BF_ZCOMP, 0x04);    // 2. 29deg z-lock 
//  writeByte(FXOS8700CQ_ADDRESS, P_L_THS_REG, 0x84);    // 3. 45deg thresh, 14deg hyst 
  writeByte(FXOS8700CQ_ADDRESS, PL_COUNT, 0x50);       // 4. debounce counter at 100ms (at 800 hz)

  /* Set up single and double tap - 5 steps:
   1. Set up single and/or double tap detection on each axis individually.
   2. Set the threshold - minimum required acceleration to cause a tap.
   3. Set the time limit - the maximum time that a tap can be above the threshold
   4. Set the pulse latency - the minimum required time between one pulse and the next
   5. Set the second pulse window - maximum allowed time between end of latency and start of second pulse
   for more info check out this app note: http://cache.freescale.com/files/sensors/doc/app_note/AN4072.pdf */
  writeByte(FXOS8700CQ_ADDRESS, PULSE_CFG, 0x7F);  // 1. enable single/double taps on all axes
  // writeByte(FXOS8700CQ_ADDRESS, PULSE_CFS, 0x55);  // 1. single taps only on all axes
  // writeByte(FXOS8700CQ_ADDRESS, PULSE_CFS, 0x6A);  // 1. double taps only on all axes
  writeByte(FXOS8700CQ_ADDRESS, PULSE_THSX, 0x04);  // 2. x thresh at 0.25g, multiply the value by 0.0625g/LSB to get the threshold
  writeByte(FXOS8700CQ_ADDRESS, PULSE_THSY, 0x04);  // 2. y thresh at 0.25g, multiply the value by 0.0625g/LSB to get the threshold
  writeByte(FXOS8700CQ_ADDRESS, PULSE_THSZ, 0x04);  // 2. z thresh at 0.25g, multiply the value by 0.0625g/LSB to get the threshold
  writeByte(FXOS8700CQ_ADDRESS, PULSE_TMLT, 0x30);  // 3. 2.55s time limit at 100Hz odr, this is very dependent on data rate, see the app note
  writeByte(FXOS8700CQ_ADDRESS, PULSE_LTCY, 0xA0);  // 4. 5.1s 100Hz odr between taps min, this also depends on the data rate
  writeByte(FXOS8700CQ_ADDRESS, PULSE_WIND, 0xFF);  // 5. 10.2s (max value)  at 100 Hz between taps max

  // Set up motion detection
  writeByte(FXOS8700CQ_ADDRESS, A_FFMT_CFG, 0x58); // Set free fall OR motion flag on x and y axes
  writeByte(FXOS8700CQ_ADDRESS, A_FFMT_THS, 0x84); // Clear debounce counter when condition no longer obtains, set x/y/z/common threshold to 0.25 g
//  writeByte(FXOS8700CQ_ADDRESS, A_FFMT_THS_X_MSB, 0x80); // Set x-axis threshold individually to 0.25 g
//  writeByte(FXOS8700CQ_ADDRESS, A_FFMT_THS_X_LSB, 0x04);  
//  writeByte(FXOS8700CQ_ADDRESS, A_FFMT_THS_Y_MSB, 0x80); // Set y-axis threshold individually to 0.25 g
//  writeByte(FXOS8700CQ_ADDRESS, A_FFMT_THS_Y_LSB, 0x04);  
//  writeByte(FXOS8700CQ_ADDRESS, A_FFMT_THS_Z_MSB, 0x80); // Set z-axis threshold individually to 1.25 g
//  writeByte(FXOS8700CQ_ADDRESS, A_FFMT_THS_Z_LSB, 0x14);  
  writeByte(FXOS8700CQ_ADDRESS, A_FFMT_COUNT, 0x08); // Set debounce to 0.08 s at 100 Hz

  // Use user-specified reference value before and after trigger event, enable accelerometer magnitude function  
  writeByte(FXOS8700CQ_ADDRESS, A_VECM_CFG, 0x78); 
  writeByte(FXOS8700CQ_ADDRESS, A_VECM_THS_MSB, 0x80 | 0x0F); // Clear debounce counter when event falls below threshold; bit 4:0 MSB THS 
  writeByte(FXOS8700CQ_ADDRESS, A_VECM_THS_LSB, 0xFF);        // Specify threshold at 0.3 g = 1228 * 0.244 g at FSR of +/-2 g
  writeByte(FXOS8700CQ_ADDRESS, A_VECM_CNT, 0x01);            // Set debounce time to 80 x 1.25 ms (in high res mode) = 100 ms at 400 Hz (200 Hz hybrid mode)
  writeByte(FXOS8700CQ_ADDRESS, A_VECM_INITX_MSB, 0x00);      // set x-axis vector magnitude reference at 0 g
  writeByte(FXOS8700CQ_ADDRESS, A_VECM_INITX_LSB, 0x00);
  writeByte(FXOS8700CQ_ADDRESS, A_VECM_INITY_MSB, 0x00);      // set y-axis vector magnitude reference at 0 g
  writeByte(FXOS8700CQ_ADDRESS, A_VECM_INITY_LSB, 0x00);
  writeByte(FXOS8700CQ_ADDRESS, A_VECM_INITZ_MSB, 0x10);      // set z-axis vector magnitude reference at +1 g = 4096 at +/- 2 g FSR
  writeByte(FXOS8700CQ_ADDRESS, A_VECM_INITZ_LSB, 0x00);
  
  FXOS8700CQActive();  // Set to active to start reading
}


// Sets the FXOS8700CQ to standby mode.
// It must be in standby to change most register settings
void FXOS8700CQStandby()
{
  byte c = readByte(FXOS8700CQ_ADDRESS, 0x2A);
  writeByte(FXOS8700CQ_ADDRESS, CTRL_REG1, c & ~(0x01));
}


// Sets the FXOS8700CQ to active mode.
// Needs to be in this mode to output data
void FXOS8700CQActive()
{
  byte c = readByte(FXOS8700CQ_ADDRESS, 0x2A);
  writeByte(FXOS8700CQ_ADDRESS, CTRL_REG1, c | 0x01);
}


        void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
	Wire.beginTransmission(address);  // Initialize the Tx buffer
	Wire.write(subAddress);           // Put slave register address in Tx buffer
	Wire.write(data);                 // Put data in Tx buffer
	Wire.endTransmission();           // Send the Tx buffer
}

        uint8_t readByte(uint8_t address, uint8_t subAddress)
{
	uint8_t data; // `data` will store the register data	 
	Wire.beginTransmission(address);         // Initialize the Tx buffer
	Wire.write(subAddress);	                 // Put slave register address in Tx buffer
	Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
	Wire.requestFrom(address, (uint8_t) 1);  // Read one uint8_t from slave register address 
	data = Wire.read();                      // Fill Rx buffer with result
	return data;                             // Return data read from slave register
}

        void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{  
	Wire.beginTransmission(address);   // Initialize the Tx buffer
	Wire.write(subAddress);            // Put slave register address in Tx buffer
	Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
	uint8_t i = 0;
        Wire.requestFrom(address, count);  // Read bytes from slave register address 
	while (Wire.available()) {
        dest[i++] = Wire.read(); }         // Put read results in the Rx buffer
}
