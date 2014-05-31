 /* MMA8652Q Example Code
 Original sketch by: Jim Lindblom
 SparkFun Electronics
 date: November 17, 2011
 license: Beerware - Use this code however you'd like. If you 
 find it useful you can buy me a beer some time.
 
 Modified by: Kris Winer, May 27, 2014 to include reset, accelerometer calibration, sleep mode functionality as well as
 parameterizing the register addresses. Added LCD functions to allow display to on breadboard monitor and motion detection
 , etc.
 
 This code should provide example usage for most features of
 the MMA8652Q 3-axis, I2C accelerometer. In the loop function
 the accelerometer interrupt outputs will be polled, and either
 the x/y/z accel data will be output, or single/double-taps,
 portrait/landscape changes will be announced to the serial port.
 Feel free to comment/uncomment out some of the Serial.print 
 lines so you can see the information you're most intereseted in.
 
 The skeleton is here, feel free to cut/paste what code you need.
 Play around with the settings in initMMA8652Q. Try running the
 code without printing the accel values, to really appreciate
 the single/double-tap and portrait landscape functions. The
 P/L stuff is really neat, something not many accelerometers have.
 
 Hardware setup:
 MMA8652 Breakout ------------ Arduino
 3.3V --------------------- 3.3V
 SDA ----------------------- A4
 SCL ----------------------- A5
 INT2 ---------------------- D5
 INT1 ---------------------- D4
 GND ---------------------- GND
 
 SDA and SCL should have external pull-up resistors (to 3.3V).
 10k resistors are on the FRDM-FXS-MULTI breakout board.
 
 Note: The MMA8652 is an I2C sensor; here we make use of the Arduino Wire library.
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

// Define registers per MMA8652Q, Document Number: MMA8652FC
// Data Sheet: Technical Data Rev. 2.0, 02/2013 3-Axis, 12-bit/8-bit Digital Accelerometer
// Freescale Semiconductor Data Sheet
#define STATUS           0x00
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
#define FF_MT_CFG        0x15
#define FF_MT_SRC        0x16
#define FF_MT_THS        0x17
#define FF_MT_COUNT      0x18
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

// The SparkFun breakout board defaults to 1, set to 0 if SA0 jumper on the bottom of the board is set
// Seven bit device address is 0011100 or 0011101 if SAO pin 0 or 1, respectively
#define SA0 1
#if SA0
#define MMA8652_ADDRESS 0x1D  // SA0 is high, 0x1C if low
#else
#define MMA8652_ADDRESS 0x1C
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

// Specify sensor full scale
uint8_t accelFSR = AFS_2g;
uint8_t accelODR = AODR_200HZ;
float aRes; // scale resolutions per LSB for the sensor
// Set the scale below either 2, 4 or 8

// Pin definitions
int int1Pin = 12;  // These can be changed, 2 and 3 are the Arduinos ext int pins
int ledPin  = 13;  // Pro Mini led

int16_t accelCount[3];  // Stores the 12-bit signed value
float ax, ay, az;       // Stores the real accel value in g's
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
  display.setTextSize(2);
  display.setCursor(0,0); display.print("MMA8652");
  display.setTextSize(1);
  display.setCursor(0, 20); display.print("3-axis 12-bit");
  display.setCursor(0, 30); display.print("accelerometer");
  display.setCursor(20,40); display.print("2 mg LSB");
  display.display();
  delay(1000);

// Set up for data display
  display.setTextSize(1); // Set text size to normal, 2 is twice normal etc.
  display.setTextColor(BLACK); // Set pixel color; 1 on the monochrome screen
  display.clearDisplay();   // clears the screen and buffer
  
  // Read the WHO_AM_I register, this is a good test of communication
  byte c = readByte(MMA8652_ADDRESS, WHO_AM_I);  // Read WHO_AM_I register
  display.clearDisplay();
  display.setCursor(0,0); display.print("MMA8652");  
  display.setCursor(0,10); display.print("I Am");
  display.setCursor(0, 20); display.print("Ox");display.print(c, HEX);  
  display.setCursor(0, 30); display.print("I Should be"); 
  display.setCursor(0, 40); display.print("Ox");display.print(0x4A, HEX);  
  display.display();
  delay(1000);

  if (c == 0x4A) // WHO_AM_I should always be 0x4A
  {  
    MMA8652Reset(); // Start by resetting sensor device to default settings
    calibrateMMA8652();
    initMMA8652();  // init the accelerometer if communication is OK
    Serial.println("MMA8652Q is online...");

  display.clearDisplay();
  display.setCursor(0,0); display.print("MMA8652");
  display.setCursor(0, 20); display.print("x bias "); display.print((int8_t)2*readByte(MMA8652_ADDRESS, OFF_X)); display.print(" mg");
  display.setCursor(0, 30); display.print("y bias "); display.print((int8_t)2*readByte(MMA8652_ADDRESS, OFF_Y)); display.print(" mg");
  display.setCursor(0, 40); display.print("z bias "); display.print((int8_t)2*readByte(MMA8652_ADDRESS, OFF_Z)); display.print(" mg");
  display.display();
  delay(2000);
  }
  else
  {
    Serial.print("Could not connect to MMA8652Q: 0x");
    Serial.println(c, HEX);
    while(1) ; // Loop forever if communication doesn't happen
  }
}

void loop()
{  
  // One can use the interrupt pins to detect a data ready condition; here we just check the STATUS register for a data ready bit
  if(readByte(MMA8652_ADDRESS, STATUS) & 0x08)  // When this bit set, all axes have new data
  {
    readAccelData(accelCount);  // Read the x/y/z adc values
    getAres();                  // get accelerometer sensitivity
    ax = (float)accelCount[0]*aRes;  // get actual g value, this depends on scale being set
    ay = (float)accelCount[1]*aRes;  // also subtract averaged accelerometer biases
    az = (float)accelCount[2]*aRes;  
    
    // Print out values
      Serial.print("x-acceleration = "); Serial.print(1000.*ax); Serial.print(" mg");   
      Serial.print("y-acceleration = "); Serial.print(1000.*ay); Serial.print(" mg");   
      Serial.print("z-acceleration = "); Serial.print(1000.*az); Serial.print(" mg");  

   uint32_t deltat = millis() - count;
   if (deltat > 500) { // update LCD once per half-second independent of read rate

    display.clearDisplay();
     
    display.setCursor(0,0); display.print("MMA8652");
    display.drawPixel(7, 8, BLACK); display.drawPixel(9, 8, BLACK); 
    display.setCursor(0,8); display.print(" x ");  display.print((int)(1000*ax)); 
    display.setCursor(43,8); display.print(" mg");
    display.drawPixel(7, 16, BLACK); display.drawPixel(9, 16, BLACK); 
    display.setCursor(0,16); display.print(" y "); display.print((int)(1000*ay)); 
    display.setCursor(43,16); display.print(" mg");
    display.drawPixel(7, 24, BLACK); display.drawPixel(9, 24, BLACK); 
    display.setCursor(0,24); display.print(" z "); display.print((int)(1000*az)); 
    display.setCursor(43,24); display.print(" mg");
    display.display();
    count = millis();
    digitalWrite(ledPin, !digitalRead(ledPin));
    }
  }

  // One can use the interrupt pins to detect a motion/tap condition; 
  // here we just check the INT_SOURCE register to interpret the motion interrupt condition
  byte source = readByte(MMA8652_ADDRESS, INT_SOURCE);  // Read the interrupt source register
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
    
  readByte(MMA8652_ADDRESS, SYSMOD); // clear sleep interrupt
  display.display(); // Write message to display
  delay(1000); //Delay a while so we can see the message
  }

  if (source & 0x10)  // If the p/l bit is set, go check those registers
      portraitLandscapeHandler();
    else if (source & 0x08)  // If tap register is set go check that
      tapHandler();
    else if (source & 0x04)  // Otherwise, if motion detection is set go check that
      motionDetect();
}

void getAres() {
  switch (accelFSR)
  {
 	// Possible accelerometer scales (and their register bit settings) are:
	// 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11). 
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case AFS_2g:
          aRes = 2.0/2048.0;
          break;
    case AFS_4g:
          aRes = 4.0/2048.0;
          break;
    case AFS_8g:
          aRes = 8.0/2048.0;
          break;
  }
}

void readAccelData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  readBytes(MMA8652_ADDRESS, OUT_X_MSB, 6, &rawData[0]);  // Read the six raw data registers into data array
  destination[0] = ((int16_t) rawData[0] << 8 | rawData[1]) >> 4;
  destination[1] = ((int16_t) rawData[2] << 8 | rawData[3]) >> 4;
  destination[2] = ((int16_t) rawData[4] << 8 | rawData[5]) >> 4;
}

 
// This function will read the status of the tap source register.
// Print if there's been a single or double tap, and on what axis.
void tapHandler()
{
  byte source = readByte(MMA8652_ADDRESS, PULSE_SRC);  // Reads the PULSE_SRC register
  if (source & 0x10)  // If AxX bit is set
  {
    if (source & 0x08)  // If DPE (double puls) bit is set
      Serial.print("    Double Tap (2) on X");  // tabbing here for visibility
    else
      Serial.print("Single (1) tap on X");

    if (source & 0x01)  { // If PoIX is set
      Serial.println(" -");
      display.setCursor(0,32); display.print("tap on -x"); }
    else {
      Serial.println(" +");
        display.setCursor(0,32); display.print("tap on +x"); }
  }
  if (source & 0x20)  // If AxY bit is set
  {
    if ((source & 0x08)==0x08)  // If DPE (double pulse) bit is set
      Serial.print("    Double Tap (2) on Y");
    else
      Serial.print("Single (1) tap on Y");

    if (source & 0x02) { // If PoIY is set
      Serial.println(" -");
      display.setCursor(0,32); display.print("tap on -y"); }
    else {
      Serial.println(" +");
      display.setCursor(0,32); display.print("tap on +y"); }
}
  if (source & 0x40)  // If AxZ bit is set
  {
    if (source & 0x08)  // If DPE (double puls) bit is set
      Serial.print("    Double Tap (2) on Z");
    else
      Serial.print("Single (1) tap on Z");
    if (source & 0x04) { // If PoIZ is set
      Serial.println(" -"); 
      display.setCursor(0,32); display.print("tap on -z"); }
    else {
      Serial.println(" +");
      display.setCursor(0,32); display.print("tap on +z"); }
   }
    display.display(); // Write message to display
    delay(1000); // Delay a while so we can see the message
}

// This function will read the p/l source register and
// print what direction the sensor is now facing
void portraitLandscapeHandler()
{
  byte pl = readByte(MMA8652_ADDRESS, 0x10);  // Reads the PL_STATUS register
  switch((pl&0x06)>>1)  // Check on the LAPO[1:0] bits
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
  if (pl & 0x01)  // Check the BAFRO bit
    Serial.print("Back");
  else
    Serial.print("Front");
  if (pl & 0x40)  // Check the LO bit
    Serial.print(", Z-tilt!");
  Serial.println();
}

// This function will read the motion detection source register and
// print motion direction
void motionDetect()
{
    byte source = readByte(MMA8652_ADDRESS, FF_MT_SRC);
  if((source >> 7) == 1) {  // If Event Active flag set in the FF_MT_SRC register

   if (source & 0x02)  // If XHE bit is set, x-motion detected
  {
    if (source & 0x01)  { // If XHP is 1, x event was negative g
      Serial.println(" -");
      display.setCursor(0,32); display.print("motion to -x"); }
    else {
      Serial.println(" +");
      display.setCursor(0,32); display.print("motion to +x"); }
  }
  if ((source & 0x08)==0x08)  // If YHE bit is set, y-motion detected
  {
    if (source & 0x04) { // If YHP is set, y event was negative g
      Serial.println(" -");
      display.setCursor(0,32); display.print("motion to -y"); }
    else {
      Serial.println(" +");
      display.setCursor(0,32); display.print("motion to +y"); }
  }
  if (source & 0x20)  // If ZHE bit is set, z-motion detected
  {
    if (source & 0x10) { // If ZHP is set
      Serial.println(" -"); 
      display.setCursor(0,32); display.print("motion to -z"); }
    else {
      Serial.println(" +");
      display.setCursor(0,32); display.print("motion to +z"); }
  }
  display.display();  // Display motion message
  delay(1000); // Wait a while so we can the message
}
} 

void calibrateMMA8652()
{
  int32_t accel_bias[3] = {0, 0, 0};
  uint16_t ii, fcount;
  int16_t temp[3];
  
  // Clear all interrupts by reading the data output and F_STATUS registers
  readAccelData(temp);
  readByte(MMA8652_ADDRESS, F_STATUS);
  
  MMA8652Standby();  // Must be in standby to change registers

  writeByte(MMA8652_ADDRESS, CTRL_REG1, 0x01);      // select 100 Hz ODR
  fcount = 100;                                     // sample for 1 second
  writeByte(MMA8652_ADDRESS, XYZ_DATA_CFG, 0x00);   // select 2 g full scale
  uint16_t accelsensitivity = 1024;                 // 1024 LSB/g

  MMA8652Active();  // Set to active to start collecting data
   
  uint8_t rawData[6];  // x/y/z FIFO accel data stored here
  for(ii = 0; ii < fcount; ii++)   // construct count sums for each axis
  {
  readBytes(MMA8652_ADDRESS, OUT_X_MSB, 6, &rawData[0]);  // Read the FIFO data registers into data array
  temp[0] = ((int16_t) rawData[0] << 8 | rawData[1]) >> 4;
  temp[1] = ((int16_t) rawData[2] << 8 | rawData[3]) >> 4;
  temp[2] = ((int16_t) rawData[4] << 8 | rawData[5]) >> 4;
  
  accel_bias[0] += (int32_t) temp[0];
  accel_bias[1] += (int32_t) temp[1];
  accel_bias[2] += (int32_t) temp[2];
  
  delay(15);  // wait for a new data reading (100 Hz)
  }
 
  accel_bias[0] /= (int32_t) fcount; // get average values
  accel_bias[1] /= (int32_t) fcount;
  accel_bias[2] /= (int32_t) fcount;
  
  if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
  else {accel_bias[2] += (int32_t) accelsensitivity;}

  rawData[0] = (-accel_bias[0]/2) & 0xFF; // get average values
  rawData[1] = (-accel_bias[1]/2) & 0xFF; // get average values
  rawData[2] = (-accel_bias[2]/2) & 0xFF; // get average values
  
  MMA8652Standby();  // Must be in standby to change registers
  
  writeByte(MMA8652_ADDRESS, OFF_X, rawData[0]); // X-axis compensation  
  writeByte(MMA8652_ADDRESS, OFF_Y, rawData[1]); // Y-axis compensation  
  writeByte(MMA8652_ADDRESS, OFF_Z, rawData[2]); // z-axis compensation 

  MMA8652Active();  // Set to active to start reading
}
  
  
// Set up sensor software reset
void MMA8652Reset() 
{
writeByte(MMA8652_ADDRESS, CTRL_REG2, 0x40); // set reset bit to 1 to assert software reset to zero at end of boot process
}

// Allow user compensation of acceleration errors
void MMA8652Offsets()
{
   MMA8652Standby();  // Must be in standby to change registers
   
   // Factory settings are pretty good; the settings below produce 1 mg error or less at 2 g full scale! For the device at rest on my table 
   // these values partially compensate for the slope of the table and the slope of the sensor in my breadboard. It is a pretty stable setup!
   // For negative values use 2's complement, i.e., -2 mg = 0xFF, etc.
   writeByte(MMA8652_ADDRESS, OFF_X, 0x00); // X-axis compensation; this is 0 mg
   writeByte(MMA8652_ADDRESS, OFF_Y, 0x00); // Y-axis compensation; this is 0 mg
   writeByte(MMA8652_ADDRESS, OFF_Z, 0x00); // z-axis compensation; this is  0 mg adjustment
   
   MMA8652Active();  // Set to active to start reading
}

// Initialize the MMA8652 registers 
// See the many application notes for more info on setting all of these registers:
// http://www.freescale.com/webapp/sps/site/prod_summary.jsp?code=MMA8652Q
// Feel free to modify any values, these are settings that work well for me.
void initMMA8652()
{
  MMA8652Standby();  // Must be in standby to change registers

  // Set up the full scale range to 2, 4, or 8g.
    writeByte(MMA8652_ADDRESS, XYZ_DATA_CFG, accelFSR);  

   // First clear CTRL_REG1
    writeByte(MMA8652_ADDRESS, CTRL_REG1, 0x00);
   // Setup the 3 data rate bits, from 0 to 7
    writeByte(MMA8652_ADDRESS, CTRL_REG1, readByte(MMA8652_ADDRESS, CTRL_REG1) & ~(0x38));
    if (accelODR <= 7)
    writeByte(MMA8652_ADDRESS, CTRL_REG1, readByte(MMA8652_ADDRESS, CTRL_REG1) | (accelODR << 3));      
    // set resolution
     writeByte(MMA8652_ADDRESS, CTRL_REG2, readByte(MMA8652_ADDRESS, CTRL_REG2) & ~(0x03)); // clear bits 0 and 1
     writeByte(MMA8652_ADDRESS, CTRL_REG2, readByte(MMA8652_ADDRESS, CTRL_REG2) |  (0x02)); // select normal(00) or high resolution (10) mode
    
// These settings have to do with setting up the sleep mode and should probably be broken up into a separate function
// set Auto-WAKE sample frequency when the device is in sleep mode

     writeByte(MMA8652_ADDRESS, 0x29, 0x40 ); // sleep after ~36 seconds of inactivity at 6.25 Hz ODR

     writeByte(MMA8652_ADDRESS, CTRL_REG1, readByte(MMA8652_ADDRESS, CTRL_REG1) & ~(0xC0)); // clear bits 7 and 8
     writeByte(MMA8652_ADDRESS, CTRL_REG1, readByte(MMA8652_ADDRESS, CTRL_REG1) |  (0xC0)); // select 1.56 Hz sleep mode sample frequency for low power

  // set sleep power mode scheme
     writeByte(MMA8652_ADDRESS, CTRL_REG2, readByte(MMA8652_ADDRESS, CTRL_REG2) & ~(0x18)); // clear bits 3 and 4
     writeByte(MMA8652_ADDRESS, CTRL_REG2, readByte(MMA8652_ADDRESS, CTRL_REG2) |  (0x18)); // select low power mode
     
  // Enable auto SLEEP
     writeByte(MMA8652_ADDRESS, CTRL_REG2, readByte(MMA8652_ADDRESS, CTRL_REG2) & ~(0x04)); // clear bit 2
     writeByte(MMA8652_ADDRESS, CTRL_REG2, readByte(MMA8652_ADDRESS, CTRL_REG2) |  (0x04)); // enable auto sleep mode

  // set sleep mode interrupt scheme
     writeByte(MMA8652_ADDRESS, CTRL_REG3, readByte(MMA8652_ADDRESS, CTRL_REG3) & ~(0x3C)); // clear bits 3, 4, 5, and 6
     writeByte(MMA8652_ADDRESS, CTRL_REG3, readByte(MMA8652_ADDRESS, CTRL_REG3) |  (0x3C)); // select wake on transient, orientation change, pulse, or freefall/motion detect
     
   // Enable Auto-SLEEP/WAKE interrupt
     writeByte(MMA8652_ADDRESS, CTRL_REG4, readByte(MMA8652_ADDRESS, CTRL_REG4) & ~(0x80)); // clear bit 7
     writeByte(MMA8652_ADDRESS, CTRL_REG4, readByte(MMA8652_ADDRESS, CTRL_REG4) |  (0x80)); // select  Auto-SLEEP/WAKE interrupt enable
   
  // Set up portrait/landscape registers - 4 steps:
  // 1. Enable P/L
  // 2. Set the back/front angle trigger points (z-lock)
  // 3. Set the threshold/hysteresis angle
  // 4. Set the debouce rate
  // For more info check out this app note: http://cache.freescale.com/files/sensors/doc/app_note/AN4068.pdf
  writeByte(MMA8652_ADDRESS, PL_CFG, 0x40);        // 1. Enable P/L
 // writeByte(MMA8652_ADDRESS, PL_BF_ZCOMP, 0x44); // 2. 29deg z-lock (don't think this register is actually writable)
 // writeByte(MMA8652_ADDRESS, P_L_THS_REG, 0x84); // 3. 45deg thresh, 14deg hyst (don't think this register is writable either)
  writeByte(MMA8652_ADDRESS, PL_COUNT, 0x50);      // 4. debounce counter at 100ms (at 800 hz)

  /* Set up single and double tap - 5 steps:
   1. Set up single and/or double tap detection on each axis individually.
   2. Set the threshold - minimum required acceleration to cause a tap.
   3. Set the time limit - the maximum time that a tap can be above the threshold
   4. Set the pulse latency - the minimum required time between one pulse and the next
   5. Set the second pulse window - maximum allowed time between end of latency and start of second pulse
   for more info check out this app note: http://cache.freescale.com/files/sensors/doc/app_note/AN4072.pdf */
  writeByte(MMA8652_ADDRESS, PULSE_CFG, 0x7F);  // 1. enable single/double taps on all axes
  // writeByte(MMA8652_ADDRESS, PULSE_CFS, 0x55);  // 1. single taps only on all axes
  // writeByte(MMA8652_ADDRESS, PULSE_CFS, 0x6A);  // 1. double taps only on all axes
  writeByte(MMA8652_ADDRESS, PULSE_THSX, 0x04);  // 2. x thresh at 0.25g, multiply the value by 0.0625g/LSB to get the threshold
  writeByte(MMA8652_ADDRESS, PULSE_THSY, 0x04);  // 2. y thresh at 0.25g, multiply the value by 0.0625g/LSB to get the threshold
  writeByte(MMA8652_ADDRESS, PULSE_THSZ, 0x04);  // 2. z thresh at 0.25g, multiply the value by 0.0625g/LSB to get the threshold
  writeByte(MMA8652_ADDRESS, PULSE_TMLT, 0x30);  // 3. 2.55s time limit at 100Hz odr, this is very dependent on data rate, see the app note
  writeByte(MMA8652_ADDRESS, PULSE_LTCY, 0xA0);  // 4. 5.1s 100Hz odr between taps min, this also depends on the data rate
  writeByte(MMA8652_ADDRESS, PULSE_WIND, 0xFF);  // 5. 10.2s (max value)  at 100 Hz between taps max

  // Set up motion detection
  writeByte(MMA8652_ADDRESS, FF_MT_CFG, 0x58); // Set motion flag on x and y axes
  writeByte(MMA8652_ADDRESS, FF_MT_THS, 0x84); // Clear debounce counter when condition no longer obtains, set threshold to 0.25 g
  writeByte(MMA8652_ADDRESS, FF_MT_COUNT, 0x8); // Set debounce to 0.08 s at 100 Hz

  // Set up interrupt 1 and 2
  writeByte(MMA8652_ADDRESS, CTRL_REG3, readByte(MMA8652_ADDRESS, CTRL_REG3) & ~(0x02)); // clear bits 0, 1 
  writeByte(MMA8652_ADDRESS, CTRL_REG3, readByte(MMA8652_ADDRESS, CTRL_REG3) |  (0x02)); // select ACTIVE HIGH, push-pull interrupts
     
 // writeByte(MMA8652_ADDRESS, 0x2C, 0x02);  // Active high, push-pull interrupts

  writeByte(MMA8652_ADDRESS, CTRL_REG4, readByte(MMA8652_ADDRESS, CTRL_REG4) & ~(0x1D)); // clear bits 0, 3, and 4
  writeByte(MMA8652_ADDRESS, CTRL_REG4, readByte(MMA8652_ADDRESS, CTRL_REG4) |  (0x1D)); // DRDY, Freefall/Motion, P/L and tap ints enabled
   
  writeByte(MMA8652_ADDRESS, CTRL_REG5, 0x01);  // DRDY on INT1, P/L and taps on INT2

  MMA8652Active();  // Set to active to start reading
}

// Sets the MMA8652 to standby mode.
// It must be in standby to change most register settings
void MMA8652Standby()
{
  byte c = readByte(MMA8652_ADDRESS, 0x2A);
  writeByte(MMA8652_ADDRESS, CTRL_REG1, c & ~(0x01));
}

// Sets the MMA8652 to active mode.
// Needs to be in this mode to output data
void MMA8652Active()
{
  byte c = readByte(MMA8652_ADDRESS, 0x2A);
  writeByte(MMA8652_ADDRESS, CTRL_REG1, c | 0x01);
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
