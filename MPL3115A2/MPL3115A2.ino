/* MPL3115A2 Example Code
 by: Kris Winer adapted from skeleton codes by Jim Lindblom, A. Weiss, and Nathan Seidle (Thanks guys!)
 Modified: February 12, 2014 by Kris Winer
 Original date: November 17, 2011, Jim Lindblom, SparkFun Electronics
 
 IDE example usage for most features of the MPL3115A2 I2C Precision Altimeter.
 
 Basic functions are implemented including absolute pressure (50 to 110 kPa), altimeter pressure (mmHg), 
 altitude (meters or feet), and temperature (-40 to 85 C).
 
 In addition, provision for the FIFO mode in the initialization, watermark/overflow setting, and register
 read functions for autonomous data logging over as many as nine hours with 32 data samples of P, T.
 
 Hardware setup:
 MPL3115A2 Breakout ------------ Arduino Mini Pro 3.3 V
 3.3V --------------------- 3.3V
 SDA ----------------------- A4
 SCL ----------------------- A5
 INT2 ---------------------- D4
 INT1 ---------------------- D5
 GND ---------------------- GND
 
 SDA and SCL should have external pull-up resistors (to 3.3V) if using a 5 V Arduino.
 They should be on the MPL3115A2 SparkFun breakout board.
 I didn't need any for the 3.3 V Pro Mini.
 
 Jim Lindblom's Note: The MMA8452 is an I2C sensor, however this code does
 not make use of the Arduino Wire library. Because the sensor
 is not 5V tolerant, we can't use the internal pull-ups used
 by the Wire library. Instead use the included i2c.h, defs.h and types.h files.
 
 The MPL3115A2 in this sketch is interfaced with a Pro Mini operating at 3.3 V, so we could use the
 Arduino Wire library. However, I will follow Jim Lindblom's example.
 
 The MPL3115A2 has internal First in/First out data storage enabling autonomous data logging for up to 32 samples
 of pressure/altitude and temperature. I programmed the Pro Mini through a FTDI Basic interface board and noticed
 every time I hooked it up to the Arduino/Sensor or opened a serial monitor the devices were re-initialized, 
 defeating the autonomous logging function.

 This was remedied by placing a 10 uF capacitor between reset and ground before either reconnecting the sensor 
 through the FTDI board or opening a serial monitor. Either event drives the reset low, the capacitor keeps the
 reset high long enough to avoid resetting. Of course, the capacitor must be removed when uploading a new or updated 
 sketch or an error will be generated.  

 I got this idea from:
 http://electronics.stackexchange.com/questions/24743/arduino-resetting-while-reconnecting-the-serial-terminal
 where there is a little more discussion. Thanks 0xAKHIL!
 
 Lastly, I put a piece of porous foam over the sensor to block ambient light since thre is some indication the 
 pressure and altitude reading are light sensitive.
 */

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>

// Using NOKIA 5110 monochrome 84 x 48 pixel display
// pin 9 - Serial clock out (SCLK)
// pin 8 - Serial data out (DIN)
// pin 7 - Data/Command select (D/C)
// pin 5 - LCD chip select (CS)
// pin 6 - LCD reset (RST)
Adafruit_PCD8544 display = Adafruit_PCD8544(9, 8, 7, 5, 6);

// Standard 7-bit I2C slave address is 1100000 = 0x60, 8-bit read address is 0xC1, 8-bit write is 0xC0
#define MPL3115A2_ADDRESS 0x60  // SA0 is high, 0x1C if low

// Register defines courtesy A. Weiss and Nathan Seidle, SparkFun Electronics
#define STATUS     0x00
#define OUT_P_MSB  0x01
#define OUT_P_CSB  0x02
#define OUT_P_LSB  0x03
#define OUT_T_MSB  0x04
#define OUT_T_LSB  0x05
#define DR_STATUS  0x06
#define OUT_P_DELTA_MSB  0x07
#define OUT_P_DELTA_CSB  0x08
#define OUT_P_DELTA_LSB  0x09
#define OUT_T_DELTA_MSB  0x0A
#define OUT_T_DELTA_LSB  0x0B
#define WHO_AM_I   0x0C
#define F_STATUS   0x0D
#define F_DATA     0x0E
#define F_SETUP    0x0F
#define TIME_DLY   0x10
#define SYSMOD     0x11
#define INT_SOURCE 0x12
#define PT_DATA_CFG 0x13
#define BAR_IN_MSB 0x14 // Set at factory to equivalent sea level pressure for measurement location, generally no need to change
#define BAR_IN_LSB 0x15 // Set at factory to equivalent sea level pressure for measurement location, generally no need to change
#define P_TGT_MSB  0x16
#define P_TGT_LSB  0x17
#define T_TGT      0x18
#define P_WND_MSB  0x19
#define P_WND_LSB  0x1A
#define T_WND      0x1B
#define P_MIN_MSB  0x1C
#define P_MIN_CSB  0x1D
#define P_MIN_LSB  0x1E
#define T_MIN_MSB  0x1F
#define T_MIN_LSB  0x20
#define P_MAX_MSB  0x21
#define P_MAX_CSB  0x22
#define P_MAX_LSB  0x23
#define T_MAX_MSB  0x24
#define T_MAX_LSB  0x25
#define CTRL_REG1  0x26
#define CTRL_REG2  0x27
#define CTRL_REG3  0x28
#define CTRL_REG4  0x29
#define CTRL_REG5  0x2A
#define OFF_P      0x2B
#define OFF_T      0x2C
#define OFF_H      0x2D

enum SAMPLERATE {
  OS_6ms = 0, // 6 ms is minimum oversampling interval, corresponds to an oversample ration of 2^0 = 1 
  OS_10ms,
  OS_18ms,
  OS_34ms,
  OS_66ms,
  OS_130ms, // 130 ms oversampling interval, 2^5 = 32 oversample ratio
  OS_258ms,
  OS_512ms
};

enum ST_VALUE {
  ATS_1s = 0, // 6 ms is minimum oversampling interval, corresponds to an oversample ration of 2^0 = 1 
  ATS_2s,
  ATS_4s,
  ATS_8s,
  ATS_16s,
  ATS_32s,
  ATS_64s, // 2^6 = 64 s interval between up to 32 FIFO samples for half an hour of data logging
  ATS_128s,
  ATS_256s,
  ATS_512s,
  ATS_1024s,
  ATS_2048s,
  ATS_4096s,
  ATS_8192s,
  ATS_16384s,
  ATS_32768s  // 2^15 = 32768 s interval between up to 32 FIFO samples = 12 days of data logging!
};

uint8_t SAMPLERATE = OS_130ms;
uint8_t ST_VALUE = ATS_4s;
int AltimeterMode = 0; // use to choose between altimeter and barometer modes for FIFO data
uint32_t deltat = 0;
uint32_t count = 0;

// Define device outputs
float altitude = 0.;
float pressure = 0.;
float temperature = 0.;

void setup()
{

  Wire.begin();
  Serial.begin(38400);
      
  display.begin(); // Initialize the display
  display.setContrast(58); // Set the contrast
  display.setRotation(2); //  0 or 2) width = width, 1 or 3) width = height, swapped etc.

// Start device display with ID of sensor
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0,0); display.print("M3115A2");
  display.setTextSize(1);
  display.setCursor(0, 20); display.print("20-bit");
  display.setCursor(0, 30); display.print("pressure");
  display.setCursor(0, 40); display.print("sensor");
  display.setCursor(20,50); display.print("altimeter");
  display.display();
  delay(1000);
  
  // Set up for data display
  display.setTextSize(1); // Set text size to normal, 2 is twice normal etc.
  display.setTextColor(BLACK); // Set pixel color; 1 on the monochrome screen
  display.clearDisplay();   // clears the screen and buffer

  // Read the WHO_AM_I register, this is a good test of communication
  uint8_t c = readByte(MPL3115A2_ADDRESS, WHO_AM_I);  // Read WHO_AM_I register
  display.setCursor(0, 0); display.print("MPL3115A2");
  display.setCursor(0,10); display.print("I AM");
  display.setCursor(0,20); display.print(c, HEX);  
  display.setCursor(0,30); display.print("I Should Be");
  display.setCursor(0,40); display.print(0xC4, HEX); 
  display.display();
  delay(1000); 
  
  if (c == 0xC4) // WHO_AM_I should always be 0xC4
  {  
    
    MPL3115A2Reset();                // Start off by resetting all registers to the default
    initRealTimeMPL3115A2();         // initialize the accelerometer for realtime data acquisition if communication is OK
    MPL3115A2SampleRate(SAMPLERATE); // Set oversampling ratio
    Serial.print("Oversampling Ratio is "); Serial.print(1<<SAMPLERATE);  
    MPL3115A2enableEventflags();     // Set data ready enable
    Serial.println("MPL3115A2 event flags enabled...");

    display.clearDisplay();   // clears the screen and buffer
    display.setCursor(0, 0); display.print("MPL3115A2");
    display.setCursor(0,10); display.print("OSR "); display.print(1<<SAMPLERATE);  
    display.setCursor(0,20); display.print("Event flags");
    display.setCursor(0,30); display.print("enabled...");
    display.display();
    delay(1000);
  }
  else
  {
    Serial.print("Could not connect to MPL3115A2: 0x");
    Serial.println(c, HEX);
    display.clearDisplay();   // clears the screen and buffer
    display.setCursor(0, 0); display.print("MPL3115A2");
    display.setCursor(0,10); display.print("Error! on 0x"); display.print(c, HEX);
    display.display();
    
    while(1) ; // Loop forever if communication doesn't happen
  }
}

void loop()
{  
    Clock();
    MPL3115A2ActiveAltimeterMode(); 
    MPL3115A2readAltitude();  // Read the altitude

    MPL3115A2ActiveBarometerMode(); 
    MPL3115A2readPressure();  // Read the pressure
    
    const int station_elevation_m = 1050.0*0.3048; // Accurate for the roof on my house; convert from feet to meters

    float baroin = pressure/100; // pressure is now in millibars

    // Formula to correct absolute pressure in millbars to "altimeter pressure" in inches of mercury 
    // comparable to weather report pressure
    float part1 = baroin - 0.3; //Part 1 of formula
    const float part2 = 0.0000842288;
    float part3 = pow(part1, 0.190284);
    float part4 = (float)station_elevation_m / part3;
    float part5 = (1.0 + (part2 * part4));
    float part6 = pow(part5, 5.2553026);
    float altimeter_setting_pressure_mb = part1 * part6; // Output is now in adjusted millibars
    baroin = altimeter_setting_pressure_mb * 0.02953;

    deltat = millis() - count;
    if(deltat > 1000) { // Display outputs every 1000 ms to the LCD
    
    Serial.print("pressure is "); Serial.print(pressure, 2); Serial.println(" Pa");  // Print altitude in meters
    Serial.print("altitude is "); Serial.print(altitude, 2);  Serial.println(" m"); // Print altitude in meters   
    Serial.print("  temperature is "); Serial.print(temperature, 2);  Serial.println(" C"); // Print temperature in C
    
    display.clearDisplay();   // clears the screen and buffer
    display.setCursor(10, 0); display.print("MPL3115A2");
    display.setCursor(0,10); display.print("P   "); display.setCursor(30,10); display.print(pressure/1000., 2);    display.print(" kPa");
    display.setCursor(0,20); display.print("Alt "); display.setCursor(24,20); display.print(altitude, 1);    display.print("    m");
    display.setCursor(0,30); display.print("Alt "); display.setCursor(24,30); display.print(3.28084*altitude, 1);  display.print("   ft");
    display.setCursor(0,40); display.print("T   "); display.setCursor(30,40); display.print(temperature, 1); display.print("    C");
    display.display();
   
    count = millis();
    }

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Some useful functions
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

void MPL3115A2readAltitude() // Get altitude in meters and temperature in centigrade
{
  uint8_t rawData[5];  // msb/csb/lsb pressure and msb/lsb temperature stored in five contiguous registers 

// We can read the data either by polling or interrupt; see data sheet for relative advantages
// First we try hardware interrupt, which should take less power, etc.
// while (digitalRead(int1Pin) == LOW); // Wait for interrupt pin int1Pin to go HIGH
// digitalWrite(int1Pin, LOW);  // Reset interrupt pin int1Pin
 while((readByte(MPL3115A2_ADDRESS, INT_SOURCE) & 0x80) == 0); // Check that the interrupt source is a data ready interrupt
// or use a polling method
// Check data read status; if PTDR (bit 4) not set, then
// toggle OST bit to cause sensor to immediately take a reading
// Setting the one shot toggle is the way to get faster than 1 Hz data read rates
// while ((readByte(MPL3115A2_ADDRESS, STATUS) & 0x08) == 0);  MPL3115A2toggleOneShot(); 
  
  readBytes(MPL3115A2_ADDRESS, OUT_P_MSB, 5, &rawData[0]);  // Read the five raw data registers into data array

// Altutude bytes-whole altitude contained defined by msb, csb, and first two bits of lsb, fraction by next two bits of lsb
  uint8_t msbA = rawData[0];
  uint8_t csbA = rawData[1];
  uint8_t lsbA = rawData[2];
// Temperature bytes
  uint8_t msbT = rawData[3];
  uint8_t lsbT = rawData[4];
 
 // Calculate altitude, check for negative sign in altimeter data
 long foo = 0;
 if(msbA > 0x7F) {
   foo = ~((long)msbA << 16 | (long)csbA << 8 | (long)lsbA) + 1; // 2's complement the data
   altitude = (float) (foo >> 8) + (float) ((lsbA >> 4)/16.0); // Whole number plus fraction altitude in meters for negative altitude
   altitude *= -1.;
 }
 else {
   altitude = (float) ( (msbA << 8) | csbA) + (float) ((lsbA >> 4)/16.0);  // Whole number plus fraction altitude in meters
 }

// Calculate temperature, check for negative sign
if(msbT > 0x7F) {
 foo = ~(msbT << 8 | lsbT) + 1 ; // 2's complement
 temperature = (float) (foo >> 8) + (float)((lsbT >> 4)/16.0); // add whole and fractional degrees Centigrade
 temperature *= -1.;
 }
 else {
   temperature = (float) (msbT) + (float)((lsbT >> 4)/16.0); // add whole and fractional degrees Centigrade
 }
}

void MPL3115A2readPressure()
{
  uint8_t  rawData[5];  // msb/csb/lsb pressure and msb/lsb temperature stored in five contiguous registers

// We can read the data either by polling or interrupt; see data sheet for relative advantages
// First we try hardware interrupt, which should take less power, etc.
// while (digitalRead(int1Pin) == LOW); // Wait for interrupt pin int1Pin to go HIGH
// digitalWrite(int1Pin, LOW);  // Reset interrupt pin int1Pin
 while((readByte(MPL3115A2_ADDRESS, INT_SOURCE) & 0x80) == 0); // Check that the interrupt source is a data ready interrupt
// or use a polling method
// Check data read status; if PTDR (bit 4) not set, then
// toggle OST bit to cause sensor to immediately take a reading
// Setting the one shot toggle is the way to get faster than 1 Hz data read rates
 //while ((readByte(MPL3115A2_ADDRESS, STATUS) & 0x08) == 0);  MPL3115A2toggleOneShot(); 
 
  readBytes(MPL3115A2_ADDRESS, OUT_P_MSB, 5, &rawData[0]);  // Read the five raw data registers into data array

// Pressure bytes
  uint8_t msbP = rawData[0];
  uint8_t csbP = rawData[1];
  uint8_t lsbP = rawData[2];
// Temperature bytes
  uint8_t msbT = rawData[3];
  uint8_t lsbT = rawData[4]; 
 
  long pressure_whole =   ((long)msbP << 16 |  (long)csbP << 8 |  (long)lsbP) ; // Construct whole number pressure
  pressure_whole >>= 6; // Only two most significant bits of lsbP contribute to whole pressure; its an 18-bit number
 
  lsbP &= 0x30; // Keep only bits 5 and 6, the fractional pressure
  lsbP >>= 4; // Shift to get the fractional pressure in terms of quarters of a Pascal
  float pressure_frac = (float) lsbP/4.0; // Convert numbers of fractional quarters to fractional pressure n Pasacl

  pressure = (float) (pressure_whole) + pressure_frac; // Combine whole and fractional parts to get entire pressure in Pascal

// Calculate temperature, check for negative sign
long foo = 0;
if(msbT > 0x7F) { // Is the most significant bit a 1? Then its a negative number in two's complement form
 foo = ~(msbT << 8 | lsbT) + 1 ; // 2's complement
 temperature = (float) (foo >> 8) + (float)((lsbT >> 4)/16.0); // add whole and fractional degrees Centigrade
 temperature *= -1.;
 }
 else {
   temperature = (float) (msbT) + (float)((lsbT >> 4)/16.0); // add whole and fractional degrees Centigrade
 }
}

/*
=====================================================================================================
Define functions according to 
"Data Manipulation and Basic Settings of the MPL3115A2 Command Line Interface Drive Code"
by Miguel Salhuana
Freescale Semiconductor Application Note AN4519 Rev 0.1, 08/2012
=====================================================================================================
*/
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Clears then sets OST bit which causes the sensor to immediately take another reading
void MPL3115A2toggleOneShot()
{
    MPL3115A2Active();  // Set to active to start reading
    uint8_t c = readByte(MPL3115A2_ADDRESS, CTRL_REG1);
    writeByte(MPL3115A2_ADDRESS, CTRL_REG1, c & ~(1<<1)); // Clear OST (bit 1)
    c = readByte(MPL3115A2_ADDRESS, CTRL_REG1);
    writeByte(MPL3115A2_ADDRESS, CTRL_REG1, c | (1<<1)); // Set OST bit to 1
}
    
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Set the Outputting Sample Rate
void MPL3115A2SampleRate(uint8_t samplerate)
{
  MPL3115A2Standby();  // Must be in standby to change registers

  uint8_t c = readByte(MPL3115A2_ADDRESS, CTRL_REG1);
  writeByte(MPL3115A2_ADDRESS, CTRL_REG1, c & ~(0x38)); // Clear OSR bits 3,4,5
  if(samplerate < 8) { // OSR between 1 and 7
  writeByte(MPL3115A2_ADDRESS, CTRL_REG1, c | (samplerate << 3));  // Write OSR to bits 3,4,5
  }
  
  MPL3115A2Active();  // Set to active to start reading
 }
 
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Initialize the MPL3115A2 registers for FIFO mode
void initFIFOMPL3115A2()
{
  // Clear all interrupts by reading the data output registers
  uint8_t temp;
  temp = readByte(MPL3115A2_ADDRESS, OUT_P_MSB);
  temp = readByte(MPL3115A2_ADDRESS, OUT_P_CSB);
  temp = readByte(MPL3115A2_ADDRESS, OUT_P_LSB);
  temp = readByte(MPL3115A2_ADDRESS, OUT_T_MSB);
  temp = readByte(MPL3115A2_ADDRESS, OUT_T_LSB);
  temp = readByte(MPL3115A2_ADDRESS, F_STATUS);
  
   MPL3115A2Standby();  // Must be in standby to change registers
  
  // Set CTRL_REG4 register to configure interupt enable
  // Enable data ready interrupt (bit 7), enable FIFO (bit 6), enable pressure window (bit 5), temperature window (bit 4),
  // pressure threshold (bit 3), temperature threshold (bit 2), pressure change (bit 1) and temperature change (bit 0)
  writeByte(MPL3115A2_ADDRESS, CTRL_REG4, 0x40);  // enable FIFO
  
  //  Configure INT 1 for data ready, all other (inc. FIFO) interrupts to INT2
  writeByte(MPL3115A2_ADDRESS, CTRL_REG5, 0x80); 
  
  // Set CTRL_REG3 register to configure interupt signal type
  // Active HIGH, push-pull interupts INT1 and INT 2
  writeByte(MPL3115A2_ADDRESS, CTRL_REG3, 0x22); 
  
  // Set FIFO mode
  writeByte(MPL3115A2_ADDRESS, F_SETUP, 0x00); // Clear FIFO mode
// In overflow mode, when FIFO fills up, no more data is taken until the FIFO registers are read
// In watermark mode, the oldest data is overwritten by new data until the FIFO registers are read
  writeByte(MPL3115A2_ADDRESS, F_SETUP, 0x80); // Set F_MODE to interrupt when overflow = 32 reached
//  writeByte(MPL3115A2_ADDRESS, F_SETUP, 0x60); // Set F_MODE to accept 32 data samples and interrupt when watermark = 32 reached

  MPL3115A2Active();  // Set to active to start reading
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Initialize the MPL3115A2 for realtime data collection 
void initRealTimeMPL3115A2()
{
  // Clear all interrupts by reading the data output registers
  uint8_t temp;
  temp = readByte(MPL3115A2_ADDRESS, OUT_P_MSB);
  temp = readByte(MPL3115A2_ADDRESS, OUT_P_CSB);
  temp = readByte(MPL3115A2_ADDRESS, OUT_P_LSB);
  temp = readByte(MPL3115A2_ADDRESS, OUT_T_MSB);
  temp = readByte(MPL3115A2_ADDRESS, OUT_T_LSB);
  temp = readByte(MPL3115A2_ADDRESS, F_STATUS);
  
   MPL3115A2Standby();  // Must be in standby to change registers
  
  // Set CTRL_REG4 register to configure interupt enable
  // Enable data ready interrupt (bit 7), enable FIFO (bit 6), enable pressure window (bit 5), temperature window (bit 4),
  // pressure threshold (bit 3), temperature threshold (bit 2), pressure change (bit 1) and temperature change (bit 0)
  writeByte(MPL3115A2_ADDRESS, CTRL_REG4, 0x80);  
  
  //  Configure INT 1 for data ready, all other interrupts to INT2
  writeByte(MPL3115A2_ADDRESS, CTRL_REG5, 0x80); 
  
  // Set CTRL_REG3 register to configure interupt signal type
  // Active HIGH, push-pull interupts INT1 and INT 2
  writeByte(MPL3115A2_ADDRESS, CTRL_REG3, 0x22); 
  
  // Set FIFO mode
  writeByte(MPL3115A2_ADDRESS, F_SETUP, 0x00); // disable FIFO mode
  
  MPL3115A2Active();  // Set to active to start reading
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Set the Auto Acquisition Time Step
void MPL3115A2TimeStep(uint8_t ST_Value)
{
 MPL3115A2Standby(); // First put device in standby mode to allow write to registers
 
 uint8_t c = readByte(MPL3115A2_ADDRESS, CTRL_REG2); // Read contents of register CTRL_REG2
 if (ST_Value <= 0xF) {
 writeByte(MPL3115A2_ADDRESS, CTRL_REG2, (c | ST_Value)); // Set time step n from 0x0 to 0xF (bits 0 - 3) for time intervals from 1 to 32768 (2^n) seconds
 }
 
 MPL3115A2Active(); // Set to active to start reading
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Enable the pressure and temperature event flags
 // Bit 2 is general data ready event mode on new Pressure/Altitude or temperature data
 // Bit 1 is event flag on new Pressure/Altitude data
 // Bit 0 is event flag on new Temperature data
void MPL3115A2enableEventflags()
{
  MPL3115A2Standby();  // Must be in standby to change registers
  writeByte(MPL3115A2_ADDRESS, PT_DATA_CFG, 0x07); //Enable all three pressure and temperature event flags
  MPL3115A2Active();  // Set to active to start reading
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Enter Active Altimeter mode
void MPL3115A2ActiveAltimeterMode()
{
 MPL3115A2Standby(); // First put device in standby mode to allow write to registers
 uint8_t c = readByte(MPL3115A2_ADDRESS, CTRL_REG1); // Read contents of register CTRL_REG1
 writeByte(MPL3115A2_ADDRESS, CTRL_REG1, c | (0x80)); // Set ALT (bit 7) to 1
 MPL3115A2Active(); // Set to active to start reading
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Enter Active Barometer mode
void MPL3115A2ActiveBarometerMode()
{
 MPL3115A2Standby(); // First put device in standby mode to allow write to registers
 uint8_t c = readByte(MPL3115A2_ADDRESS, CTRL_REG1); // Read contents of register CTRL_REG1
 writeByte(MPL3115A2_ADDRESS, CTRL_REG1, c & ~(0x80)); // Set ALT (bit 7) to 0
 MPL3115A2Active(); // Set to active to start reading
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Software resets the MPL3115A2.
// It must be in standby to change most register settings
void MPL3115A2Reset()
{
  writeByte(MPL3115A2_ADDRESS, CTRL_REG1, 0x04); // Set RST (bit 2) to 1
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Sets the MPL3115A2 to standby mode.
// It must be in standby to change most register settings
void MPL3115A2Standby()
{
  uint8_t c = readByte(MPL3115A2_ADDRESS, CTRL_REG1); // Read contents of register CTRL_REG1
  writeByte(MPL3115A2_ADDRESS, CTRL_REG1, c & ~(0x01)); // Set SBYB (bit 0) to 0
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Sets the MPL3115A2 to active mode.
// Needs to be in this mode to output data
void MPL3115A2Active()
{
  uint8_t c = readByte(MPL3115A2_ADDRESS, CTRL_REG1); // Read contents of register CTRL_REG1
  writeByte(MPL3115A2_ADDRESS, CTRL_REG1, c | 0x01); // Set SBYB (bit 0) to 1
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Poor man's system clock that must be set to the correct time upon start of the program
// A real time clock on board the circuit would be better
void Clock()
{
  //this function prints the current time
//  lcd.clear(); // start by clearing the LCD

  // Initialize clock

  // set up some variables for the clock function // Define variables for seconds, minutes, and hours 

  unsigned long seconds = 0;
  unsigned long minutes = 0;
  unsigned long hours   = 0;
  // Define variables to set initial values for seconds, minutes, and hours 
  unsigned long setseconds = 30; // set clock seconds 
  unsigned long setminutes = 3; // set clock minutes
  unsigned long sethours = 5; // set clock hours
  // Define variables to set alarm values for seconds, minutes, and hours 
  unsigned long tseconds = 0; // set clock seconds 
  unsigned long tminutes = 0; // set clock minutes
  unsigned long thours   = 0; // set clock hours

  // Set the time units using the millisecond utility 
  // Remember to add the fractions of a minute (setseconds/60) to the minute tally and 
  // the fractions of an hour (setseconds/3600 + setminutes/60) to the hour tally.

  seconds = setseconds + (millis()/1000); // 1 second = 1000 milliseconds
  minutes = setminutes + ((millis() + setseconds*1000)/60000); // 1 minute = 60,000 milliseconds
  hours   = sethours   + ((millis() + setseconds*1000 + setminutes*60000)/3600000); // 1 hour = 3,600,000 milliseconds

  // print colons separating hours, minutes, and seconds

//  lcd.setCursor(2,0);
 // lcd.print(":");
 // lcd.setCursor(5,0);
 // lcd.print(":");

  // Check if the seconds/minutes are greater than 60, // if so, truncate to less than 60; // Check if the hours are greater than 12, if so truncate to // less than 12.

  while(seconds >= 60) {
    seconds = seconds - 60;
  } // end while

  while(minutes >= 60) {
    minutes = minutes - 60;
  } // end while

  while(hours > 12) {
    hours = hours - 12;
  } // end while

  // If seconds less than 10, print a zero in the tens place and seconds thereafter

  if(seconds <= 9) {
 //   lcd.setCursor(6,0);
 //   lcd.print("0");
 //   lcd.setCursor(7,0);
 //   lcd.print(seconds);
  }

  // Otherwise, if seconds greater than 9 print

  else {
 //   lcd.setCursor(6,0);
 //   lcd.print(seconds);
  }

  // If minutes less than 10, print a zero in the tens place and minutes thereafter

  if(minutes <= 9) {
 //   lcd.setCursor(3,0);
 //   lcd.print("0");
 //   lcd.setCursor(4,0);
 //   lcd.print(minutes);
  }

  // Otherwise, if minutes greater than 9 print

  else {
 //   lcd.setCursor(3,0);
 //   lcd.print(minutes);
  }

  // If hours less than 10, print a zero in the tens place and hours thereafter

  if(hours <= 9) {
 //   lcd.setCursor(0,0);
 //   lcd.print("0");
 //   lcd.setCursor(1,0);
 //   lcd.print(hours);
  }

  // Otherwise, if hours greater than 9 print

  else {
 //   lcd.setCursor(0,0);
 //   lcd.print(hours);
  }

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

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// End of Sketch MPL3115A2
