 /* MAG3110 Example Code
 Original sketch by: Kris Winer, May 27, 2014 
 
 Includes reset, magnetometer initialization and calibration, as well as parameterizing the register addresses. 
 Added LCD functions to allow data display to on-breadboard monitor.
 
 license: Beerware - Use this code however you'd like. If you 
 find it useful you can buy me a beer some time.
 
 This code should provide example usage for most features of
 the MAG3110 3-axis, I2C 16-bit magnetometer. In the loop function
 the magnetoometer interrupt outputs will be polled, and either
 the x/y/z mag data will be output, or magnetic threshold detection will be displayed.
 
 Hardware setup:
 MAG3110 Breakout --------- Arduino
 3.3V --------------------- 3.3V
 SDA ----------------------- A4
 SCL ----------------------- A5
 INT2 ---------------------- D5
 INT1 ---------------------- D4
 GND ---------------------- GND
 
 SDA and SCL should have external pull-up resistors (to 3.3V).
 10k resistors are on the FRDM-FXS-MULTI breakout board.
 
 Note: The MAG3110 is an I2C sensor; here we make use of the Arduino Wire library.Because the sensor is not 5V tolerant, we are using a 3.3 V 8 MHz Pro Mini or a 3.3 V Teensy 3.1.
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

// Define registers per MAG3110, Document Number: MAG3110FC
// Data Sheet: Technical Data Rev. 2.0, 02/2013 3-Axis, 12-bit/8-bit Digital Accelerometer
// Freescale Semiconductor Data Sheet
#define DR_STATUS        0x00
#define OUT_X_MSB        0x01    
#define OUT_X_LSB        0x02
#define OUT_Y_MSB        0x03
#define OUT_Y_LSB        0x04
#define OUT_Z_MSB        0x05
#define OUT_Z_LSB        0x06
#define WHO_AM_I         0x07   
#define SYSMOD           0x08
#define OFF_X_MSB        0x09    
#define OFF_X_LSB        0x0A
#define OFF_Y_MSB        0x0B
#define OFF_Y_LSB        0x0C
#define OFF_Z_MSB        0x0D
#define OFF_Z_LSB        0x0E
#define DIE_TEMP         0x0F
#define CTRL_REG1        0x10
#define CTRL_REG2        0x11

#define SA0 0
#if SA0
#define MAG3110_ADDRESS 0xoF  // SA0 is high, 0xOE if low
#else
#define MAG3110_ADDRESS 0x0E
#endif

// Set initial input parameters
enum magODR {
  mODR_80Hz_16os = 0,  // 80 Hz data output rate with 16 times oversampling (1280 Hz is max ADC rate)
  mODR_40Hz_32os,
  mODR_20Hz_64os,
  mODR_10Hz_128os,
  mODR_40Hz_16os,  
  mODR_20Hz_32os,
  mODR_10Hz_64os,
  mODR_5Hz_128os,
  mODR_20Hz_16os,  
  mODR_10Hz_32os,
  mODR_5Hz_64os,
  mODR_2_5Hz_128os,
  mODR_10Hz_16os,  
  mODR_5Hz_32os,
  mODR_2_5Hz_64os,
  mODR_1_25Hz_128os,
  mODR_5Hz_16os,  
  mODR_2_5Hz_32os,
  mODR_1_25Hz_64os,
  mODR_0_63Hz_128os,
  mODR_2_5Hz_16os,  
  mODR_1_25Hz_32os,
  mODR_0_63Hz_64os,
  mODR_0_31Hz_128os,
  mODR_1_25Hz_16os,  
  mODR_0_63Hz_32os,
  mODR_0_31Hz_64os,
  mODR_0_16Hz_128os,
  mODR_0_63Hz_16os,  
  mODR_0_31Hz_32os,
  mODR_0_11Hz_64os,
  mODR_0_08Hz_128os
};

// Specify sensor sample data rate and oversampling
uint8_t magODR = mODR_80Hz_16os;

// Pin definitions
int ledPin  = 13;  // Pro Mini led

int16_t magCount[3];  // Stores the 12-bit signed value
float mx, my, mz;       // Stores the real accel value in g's
int8_t tempCount;
float temperature;
uint32_t count = 0;

void setup()
{
  Wire.begin();
  Serial.begin(38400);
 
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
  
  display.begin(); // Initialize the display
  display.setContrast(58); // Set the contrast
  display.setRotation(2); //  0 or 2) width = width, 1 or 3) width = height, swapped etc.

  
// Start device display with ID of sensor
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0,0); display.print("MAG3110");
  display.setTextSize(1);
  display.setCursor(0, 20); display.print("3-axis 16-bit");
  display.setCursor(0, 30); display.print("magnetometer");
  display.setCursor(0, 40); display.print("1 mGauss LSB");
  display.display();
  delay(1000);

// Set up for data display
  display.setTextSize(1); // Set text size to normal, 2 is twice normal etc.
  display.setTextColor(BLACK); // Set pixel color; 1 on the monochrome screen
  display.clearDisplay();   // clears the screen and buffer
  
  // Read the WHO_AM_I register, this is a good test of communication
  byte c = readByte(MAG3110_ADDRESS, WHO_AM_I);  // Read WHO_AM_I register
  display.clearDisplay();
  display.setCursor(0,0); display.print("MAG3110");  
  display.setCursor(0,10); display.print("I Am");
  display.setCursor(0, 20); display.print("Ox");display.print(c, HEX);  
  display.setCursor(0, 30); display.print("I Should be"); 
  display.setCursor(0, 40); display.print("Ox");display.print(0xC4, HEX);  
  display.display();
  delay(1000);

  if (c == 0xC4) // WHO_AM_I should always be 0x4A
  {  
    MAG3110Reset();  // Start by resetting sensor device to default settings
    MAG3110Offsets(); // Apply user offsets
    initMAG3110();   // init the accelerometer if communication is OK
    Serial.println("MAG3110 is online...");
  }
  else
  {
    Serial.print("Could not connect to MAG3110: 0x");
    Serial.println(c, HEX);
    while(1) ; // Loop forever if communication doesn't happen
  }
}

void loop()
{  
  // One can use the interrupt pins to detect a data ready condition; here we just check the STATUS register for a data ready bit
  if(readByte(MAG3110_ADDRESS, DR_STATUS) & 0x08)  { // When this bit set, all axes have new data
 
    readMagData(magCount);               // Read the x/y/z adc values
    mx = (float)magCount[0]*10./32768.;  // get actual Gauss value 
    my = (float)magCount[1]*10./32768.;   
    mz = (float)magCount[2]*10./32768.;  
    

    tempCount = readTempData();  // Read the x/y/z adc values
    temperature = (float) tempCount + 26.; // Temperature in degrees Centigrade
  }   
    uint32_t deltat = millis() - count;
   if (deltat > 500) { // update LCD once per half-second independent of read rate
 
    // Print out values
    Serial.print("x-magnetic field = "); Serial.print(1000.*mx); Serial.print(" mG");   
    Serial.print("y-magnetic field = "); Serial.print(1000.*my); Serial.print(" mG");   
    Serial.print("z-magnetic field = "); Serial.print(1000.*mz); Serial.print(" mG");  

    display.clearDisplay();    
    display.setCursor(0, 0); display.print("MAG3110");
    display.setCursor(0, 8); display.print(" x ");  display.print((int)(1000*mx)); 
    display.setCursor(43, 8); display.print(" mG");
    display.setCursor(0, 16); display.print(" y "); display.print((int)(1000*my)); 
    display.setCursor(43, 16); display.print(" mG");
    display.setCursor(0, 24); display.print(" z "); display.print((int)(1000*mz)); 
    display.setCursor(43, 24); display.print(" mG");
    display.setCursor(0, 32); display.print(" T "); display.print(temperature, 1); 
    display.setCursor(43, 32); display.print(" C");
    display.display();
    
    count = millis();
    digitalWrite(ledPin, !digitalRead(ledPin));
   }
}


void readMagData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  readBytes(MAG3110_ADDRESS, OUT_X_MSB, 6, &rawData[0]);  // Read the six raw data registers into data array
  destination[0] = ((int16_t) rawData[0] << 8 | rawData[1]);
  destination[1] = ((int16_t) rawData[2] << 8 | rawData[3]);
  destination[2] = ((int16_t) rawData[4] << 8 | rawData[5]);
}

int8_t readTempData()
{
  return (int8_t) readByte(MAG3110_ADDRESS, DIE_TEMP);  // Read the 8-bit 2's complement data register 
}
  
  
// Set up sensor software reset
void MAG3110Reset() 
{
writeByte(MAG3110_ADDRESS, CTRL_REG2, 0x10); // set reset bit to 1 to assert software reset to zero at end of boot process
}

// Allow user compensation of acceleration errors
void MAG3110Offsets()
{
   MAG3110Standby();  // Must be in standby to change registers
   
   writeByte(MAG3110_ADDRESS, OFF_X_MSB, 0x00); // X-axis compensation; this is 0 mg
   writeByte(MAG3110_ADDRESS, OFF_X_LSB, 0x00); // X-axis compensation; this is 0 mg
   writeByte(MAG3110_ADDRESS, OFF_Y_MSB, 0x00); // X-axis compensation; this is 0 mg
   writeByte(MAG3110_ADDRESS, OFF_Y_LSB, 0x00); // X-axis compensation; this is 0 mg
   writeByte(MAG3110_ADDRESS, OFF_Z_MSB, 0x00); // X-axis compensation; this is 0 mg
   writeByte(MAG3110_ADDRESS, OFF_Z_LSB, 0x00); // X-axis compensation; this is 0 mg
   
   MAG3110Active();  // Set to active to start reading
}

// Initialize the MAG3110 registers 
// See the many application notes for more info on setting all of these registers:
// http://www.freescale.com/webapp/sps/site/prod_summary.jsp?code=MAG3110
// Feel free to modify any values, these are settings that work well for me.
void initMAG3110()
{
  MAG3110Standby();  // Must be in standby to change registers

  // Set up the magnetometer sample rate and oversample ratio
    writeByte(MAG3110_ADDRESS, CTRL_REG1, magODR << 3);  
  // Enable automatic magnetic sensor resets
    writeByte(MAG3110_ADDRESS, CTRL_REG2, 0x80);  // set normal mode, correct with user offset registers

  MAG3110Active();  // Set to active to start reading
}

// Sets the MAG3110 to standby mode.
// It must be in standby to change most register settings
void MAG3110Standby()
{
  byte c = readByte(MAG3110_ADDRESS, CTRL_REG1);
  writeByte(MAG3110_ADDRESS, CTRL_REG1, c & ~(0x01));
}

// Sets the MAG3110 to active mode.
// Needs to be in this mode to output data
void MAG3110Active()
{
  byte c = readByte(MAG3110_ADDRESS, CTRL_REG1);
  writeByte(MAG3110_ADDRESS, CTRL_REG1, c | 0x01);  
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
