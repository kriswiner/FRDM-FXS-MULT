 /* FXAS21000 Example Code
 Original sketch by: Kris Winer
 May 30, 2014
 
 license: Beerware - Use this code however you'd like. If you 
 find it useful you can buy me a beer some time.
 
 FXAS21000 is a small, low-power, 3-axis yaw, pitch, and roll
 angular rate gyroscope. The full-scale range is adjustable from
 ±200°/s to ±1600°/s. It features both I2C and SPI interfaces. We parameterize the registers, 
 calibrate the gyro, get properly scaled angular rates, and display all on an on-breadboard 
 84 x 68 Nokia 5110 LCD display.
 
 We also set up the angular rate threshold detection so rates over some fraction (here 10%) of the full scale are
 displayed with axis over threshold and the polarity.
 
 Hardware setup:
 FXAS21000 Breakout ------- Arduino
 3.3V --------------------- 3.3V
 SDA ----------------------- A4
 SCL ----------------------- A5
 GND ---------------------- GND
 
 SDA and SCL should have external pull-up resistors (to 3.3V).
 10k resistors are on the FRDM-FXS-MULTI breakout board.
 
 Note: The FXAS21000 is an I2C sensor; here we make use of the Arduino Wire library.
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

// Define registers per Freescale Semiconductor, Inc.
// FXAS21000 Data Sheet: Advance Information Rev 1.1, 10/2013 3-Axis, 14-bit Digital MEMS Gyroscope
// Freescale Semiconductor Data Sheet
#define STATUS           0x00
#define OUT_X_MSB        0x01    
#define OUT_X_LSB        0x02
#define OUT_Y_MSB        0x03
#define OUT_Y_LSB        0x04
#define OUT_Z_MSB        0x05
#define OUT_Z_LSB        0x06
#define DR_STATUS        0x07
#define F_STATUS         0x08
#define F_EVENT          0x0A
#define INT_SRC_FLAG     0x0B
#define WHO_AM_I         0x0C   // Should return 0xD1
#define CTRL_REG0        0x0D
#define RT_CFG           0x0E   
#define RT_SRC           0x0F
#define RT_THS           0x10
#define RT_COUNT         0x11
#define TEMP             0x12
#define CTRL_REG1        0x13
#define CTRL_REG2        0x14

// The SparkFun breakout board defaults to 1, set to 0 if SA0 jumper on the bottom of the board is set
// Seven bit device address is 0011100 or 0011101 if SAO pin 0 or 1, respectively
#define SA0 0
#if SA0
#define FXAS21000_ADDRESS 0x21  // SA0 is high, 0x1C if low
#else
#define FXAS21000_ADDRESS 0x20
#endif

// Set initial input parameters
enum gyroFSR {
  GFS_1600DPS = 0,
  GFS_800DPS,
  GFS_400DPS,
  GFS_200DPS
};

enum gyroODR {
  GODR_200HZ = 0, // 200 Hz
  GODR_100HZ,
  GODR_50HZ,
  GODR_25HZ,
  GODR_12_5HZ,
  GODR_6_25HZ, // 6.25 Hz, etc.
  GODR_3_125HZ,
  GODR_1_5625HZ
};

// Specify sensor full scale
uint8_t gyroFSR = GFS_200DPS;
uint8_t gyroODR = GODR_200HZ;
float gRes, gBias[3] = {0, 0, 0}; // scale resolutions per LSB for the sensors

// Pin definitions
int int1Pin = 12;  // These can be changed, 2 and 3 are the Arduinos ext int pins
int ledPin  = 13;  // Pro Mini led

int16_t gyroCount[3];  // Stores the 12-bit signed value
float gx, gy, gz;  // Stores the real accel value in g's
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
  display.setCursor(0,0); display.print("FXAS21000");
  display.setCursor(0, 20); display.print("3-axis 14-bit");
  display.setCursor(0, 30); display.print("gyroscope");
  display.setCursor(0, 40); display.print("25 mdeg/s LSB");
  display.display();
  delay(1000);

// Set up for data display
  display.setTextSize(1); // Set text size to normal, 2 is twice normal etc.
  display.setTextColor(BLACK); // Set pixel color; 1 on the monochrome screen
  display.clearDisplay();   // clears the screen and buffer
  
  // Read the WHO_AM_I register, this is a good test of communication
  byte c = readByte(FXAS21000_ADDRESS, WHO_AM_I);  // Read WHO_AM_I register
  display.clearDisplay();
  display.setCursor(0,0); display.print("FXAS21000");  
  display.setCursor(0,10); display.print("I Am");
  display.setCursor(0, 20); display.print("Ox");display.print(c, HEX);  
  display.setCursor(0, 30); display.print("I Should be"); 
  display.setCursor(0, 40); display.print("Ox");display.print(0xD1, HEX);  
  display.display();
  delay(1000);
  
  if (c == 0xD1) // WHO_AM_I should always be 0x2A
  {  
    FXAS21000Reset(); // Start by resetting sensor device to default settings
    calibrateFXAS21000(gBias);
    initFXAS21000();  // init the accelerometer if communication is OK
    Serial.println("FXAS21000Q is online...");

  display.clearDisplay();
  display.setCursor(0,0); display.print("FXAS21000");  
  display.setCursor(0,10); display.print("gyro bias");
  display.setCursor(0, 20); display.print("x "); display.print(gBias[0], 2); display.print(" o/s");
  display.setCursor(0, 30); display.print("y "); display.print(gBias[1], 2); display.print(" o/s");
  display.setCursor(0, 40); display.print("z "); display.print(gBias[2], 2); display.print(" o/s");
  display.display();
  delay(1000);
  }
  else
  {
    Serial.print("Could not connect to FXAS21000Q: 0x");
    Serial.println(c, HEX);
    while(1) ; // Loop forever if communication doesn't happen
  }
}

void loop()
{  
  // One can use the interrupt pins to detect a data ready condition; here we just check the STATUS register for a data ready bit
  if(readByte(FXAS21000_ADDRESS, DR_STATUS) & 0x08)  // When this bit set, all axes have new data
  {
    readGyroData(gyroCount);  // Read the x/y/z adc values
    getGres();
    
    // Calculate the gyro value into actual degrees per second
    gx = (float)gyroCount[0]*gRes - gBias[0];  // get actual gyro value, this depends on scale being set
    gy = (float)gyroCount[1]*gRes - gBias[1];  
    gz = (float)gyroCount[2]*gRes - gBias[2];   
    
    // Print out values
    Serial.print("x-rate = "); Serial.print(gx); Serial.print(" deg/s");   
    Serial.print("y-rate = "); Serial.print(gy); Serial.print(" deg/s");   
    Serial.print("z-rate = "); Serial.print(gz); Serial.print(" deg/s");  
      
    tempCount = readTempData();  // Read the x/y/z adc values
    temperature = (float) tempCount; // Temperature in degrees Centigrade
  }
  
    uint32_t deltat = millis() - count;
    if (deltat > 500) { // update LCD once per half-second independent of read rate

    display.clearDisplay();    
    display.setCursor(0,0);   display.print("FXAS21000 gyro");
    display.setCursor(0,8);   display.print("x ");  display.print(gx, 1); 
    display.setCursor(43,8);  display.print(" deg/s");
    display.setCursor(0,16);  display.print("y "); display.print(gy, 1); 
    display.setCursor(43,16); display.print(" deg/s");
    display.setCursor(0,24);  display.print("z "); display.print(gz, 1); 
    display.setCursor(43,24); display.print(" deg/s");
    display.setCursor(0, 32); display.print("T "); display.print(temperature, 1); 
    display.setCursor(43,32); display.print(" C");
   display.display(); 

    count = millis();
    digitalWrite(ledPin, !digitalRead(ledPin));
    }

  // One can use the interrupt pins to detect a motion/tap condition; 
  // here we just check the RT_SOURCE register to interpret the rate interrupt condition
  byte source = readByte(FXAS21000_ADDRESS, RT_SRC);  // Read the interrupt source register
  if(source & 0x40) {  // Check if event flag has been set
 
   // Check source of event
   if(source & 0x20) {    // Z-axis rate event
    if(source & 0x10) {
    Serial.println("-z-axis RT exceeded");
    display.setCursor(48, 40); display.print("-z "); 
    }
    else {
    Serial.println("+z-axis rate exceeded");
    display.setCursor(48, 40); display.print("+z ");
    }
   }
    
 
   // Check source of event
   if(source & 0x08) {    // Y-axis rate event
    if(source & 0x04) {
    Serial.println("-y-axis RT exceeded");
    display.setCursor(24, 40); display.print("-y ");
    }
    else {
    Serial.println("+y-axis rate exceeded");
    display.setCursor(24, 40); display.print("+y ");
    }
   }
   
      // Check source of event
   if(source & 0x02) {    // X-axis rate event
    if(source & 0x01) {
    Serial.println("-x-axis RT exceeded");
    display.setCursor(0, 40); display.print("-x ");
    }
    else {
    Serial.println("+x-axis rate exceeded");
    display.setCursor(0, 40); display.print("+x ");
    }
   }
    display.setCursor(64, 40); display.print("RT!");
    display.display();
  }

}

///////////////////////////////////////////////////////////////////////////////
// Useful functions to access the FXAS21000 gyroscope
///////////////////////////////////////////////////////////////////////////////

void getGres() {
  switch (gyroFSR)
  {
 	// Possible gyro scales (and their register bit settings) are:
	// 200 DPS (11), 400 DPS (10), 800 DPS (01), and 1600 DPS  (00). 
    case GFS_1600DPS:
          gRes = 1600.0/8192.0;
          break;
    case GFS_800DPS:
          gRes = 800.0/8192.0;
          break;
    case GFS_400DPS:
          gRes = 400.0/8192.0;
          break;           
    case GFS_200DPS:
          gRes = 200.0/8192.0;
  }
}

void readGyroData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  readBytes(FXAS21000_ADDRESS, OUT_X_MSB, 6, &rawData[0]);  // Read the six raw data registers into data array
  destination[0] = ((int16_t) rawData[0] << 8 | rawData[1]) >> 2; // signed 14-bit integers
  destination[1] = ((int16_t) rawData[2] << 8 | rawData[3]) >> 2;
  destination[2] = ((int16_t) rawData[4] << 8 | rawData[5]) >> 2;
}

int8_t readTempData()
{
  return (int8_t) readByte(FXAS21000_ADDRESS, TEMP);  // Read the 8-bit 2's complement data register 
}

 
void calibrateFXAS21000(float * gBias)
{
  int32_t gyro_bias[3] = {0, 0, 0};
  uint16_t ii, fcount;
  int16_t temp[3];
  
  // Clear all interrupts by reading the data output and STATUS registers
  readGyroData(temp);
  readByte(FXAS21000_ADDRESS, STATUS);
  
  FXAS21000Standby();  // Must be in standby to change registers

  writeByte(FXAS21000_ADDRESS, CTRL_REG1, 0x08);   // select 50 Hz ODR
  fcount = 50;                                     // sample for 1 second
  writeByte(FXAS21000_ADDRESS, CTRL_REG0, 0x03);   // select 200 deg/s full scale
  uint16_t gyrosensitivity = 41;                   // 40.96 LSB/deg/s

  FXAS21000Active();  // Set to active to start collecting data
   
  uint8_t rawData[6];  // x/y/z FIFO accel data stored here
  for(ii = 0; ii < fcount; ii++)   // construct count sums for each axis
  {
  readBytes(FXAS21000_ADDRESS, OUT_X_MSB, 6, &rawData[0]);  // Read the FIFO data registers into data array
  temp[0] = ((int16_t) rawData[0] << 8 | rawData[1]) >> 2;
  temp[1] = ((int16_t) rawData[2] << 8 | rawData[3]) >> 2;
  temp[2] = ((int16_t) rawData[4] << 8 | rawData[5]) >> 2;
  
  gyro_bias[0] += (int32_t) temp[0];
  gyro_bias[1] += (int32_t) temp[1];
  gyro_bias[2] += (int32_t) temp[2];
  
  delay(25); // wait for next data sample at 50 Hz rate
  }
 
  gyro_bias[0] /= (int32_t) fcount; // get average values
  gyro_bias[1] /= (int32_t) fcount;
  gyro_bias[2] /= (int32_t) fcount;
  
  gBias[0] = (float)gyro_bias[0]/(float) gyrosensitivity; // get average values
  gBias[1] = (float)gyro_bias[1]/(float) gyrosensitivity; // get average values
  gBias[2] = (float)gyro_bias[2]/(float) gyrosensitivity; // get average values

  FXAS21000Ready();  // Set to ready
}
  
  
// Set up sensor software reset
void FXAS21000Reset() 
{
writeByte(FXAS21000_ADDRESS, CTRL_REG1, 0x20); // set reset bit to 1 to assert software reset to zero at end of boot process
delay(100);
while(!(readByte(FXAS21000_ADDRESS, INT_SRC_FLAG) & 0x08))  { // wait for boot end flag to be set
}
display.clearDisplay();
display.setCursor(0,0); display.print("FXAS21000");
display.setCursor(0,8); display.print("boot end");
display.setCursor(0,16); display.print("flag");  
display.setCursor(0,24); display.print("detected");  
display.display();
}



// Initialize the FXAS21000 registers 
// See the many application notes for more info on setting all of these registers:
// http://www.freescale.com/webapp/sps/site/prod_summary.jsp?code=FXAS21000Q
// Feel free to modify any values, these are settings that work well for me.
void initFXAS21000()
{
  FXAS21000Standby();  // Must be in standby to change registers

  // Set up the full scale range to 200, 400, 800, or 1600 deg/s.
  writeByte(FXAS21000_ADDRESS, CTRL_REG0, gyroFSR);      // write FSR

  // Setup the 3 data rate bits, 4:2
  if(gyroODR < 8) { // data rate can only be 0 to 7
  writeByte(FXAS21000_ADDRESS, CTRL_REG1, gyroODR << 2); 
  }

  // Disable FIFO, route FIFO and rate threshold interrupts to INT2, enable data ready interrupt, route to INT1
  // Active HIGH, push-pull output driver on interrupts
  writeByte(FXAS21000_ADDRESS, CTRL_REG2,  0x0E);
  
  // Set up rate threshold detection; at max rate threshold = FSR; rate threshold = THS*FSR/128
  writeByte(FXAS21000_ADDRESS, RT_CFG, 0x07);         // enable rate threshold detection on all axes
  writeByte(FXAS21000_ADDRESS, RT_THS, 0x00 | 0x0D);  // unsigned 7-bit THS, set to one-tenth FSR; set clearing debounce counter
  writeByte(FXAS21000_ADDRESS, RT_COUNT, 0x04);       // set to 4 (can set up to 255)
        
  FXAS21000Active();  // Set to active to start reading
}


// Sets the FXAS21000 to standby mode.
// It must be in standby to change most register settings
void FXAS21000Standby()
{
  byte c = readByte(FXAS21000_ADDRESS, CTRL_REG1);
  writeByte(FXAS21000_ADDRESS, CTRL_REG1, c & ~(0x03));  // Clear bits 0 and 1; standby mode
}

// Sets the FXAS21000 to active mode.
// Needs to be in this mode to output data
void FXAS21000Ready()
{
  byte c = readByte(FXAS21000_ADDRESS, CTRL_REG1);
  writeByte(FXAS21000_ADDRESS, CTRL_REG1, c & ~(0x03));  // Clear bits 0 and 1; standby mode
  writeByte(FXAS21000_ADDRESS, CTRL_REG1, c |   0x01);   // Set bit 0 to 1, ready mode; no data acquisition yet
}

// Sets the FXAS21000 to active mode.
// Needs to be in this mode to output data
void FXAS21000Active()
{
  byte c = readByte(FXAS21000_ADDRESS, CTRL_REG1);
 writeByte(FXAS21000_ADDRESS, CTRL_REG1, c & ~(0x03));  // Clear bits 0 and 1; standby mode
 writeByte(FXAS21000_ADDRESS, CTRL_REG1, c |   0x02);   // Set bit 1 to 1, active mode; data acquisition enabled
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
