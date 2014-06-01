FRDM-FXS-MULT
=============

Freescale sensor evaluation board

This sensor evaluation board from Freescale has a variety of their latest motion sensors including:

MPL3115A2 20-bit pressure sensor/altimeter

MAG3110 16-bit magnetometer

MMA8652FC 12-bit accelerometer

FXAS21000 14-bit gyroscope

FXOS8700CQ combination 14-bit accelerometer/16-bit magnetometer

FXLS8471 14-bit accelerometer

MMA9553L 14-bit accelerometer/pedometer

ALS-PT19-315 analog light sensor

Micro SD card for data logging

Many of these sensors have 32 byte FIFO registers for primitive data logging or data collection during microcontroller sleep followed by microcontroller burst reading of the data, which can be useful for low power operation. All of the sensors enable fast (400 kHz) I2C communication, which I am using in all of the sketches, and a few also allow SPI communication. The FXLS8471 accelerometer is very similar in functionality to the MMA8652FC accelerometer and, although it allows both I2C and SPI, the I2C is apparently not connected in the MULTI board design so SPI must be used here.

I have created Arduino sketches running on a 3.3 V 8 MHz ATMEGA 328P Pro Mini for each of the sensors individually demonstrating most of the capabilities afforded by each. In addition, I have combined several of the sensors into a single program to probe the advantages, if any, of 9 DoF sensor fusion with multiple accelerometer and magnetometer inputs. I use open-source Madgwick and Mahoney sensor fusion filters similar to those used to provide sensor fusion for the MPU-9x50 and LSM9DS0.
