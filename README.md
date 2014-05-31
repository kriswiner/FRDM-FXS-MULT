FRDM-FXS-MULT
=============

Freescale sensor evaluation board

This sensor evaluation board from Freescale has a variety of their latest motion sensors including:

MPL3115A2 20-bit pressure sensor/altimeter

MAG3110 magnetometer

MMA8652FC 12-bit accelerometer

FXAS21000 14-bit gyroscope

FXOS8700CQ combination accelerometer/magnetometer

FXLS8471 14-bit accelerometer

MMA9553L 14-bit accelerometer/pedometer

ALS-PT19-315 analog light sensor

Micro SD card for data logging

I have created Arduino sketches running on a 3.3 V 8 MHz ATMEGA 328P Pro Mini for each of the sensors individually demonstrating most of the capabilities afforded by each. In addition, I have combined several of the sensor into a single program to probe the advantages, if any, of 9 DoF sensor fusion with multiple accelerometer and magnetometer inputs. I use open-source Madgwick and Mahoney sensor fusion filters similar to those used to provide sensor fusion for the MPU-9x50 and LSM9DS0.
