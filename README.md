# flora-magic

## Introduction

This is a program I have written that allows to play with :

- [Adafruit Flora board](https://www.adafruit.com/index.php?main_page=category&cPath=92) wearable electronics platform
- [Adafruit NeoPixels](https://www.adafruit.com/product/1655) and leds
- [Adafruit LSM9DS0](https://www.adafruit.com/product/2021) IMU chip

The idea was to stich this hardware to a beautiful glove (it could be an other cloth) and to produce subtile led effets according to the movements.

This is raw material to be used by future e-wearing projects.

This code is GNU GPL.



## Libraries

- Adafruit hardware specific (Adafruit_NeoPixel, Adafruit_LSM9DS0)
- [Arduino MsTimer2](http://playground.arduino.cc/Main/MsTimer2) to trigger leds from cyclic ISR
- [SoftTimer](https://github.com/prampec/arduino-softtimer) to be able to get IMU data from a fast cyclic pseudo-task

## Math

The math stuff is all about using accelerometer and gyroscope data from the IMU in order the deduce the spatial attitude of the device.

I started from a [good page at Instructable](https://www.instructables.com/id/Accelerometer-Gyro-Tutorial/).

## Application level code
`SetPix()` and `SetLed()` are customizable for application. The way they are developped in this code is not interesting at this stage.

`GetIMU()` updates 3 data structures that are usable for application :
- `LastGyroData` smoothed gyroscope data
- `RelativeVector` attitude vector deduced once gravity effect has been removed
- `LastCompoundVector` application and tunable compound vector from a balance between accelerometer and gyros
