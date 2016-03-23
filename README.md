# CG023
**Firmware for CG023**

This is an alternate firmware for the Eachine CG023 quadcopter. It is a port of the Eachine H8 firmware, also by me ( silverxxx).

The quadcopter is based around the STM32F031 chip, Cortex M0 processor with 16K flash memory.

###Radio protocol:
Current options are stock cg023 transmitter or H8 mini transmitter / devo. I recommend using the H8 protocol with Devo tx, as the cg protocol only allows approx 7 bits accuracy.

###Accelerometer calibration:
Hold pitch stick down for 3 seconds, with throttle off. Needs to be done on a level surface. Saved so it only needs to be done once. You may need to use high rates in order to reach the treshold.

###Differences from H8 version:
 * rates have been integrated into protocol file, and depend on protocol abilities
 * the quadcopter rate ( in deg/sec) is no longer multiplied by 2, so it's the actual rate with devo.
 * linux version compiles way above 16K, so it's not included.
 * acro only version can be compiled by enabling respective setting in config.h

###History:

####23.03.16
* some optimizations, etc

####20.03.16:
* dual mode added
* added alternate led "battery low 2" ( 3.3V )

####20.03.16:
* CG023 stock tx protocol added


