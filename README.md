Binary Clock
============

This repo contains the source files for my self made binary clock. It uses a TI
BQ32000 real-time clock chip for time keeping, a TI TLC5928 LED driver for the
LEDs and an Atmel AtTiny24A microcontroller to control it. I plan on writing a
blog post on my website with more details about it soon. I will add a link
here as soon as I finish.

The pcb/ folder contains the schemtaic and board layout (made with EAGLE). The
firmware and source code for the Atmel microcontroller can be found in the code/
folder.

The v1 board layout contains some errors. The footprint for the LDO is wrong and
connects +3V3 to GND. Also I forgot to add a decoupling capacitor for the RTC.
You can work around this by isolating the big tab  of the LDO from GND, the RTC
functions without decoupling. Version 2 fixes those errors.
