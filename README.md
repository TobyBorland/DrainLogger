DrainLogger
===========

Production code for capacitance based water level sensing on Arduino platform FEB2013

The circuit uses a LadyAda SD/RTC board and a custom high sensitivity capacitance sensing board (Cap_Shield_07*.*)
Included are Gerber design files for a Solar power regulator that float charges a SLA 1500mAh battery and supplies power to the circuit (Solar_SLA_PSU_02*.*).
The Arduino circuit has been redeveloped for micro-power consumption, the shields are switched off via a power MOSFET and the processor set to hibernate.
The MOSFET is driven by a totem pole NPN/PNP switch, a Xino Arduino clone board with prototyping space is used.