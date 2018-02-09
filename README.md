# [VAV](https://en.wikipedia.org/wiki/Variable_air_volume)/[HVAC](https://en.wikipedia.org/wiki/HVAC) Controller Firmware for AtMega MK

## Overview

This firmware allows using [Modbus](https://en.wikipedia.org/wiki/Modbus) protocol via RS485 interface controlling air conditioner system.

**Features**

* 3 output ports 1-10v, for dumpers.
* 1 output port Mitsubishi CN2A
* Proportional regulation AirFlow regarding how many dumpers opened
* Changeable Modbus Settings (slave address and baudrate)
* Ability to change proportional coefficients and ability to override value using Modbus

Check documentation in Project Wiki (on Russian)