MRF24J40 Driver
============

This is a cross-platform driver for the [MRF24J40](http://www.microchip.com/wwwproducts/Devices.aspx?product=MRF24J40) radio. It is known to work with the MRF24J40MA module and PIC18 devices, but it written in standard C. An minimal Hardware Abstraction Layer is needed to operate it on any device.

## Features

* Reception 
* Transmission
* Security (at the MAC level)

This driver was mainly written to be used with the [OSNP Protocol Stack](https://github.com/briksoftware/osnp), but can be used with any IEEE 802.15.4 based protocol.

## Examples

Download [Gradusnik](https://github.com/briksoftware/mrf24j40) for an example of how to use this code.

## Notes

This driver makes use of an undocumented feature of the MRF24J40 chip: namely, when writing to any FIFO, you can write the address once, and then write as many data bytes as needed until you release the CS signal. The same is also true for reading (just continue sending zeros and read the answer). This should improve performance a little.

The same trick cannot be applied to Control Registers, or at least I have not been able to do it.
