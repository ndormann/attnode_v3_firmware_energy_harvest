# ATTNode v3 Firmware

## Documentation

**The full documentation for firmware options, payload decoders and programming can be found at [attno.de](https://www.attno.de/21-firmware-v3)**

## Configuration and Programming

This is the Repository for ATTNode v3 compatible firmware. At the moment it supports LoRa communication using OTAA and a BME280 or SHT21 sensor, as well as deep sleep between measurements.

The Firmware is developed using [PlatformIO](https://platformio.org/). At least Version 5.1.0 is needed for ATTiny3216 Support.

To Set the Fuses for Clock Speed, BOD Levels etc., use the "Set Fuses" Operation in PlatformIO. This has to be done once for a "fresh" Node or when the Board Config in PlatformIO was changed. Afterwards it is enough to use the normal "Upload" function for Code or config.h changes.

Before Programming Node, copy src/config.h.example to src/config.h and set the used sensor, LoRaWAN keys and other options as needed.

Programming is done using a [MicroUPDI Programmer](https://github.com/MCUdude/microUPDI), settings in platformio.ini are set to use it. For other pogrammer options see the PlatformIO Documentation

## Configuring via Downlink

It is possible to change the sending interval via Downlink-Packets at runtime. The time between Transmits is specified in minutes (or more exactly, 64 Second intervals) and has to be sent as a 2 byte value, which will be interpreted as an uint. so for example 0x0001 means 1 Minute, 0x0002 means 2 Minutes and so on. Sending 0xFFFF resets the value to the compiled in default.

## Acknowledgements

Parts of this code where kindly provided by [@shempe](https://twitter.com/shempe)
