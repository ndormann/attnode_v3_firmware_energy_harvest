# ATTNode v3 Firmware

## Documentation

**The full documentation for firmware options, payload decoders and programming can be found at [attno.de](https://www.attno.de/21-firmware-v3)**

## Configuration and Programming

This is the code repository for ATTNode v3 compatible firmware. At the moment it supports LoRa communication using OTAA and a BME280 or SHT21 sensor, as well as deep sleep between measurements.

The firmware is developed using [PlatformIO](https://platformio.org/). At least version 5.1.0 is needed for ATTiny3216 support.

To set the fuses for clock speed, BOD levels etc., use the "Set Fuses" operation in PlatformIO. This has to be done once for a "fresh" Node or when the Board Config in platformio.ini was changed. Afterwards it is enough to use the normal "Upload" function for Code or config.h changes.

Before programming a node, copy src/config.h.example to src/config.h and set the used sensor, LoRaWAN keys and other options as needed.

Programming is done using a [MicroUPDI Programmer](https://github.com/MCUdude/microUPDI), settings in platformio.ini are set to use it. For other pogrammer options see the PlatformIO Documentation

## Payload Decoder

You need to specify a Payload Decoder fitting for your configured Sensors for a Node. See payload/index.html in this repository. Open it in your Browser of Choice and select the enabled sensors. It will generate the Payload Decoder fitting for the choosen Sensors. You can also use the Online Version at [attno.de](https://www.attno.de/payload-generator)

## Configuring via Downlink

It is possible to change the sending interval via Downlink-Packets at runtime. Downlinks for setting the transmit interval have to be sent on port 1. The time between Transmits is specified in minutes (or more exactly, 64 Second intervals) and as a 2 byte value, which will be interpreted as an uint. So for example 0x0001 means 1 Minute, 0x0002 means 2 Minutes and so on. Sending 0xFFFF resets the value to the compiled in default.

Sending an arbitrary value on port 2 will activate the calibration sequence for sensors that support it (like some of the CO2-Sensors).

## Acknowledgements

Parts of this code where kindly provided by [@shempe](https://twitter.com/shempe)
