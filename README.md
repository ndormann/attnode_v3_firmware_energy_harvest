# ATTNode v3 Firmware

## Documentation

**The full documentation for firmware options, payload decoders and programming can be found at [attno.de](https://www.attno.de/21-firmware-v3)**

## Configuration and Programming

This is the code repository for ATTNode v3 compatible firmware. At the moment it supports LoRa communication using OTAA and a BME280 or SHT21 sensor, as well as deep sleep between measurements.

The firmware is developed using [PlatformIO](https://platformio.org/). At least version 5.1.0 is needed for ATTiny3216 support.

To set the fuses for clock speed, BOD levels etc., use the "Set Fuses" operation in PlatformIO. This has to be done once for a "fresh" Node or when the Board Config in platformio.ini was changed. Afterwards it is enough to use the normal "Upload" function for Code or config.h changes.

Before programming a node, copy src/config.h.example to src/config.h and set the used sensor, LoRaWAN keys and other options as needed.

Programming is done using a [MicroUPDI Programmer](https://github.com/MCUdude/microUPDI), settings in platformio.ini are set to use it. For other pogrammer options see the PlatformIO Documentation

## Multisensor Mode

The Firmware can be configured for multiple sensors at once (see comments in config.h.example). In this case the default payload decoder from the website will not be able to correctly determine the used sensors. You **must** define a specific decoder in this case. In the TTN v3 Stack a decoder can be set per device. Use the following as an example, and uncomment the parts for each enabled sensor, then make sure the placeholder for the byte index (**ii**) 
is filled in ascending order, starting with the first enabled sensor from left to right, beginning with 1

    function decodeUplink(input) {
      var decoded = {};
      // Battery Voltage, always enabled
      decoded.v = (input.bytes[0] * 20) / 1000.0;
      
      // CO2-Sensor (SG112A, MH-Z19C, Sensair S8)
      // decoded.ppm = ((input.bytes[ii]) | (input.bytes[ii] << 8 ));

      // Temperature and Humidity (BME280 / SHT21)
      // decoded.t = ((input.bytes[ii]) | (input.bytes[ii] << 8 ) | (input.bytes[ii] << 16 ) | (input.bytes[ii] << 24)) / 100.0;
      // decoded.h = ((input.bytes[ii]) | (input.bytes[ii] << 8 ) | (input.bytes[ii] << 16 ) | (input.bytes[ii] << 24)) / 100.0;

      // Atmospheric Pressure (BME280)
      // decoded.p = ((input.bytes[ii]) | (input.bytes[ii] << 8 ) | (input.bytes[ii] << 16 ) | (input.bytes[ii] << 24)) / 100.0;

      // DS18B20 - Will append all recognized Sensors as t1, t2, t3...
      // var i;
      // var j = 1;
      // for (i = ii; i < input.bytes.length-1; i=i+2) {
      //   decoded["t" + j] = ((input.bytes[i]) | (input.bytes[i+1] << 8 ));
      //   var sign = input.bytes[i+1] & (1 << 7);
      //   if (sign)
      //     decoded["t" + j] = 0xFFFF0000 | decoded["t" + j];
      //   decoded["t" + j] = decoded["t" + j] / 100.0;
      //   j++;
      // }
  
      // Leave this part as is
      return {
        data: decoded,
        warnings: [],
        errors: []
      };
    }

Please be also aware, that not all sensor combinations are valid. Some might use the same interface and interfere with each others readings. Also keep in mind that RAM- and Flash-Space are limited, which might lead to crashes or the code not compiling/flashing correctly if to many sensors are enabled at the same time.

## Configuring via Downlink

It is possible to change the sending interval via Downlink-Packets at runtime. The time between Transmits is specified in minutes (or more exactly, 64 Second intervals) and has to be sent as a 2 byte value, which will be interpreted as an uint. so for example 0x0001 means 1 Minute, 0x0002 means 2 Minutes and so on. Sending 0xFFFF resets the value to the compiled in default.

## Acknowledgements

Parts of this code where kindly provided by [@shempe](https://twitter.com/shempe)
