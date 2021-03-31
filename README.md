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

You need to specify a Payload Decoder fitting for your configured Sensors for a Node. The Following code Shows an example for a Payload Decoder usable with TTN Stack v3. Uncomment the Parts for your used sensors. There is a Start and End comment for each possible sensor/value, uncomment the Lines between them (The ones starting with //) to activate the Decoding for a particular Sensor. Be aware that there might be an overlap in the sensor namings for sensors with the same Values (e.g. SCD30 and BME280/SHT21 all report Temperature and Humidity. If you use them in parallel you might want to change the names of the decoded fields).

    function bytesToInt16(bytes, start) {
      var out  = ((bytes[start]) | (bytes[start+1] << 8 ));
      var sign = bytes[start+1] & (1 << 7);
      if (sign)
        out = 0xFFFF0000 | out;
      return out;
    }

    function bytesToUInt16(bytes, start) {
      return ((bytes[start]) | (bytes[start+1] << 8 ));
    }

    function bytesToInt32(bytes, start) {
      return ((bytes[start]) | (bytes[start+1] << 8) | (bytes[start+2] << 16) | (bytes[start+3] << 24));
    }

    function decodeUplink(input) {
      var decoded = {};
      /* Battery Voltage, always enabled */
      decoded.v = (input.bytes[0] * 20) / 1000.0;
      
      var i = 1;
      /* Start CO2-Sensor (SG112A, MH-Z19C, Sensair S8, Sensirion SCD30) PPM */
      // decoded.ppm = ((input.bytes[ii]) | (input.bytes[ii] << 8 ));
      // i += 2;
      /* End CO2 Sensor PPM */

      /* Start Temperature + Humidity SCD30 */
      // decoded.t = bytesToInt16(input.bytes, i)/100;
      // decoded.h = bytesUToInt16(input.bytes, i+2)/100;
      // i += 4;
      /* End Temperature + Humidity SCD30 */
      
      /* Start Temperature and Humidity (SHT21) */
      // decoded.t = bytesToInt32(input.bytes, i)/100;
      // decoded.h = bytesToInt32(input.bytes, i+4)/100;
      // i += 8;
      /* End Temperature + Humidity BME/SHT */
      
      /* Start Temperature, Humidity, Atmospheric Pressure (BME280) */
      // decoded.t = bytesToInt32(input.bytes, i)/100;
      // decoded.h = bytesToInt32(input.bytes, i+4)/100;
      // decoded.p = bytesToInt32(input.bytes, i+8)/100;
      // i += 12;
      /* End Atmospheric Pressure

      /* Start DS18B20 Temperatures 
         Will append all recognized Sensors as t1, t2, t3... */
      // var n = 1;
      // for (var j = i; j < input.bytes.length-1; j+=2) {
      //   decoded["t" + n] = bytesToInt16(input.bytes, j);
      //   j++;
      // }
      /* End DS18B20 Temperatures */
  
      /* Leave this part as is */
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
