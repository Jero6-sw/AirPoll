# AirPoll
Pollution sensor module connected by ESP32-Bluetooth to a smartphone Aircasting app


## Hardware
- ESP32 devkitC
- DS18B20+ : Temperature sensor through 1-wire
- DSM501 : Particule sensor
- TGS2611 : NO2 Methane sensor (analog)

## Software for ESP32
 - Bluetooth connexion using internal module and SPP protocol
 - Sensors acquisition
 - Data transmission each 10 seconds to aircasting smartphone app
 
## Aircasting protocol
 
Each sensor reading should be written as one line to the serial output. Lines should end with '\\n' and should have the following format:

\<Measurement value\>;\<Sensor package name\>;\<Sensor name\>;\<Type of measurement\>;\<Short type of measurement\>;\<Unit name\>;\<Unit symbol/abbreviation\>;\<T1\>;\<T2\>;\<T3\>;\<T4\>;\<T5\>

The Sensor name should be different for each sensor.

 \<Sensor package name\> = AirPoll + BT MAC address
 
 \<Sensor name\> = AirPoll-CH4 / AirPoll-PM1 / AirPoll-C
 
 \<Type of measurement\>;\<Short type of measurement\> = Methane;CH4 Gas / Particulate Matter;PM / Temperature;C / Humidity;RH
 
 \<Unit name\>;\<Unit symbol/abbreviation\> = response indicator;RI / degrees Celsius;C / percent;%

T1..T5 are integer thresholds which guide how values should be displayed -
- lower than T1 - extremely low / won't be displayed
- between T1 and T2 - low / green
- between T2 and T3 - medium / yellow
- between T3 and T4 - high / orange
- between T4 and T5 - very high / red
- higher than T5 - extremely high / won't be displayed
 
 
 
 
