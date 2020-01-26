# ESP8266 Simple Weather Station
![PIC16F15376 Curiosity Nano](documents/weather_station.jpg)

This is a simple Wi-Fi Weather Station based on a NodeMCU (ESP8266).  
It Shows Temperature, Humidity, Heat Index, Pressure, Altitude, Minimum and Maxim Temperatures on a display and on a remote device.  
I made 2 versions:  
- Server version (you connect cirectly to station IP address)
- MQTT version (you need an MQTT Broker and a system for visualize data: I use a Raspberry Pi with Mosquitto and NodeRed)

## Sensors used
- DHT22 for Temperature and Humidity
- BME280 for atmosferic pressure

BME280 has also a temperature sensor but I found it inaccurate.  
In the picture I've enclosed circuit and a little 220V->5V switching PSU in an IP56 external box with transparent lid: it's made by Gewiss and box code is GW 44 426
