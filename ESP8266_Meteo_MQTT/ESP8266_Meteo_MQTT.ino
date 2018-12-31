/*
WEATHER STATION WITH DHT22 AND BME280
Version 2, with MQTT

Copyright (c) 2018 Giovanni Bernardo (CyB3rn0id)
http://www.settorezero.com
http://www.facebook.com/settorezero
http://www.twitter.com/settorezero

DESCRIPTION
This application runs on an ESP8266-based board connected via Wi-Fi in a local network 
You can read Humidity, Temperature, Heat Index, Pressure, Altitude, Minimum and Maximum temperature
temperature, humidity and heat index are obtained from DHT22 since is more accurate than BME280
from BME280 only pressure value (and altitude) is taken.
This is the MQTT Version, so you can't connect to a webserver for reading values but you must use
an MQTT Broker (I'm Using a raspberry with Mosquitto+NodeRED)

software watchdog timer used:
https://github.com/esp8266/Arduino/issues/1532

PREREQUISITES:
- PubSubClient by Nick O'Leary
- Adafruit Unified Sensor Library (by Adafruit)
- Adafruit BME280 library
- Adafruit DHT22 library
- Adafruit PCD8544 modified for ESP8266 => https://github.com/WereCatf/Adafruit-PCD8544-Nokia-5110-LCD-library
- Adafruit GFX

NOTE
I2C on ESP8266 default on GPIO4/D2 (SDA) and GPIO5/D1 (SCL)
but you can change GPIOS using Wire.begin(SDA,SCL)

SETTINGS for Arduino IDE and NodeMCU Devkit:

Board:              Generic ESP8266 Module
Flash mode:         DIO
Flash frequency:    40MHz
CPU frequency:      80MHz
Flash size:         4M (3M SPIFFS)
Reset method:       nodemcu
Upload speed:       115200

LICENSE
The MIT License (MIT)

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORTOR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
 
#include <ESP8266WiFi.h>
#include <Adafruit_BME280.h>
#include <Ticker.h>
#include <DHT.h>
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>
#include <PubSubClient.h>

/*
#define D0  16
#define D1   5      // SCL
#define D2   4      // SDA
#define D3   0
#define D4   2
#define D5  14
#define D6  12
#define D7  13
#define D8  15      // Problems using this I/O (code upload fails!)
#define D9   3      // RX
#define D10  1      // TX
*/

// comment for real usage
// uncomment for test mode (usage without sensors connected)
//#define TEST

// standard pressure at my home
// http://www.villasmunta.it/Laboratorio/un_semplice_algoritmo_per_il_cal.htm
// my home is at an altitude of 240m
// PHPA = (0,9877^(240/100))*1013,25 = 983.6
#define MY_STANDARD_PRESSURE (983.6) 

// 20 seconds for watchdog timer
#define OSWATCH_RESET_TIME 20

// comment this if you want IP address assigned by DHCP
#define USE_STATIC_IP

// change SSID and passphrase according your network
const char* ssid = "[YOUR-SSID]";
const char* password = "[YOUR-PASSPHRASE]";
const char* mqtt_server = "192.168.1.101";
String mqtt_topic = "cortile";
const uint16_t mqtt_port = 1883;
const char* clientID = "ESP8266_Meteo";

WiFiClient wifiClient;
PubSubClient client(mqtt_server, mqtt_port, wifiClient);

// data used for static IP configuration
#ifdef USE_STATIC_IP
  IPAddress ip(192,168,1,30); // ip address you want to assign to your board
  IPAddress gateway(192,168,1,1); // router address
  IPAddress subnet(255,255,255,0); // subnet mask
#endif

// led (wi-fi activity)
#define LED         16 // (D0)

// not possible to connect sensor on D8/GPIO15 since this will cause upload failing 
// (maybe HCS function of Soc is affected by pullup resistor required by DHT22?)
#define DHTPIN      13 // (D7)
#define DHTTYPE DHT22 // Sensor used: DHT22 (aka AM2302) OR AM2321
#define SEALEVELPRESSURE_HPA (1013.25) // used by BME library for computation of altitude (1013.25hPa = 1atm)

float h,t,hi,p,a,max_t,min_t,dp; // humidity,temperature,heat index,pressure,altitude,max temperature,min temperature,pressure delta
String sensorRead; // html string with sensor values separated by comma, used for serial output

// Display will be used in Software-SPI mode with CS (SCE) tied to ground
// D6 => pin 12 - Serial clock (SCLK)
// D5 => pin 14 - Serial data (DIN)
// D3 => pin  0 - Data/Command select (D/C)
// D4 => pin  2 - Reset (RST)
Adafruit_PCD8544 display = Adafruit_PCD8544 (12, 14, 0, 2);

// BME280 used in I2C mode
// put CSB to +V for 0x77 address
// put SDO to +V for I2C mode
Adafruit_BME280 bme; 

// set-up DHT sensor
DHT dht(DHTPIN, DHTTYPE);

unsigned long last_loop; // used by software watchdog timer
Ticker tickerOSWatch; // software watchdog timer
WiFiServer server(80); // html server on port 80 (standard)

// watchdog
void ICACHE_RAM_ATTR osWatch(void) 
  {
  unsigned long t = millis();
  unsigned long last_run = abs(t - last_loop);
  if(last_run >= (OSWATCH_RESET_TIME * 1000)) 
    {
    // save the hit here to eeprom or to rtc memory if needed
    ESP.restart();  // normal reboot 
    //ESP.reset();  // hard reset
    }
  }

// stuck in alarm condition (quick led blink)
void alarm(void)
  {
  // init software watchdog timer (reset after 2 minutes)
  last_loop = millis();
  tickerOSWatch.attach_ms(((120 / 3) * 1000), osWatch);
  while(1)
    {
    digitalWrite(LED,HIGH);  
    delay(200);
    digitalWrite(LED,LOW);  
    delay(200);
    }
  }
  
void setup() 
  {
  // init led
  pinMode(LED,OUTPUT);
  digitalWrite(LED,LOW);

  // init display
  display.begin();
  display.setContrast(39); //57
  display.setTextSize(1);
  display.setTextColor(BLACK);
  delay(2000);
  display.clearDisplay();

  // init serial
  Serial.begin(115200);
  
  // init DHT22
  dht.begin();

  // init BME280
  #ifndef TEST
  if (!bme.begin())
    {
    Serial.print("Error connecting to BME280");
    display.setCursor(0,0);
    display.print("BME280 Error");
    display.setCursor(0,8);
    display.print("Please check");
    display.display();
    //alarm();
    delay(2000);
    }
  else 
    {
    Serial.println("BME280 connected");
    }
  delay(10);
  #endif

  // init Wi-Fi
  Serial.print("Connecting to ");
  Serial.println(ssid);
  display.clearDisplay();
  display.setCursor(0,0);
  display.print("Connecting to");
  display.setCursor(0,8);
  display.print(" ");
  display.print(ssid);
  display.display();

  // workaround for issue #2186 => ESP8266 doesn't connect to WiFi with release 2.3.0
  // https://github.com/esp8266/Arduino/issues/2186#issuecomment-228581052
  WiFi.persistent(false);
  WiFi.mode(WIFI_OFF);
  WiFi.mode(WIFI_STA);
  // end workaround
  
  #ifdef USE_STATIC_IP
    WiFi.config(ip,gateway,subnet);
  #endif
  WiFi.begin(ssid, password);
    
  // Connect to WiFi network
  static uint8_t retries=0;
  while (WiFi.status() != WL_CONNECTED) 
    {
    delay(500);
    Serial.print(".");
    display.write('.');
    display.display();
    retries++;
    if (retries>50)
      {
      display.clearDisplay();
      display.setCursor(0,0);
      display.print("Wi-Fi Error");
      display.setCursor(0,8);
      display.print("Please check");
      display.display();
      //alarm();
      retries=0;
      }
    }
  
  Serial.println("");
  Serial.println("WiFi connected");
  digitalWrite(LED,HIGH);

  // Connection to MQTT Broker
  if (client.connect(clientID)) 
    {
    Serial.println("MQTT Broker connected");
    }
  else 
    {
    Serial.println("MQTT Broker connection FAILED");
    } 
  
  // init software watchdog timer  
  last_loop = millis();
  tickerOSWatch.attach_ms(((OSWATCH_RESET_TIME / 3) * 1000), osWatch);
  }

// read sensors and fill string
// parameter set to true will reset min/max values
void readSensor(bool resetvalues)
  {
  // sensorRead structure:
  // temperature,humidity,heatindex,pressure,altitude,minimum temperature,maximum temperature,delta pressure
  
  static bool first_read=true;
  static float t1,h1,hi1; // previous good values from DHT22

  #ifdef TEST
  
    // test mode
    h = 50.1;
    t = 24.2;
    hi = 23.8;
    p = 1034.1;
    a = 900;
    max_t=30.2;
    min_t=19.1;
    dp = p-MY_STANDARD_PRESSURE;
    
  #else 
    
    // not test mode, real usage
    h = dht.readHumidity();
    t = dht.readTemperature();
    p = bme.readPressure()/100.0F; // pressure in hPa(=mBar)
    a = bme.readAltitude(SEALEVELPRESSURE_HPA); // altitude in meters above sea level
    dp = p-MY_STANDARD_PRESSURE;

    if (resetvalues) {first_read=true;}

    // reset display buffer
    display.clearDisplay();

    // values from DHT22 not good
    if (isnan(h) || isnan(t)) 
      {
      // set previous values
      t=t1;
      h=h1;
      hi=hi1;
      }
    else
      {
      // values good
      hi = dht.computeHeatIndex(t, h, false); // heat index
      // set "previous" values to actual
      t1=t;
      h1=h;
      hi1=hi;
       
      // set/reset min and max values
      if (first_read)
        {
        max_t=t;
        min_t=t;
        first_read=false;  
        }
    else
        {
        if (t>max_t) {max_t=t;}
        if (t<min_t) {min_t=t;}
        }
      } // good values from DHT22

    #endif // end real usage

    // Display, row 1
    display.setCursor(0,0);
    display.print("Temp: ");
    display.print(t,1);
    display.write(0x5c);
    
    // Display, row 2
    display.setCursor(0,8);
    display.print("Perc: ");
    display.print(hi,1);
    display.write(0x5c);
        
    // Display, row 3
    display.setCursor(0,16);
    display.print("Umid: ");
    display.print(h,1);
    display.print("%");
    
    // Display, row 4
    display.setCursor(0,24);
    display.print("Pres: ");
    // only one decimal if number too long
    if (p>999)
      {
      display.print(p,1); 
      }
    else
      {
      display.print(p,2);
      }
    
    // Display, row 5
    display.setCursor(0,32);
    if (dp>=0)
      {
      display.print("Alta Press. ");  
      }
    else
      {
      display.print("Bassa Press.");  
      }
        
    // Display, row 6
    // ip address
    display.setCursor(0,40);
    display.println(WiFi.localIP());

    // transfer data from buffer to lcd
    display.display();
   } // end readSensor


bool mqtt_publish_values(void)
  {
  bool published=false;

  // string for serial debug
  sensorRead=String(t,1);
  sensorRead += ",";
  sensorRead += String(h,1);
  sensorRead += ",";
  sensorRead += String(hi,1);
  sensorRead += ",";
  sensorRead += String(p,2);
  sensorRead += ",";
  sensorRead += String(a,1);    
  //sensorRead += ",";
  //sensorRead += String(min_t,1);
  //sensorRead += ",";
  //sensorRead += String(max_t,1);
  sensorRead += ",";
  if (dp>=0)
      {
      sensorRead += "+";
      }
    else
      {
      }
  sensorRead += String(dp,1);
  sensorRead += "\0";
  // output on serial port   
  Serial.print("Sensor Readings:");
  Serial.println(sensorRead);
  char result[7]; // Buffer, since 
  // values to be published:
  dtostrf(t, 5, 1, result);
  published&=client.publish("temperature/cortile", result);
  published&=client.publish("cortile/temperature", result);
  dtostrf(h, 5, 1, result);
  published=client.publish("humidity/cortile", result);
  published=client.publish("cortile/humidity", result);
  dtostrf(hi, 5, 1, result);
  published&=client.publish("heat_index/cortile", result);
  published&=client.publish("cortile/heat_index", result);
  dtostrf(p, 5, 1, result);
  published&=client.publish("pressure/cortile", result);
  published&=client.publish("cortile/pressure", result);
  dtostrf(a, 5, 1, result);
  published&=client.publish("altitude/cortile",result);
  published&=client.publish("cortile/altitude",result);
  dtostrf(dp, 5, 1, result);
  published&=client.publish("pressure_delta/cortile",result);
  published&=client.publish("cortile/pressure_delta",result);

  return (published);
  }

void loop() 
  {
  static bool flash=true; // used for blinking the led
  last_loop = millis(); // value for watchdog timer
  static unsigned long prevTime=0;
  
  // prevent millis() overflow
  if (prevTime>millis()) 
    {
    prevTime=0;
    }

  // read sensors every 3 seconds
  if ((millis()-5000)>prevTime)
    {
    prevTime=millis();
    flash ^= 1; // blink the led
    digitalWrite(LED,flash);  
    readSensor(false); // read sensor without reset min/max values
    
   if (mqtt_publish_values())
      {
      Serial.println("Values published");
      }
    else 
      {
      Serial.println("One or more messages failed to send");
      Serial.println("I'll try again");
      client.connect(clientID);
      delay(10);
      mqtt_publish_values();
      }
    }
  
  }
