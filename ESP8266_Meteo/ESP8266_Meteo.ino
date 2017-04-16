/*
WEATHER STATION WITH DHT22 AND BME280

Copyright (c) 2017 Giovanni Bernardo (CyB3rn0id)
http://www.settorezero.com
http://www.facebook.com/settorezero
http://www.twitter.com/settorezero

DESCRIPTION
This application runs on an ESP8266-based board connected via Wi-Fi in a local network 
You can read Humidity, Temperature, Heat Index, Pressure, Altitude, Minimum and Maximum temperature
temperature, humidity and heat index are obtained from DHT22 since is more accurate than BME280
from BME280 only pressure value (and altitude) is taken

software watchdog timer used:
https://github.com/esp8266/Arduino/issues/1532

PREREQUISITES:
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
#include <Adafruit_GFX.h>       // fonts and graphic functions
#include <Adafruit_PCD8544.h>
#include "webpage.h"

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

// led (wi-fi activity)
#define LED         16 // (D0)
// comment for real usage
// uncomment for test mode (usage without sensors connected)
//#define TEST

// not possible to connect sensor on D8/GPIO15 since this will cause upload failing 
// (maybe HCS function of Soc is affected by pullup resistor required by DHT22?)
#define DHTPIN      13 // (D7)
#define DHTTYPE DHT22 // Sensor used: DHT22 (aka AM2302) OR AM2321
#define SEALEVELPRESSURE_HPA (1013.25) // used by BME library for computation of altitude (1013.25hPa = 1atm)

// standard pressure at my home
// http://www.villasmunta.it/Laboratorio/un_semplice_algoritmo_per_il_cal.htm
// my home is at an altitude of 240m
// PHPA = (0,9877^(240/100))*1013,25 = 983.6
#define MY_STANDARD_PRESSURE (983.6) 

#define OSWATCH_RESET_TIME 20 // 20 seconds of watchdog timer
#define USE_STATIC_IP // comment this if you want address assigned by DHCP

// change SSID and passphrase according your network
const char* ssid = "[YOUR SSID]";
const char* password = "[YOUR PASSWORD]";

// data used for static IP configuration
#ifdef USE_STATIC_IP
  IPAddress ip(192,168,1,30); // ip address you want to assign to your board
  IPAddress gateway(192,168,1,1); // router address
  IPAddress subnet(255,255,255,0); // subnet mask
#endif

float h,t,hi,p,a,max_t,min_t,dp; // humidity,temperature,heat index,pressure,altitude,max temperature,min temperature,pressure delta
String sensorRead; // html string with sensor values separated by comma, used for ajax refresh
const char* header="HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nConnection: close\r\n\r\n";
const char* doctype="<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.01 Transitional//EN\" \"http://www.w3.org/TR/html4/loose.dtd\">\r\n";

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
  Serial.begin(9600);
  
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
 
  // Start the server
  server.begin();
  Serial.println("Server started");
 
  // Print the IP address
  Serial.print("Use this URL to connect: ");
  Serial.print("http://");
  Serial.print(WiFi.localIP());
  Serial.println("/");
  digitalWrite(LED,HIGH);
  
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

  #ifndef TEST
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
    sensorRead="---,---,---";
    // set previous values (only used by lcd)
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

    // string for ajax webpage refresh
    sensorRead=String(t,1);
    sensorRead += ",";
    sensorRead += String(h,1);
    sensorRead += ",";
    sensorRead += String(hi,1);
       
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

    // add pressure and altitude from BME280
    sensorRead += ",";
    sensorRead += String(p,2);
    sensorRead += ",";
    sensorRead += String(a,1);    

    // add minimum temperature
    sensorRead += ",";
    sensorRead += String(min_t,1);
    
    // add maximum temperature
    sensorRead += ",";
    sensorRead += String(max_t,1);


    // add comment about weather
    // http://www.villasmunta.it/osservando_il_barometro.htm
    sensorRead += ",";
    if (dp>=0)
      {
      sensorRead += "+";
      }
    else
      {
      }
    sensorRead += String(dp,1);
    sensorRead += "\0"; // required or JS will break the balls
    
    #else
    
      // TEST MODE without sensors
      // sensorRead structure:
      // temperature,humidity,heatindex,pressure,altitude,minimum temperature,maximum temperature,delta pressure
      sensorRead="24.5,56.3,25.1,1034.1,240,9.1,26.3,+2.1\0";
    
    #endif
    
    // output on serial port
    Serial.print("Sensor Readings:");
    Serial.println(sensorRead);

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
  if ((millis()-3000)>prevTime)
    {
    prevTime=millis();
    flash ^= 1; // blink the led
    digitalWrite(LED,flash);  
    readSensor(false); // read sensor without reset min/max values
    }
           
  // Check if a client has connected
  WiFiClient client = server.available();
  if (!client) 
    {
    return;
    }
 
  // Wait until the client sends some data
  Serial.println("new client");
  while(!client.available())
    {
    delay(1);
    }

  // Read the first line of the request
  String request = client.readStringUntil('\r');
  Serial.println(request);
  client.flush();

  // reset values requested
  if (request.indexOf("reset")>0)
    {
    readSensor(true); // read sensor resetting min/max values
    }
    
  // javascript requested sensor values
  if(request.indexOf("getValues") > 0)
    {
    client.print(header);
    client.print(sensorRead);
    Serial.print("Reading: ");
    Serial.println(sensorRead);
    request="";
    }
  else
    {
    // normal request, write html page
    // body
    String p3= "<body onLoad=\"getValues()\">\n\r"; 
    p3 += "<div style=\"text-align:center\">\n\r";
    
    p3 += "<div class=\"st\">temperatura</div>\n\r";
    p3 += "<a id=\"t\" class=\"bo\" style=\"background-color:#ff6600; margin-bottom:8px;\" href=\"?reset\">";
    p3 += String(t,1);
    p3 += "&deg;C</a>\n\r";

    // used unicode arrows for min/max:
    // http://www.w3schools.com/charsets/ref_utf_arrows.asp
    p3 += "<div style=\"text-align:center; width:100%\">";
    
    // min
    p3 += "<div id=\"tmin\" class=\"bom\" style=\"background-color:#ffc300; width:49%;\">&#8681;&nbsp;";
    p3 += String(min_t,1);
    p3 += "&deg;</div>\n\r";
    
    // max
    p3 += "<div id=\"tmax\" class=\"bom\" style=\"background-color:#ff4300; width:49%;\">&#8679;&nbsp;";
    p3 += String(max_t,1);
    p3 += "&deg;</div>\n\r";
    p3 += "</div>\n\r";
    
    // heat index
    p3 += "<div id=\"hi\" class=\"bo\" style=\"background-color:#ef5f99; font-size:24pt;\">&#128102;&nbsp;";
    p3 += String(hi,1);
    p3 += "&deg;C</div>\n\r";
    
    // relative humidity
    p3 += "<div class=\"st\">umidit&agrave; relativa</div>\n\r";
    p3 += "<div id=\"h\" class=\"bo\" style=\"background-color:#006699\">";
    p3 += String(h,1);
    p3 += "%</div>\n\r";
    
    // pressure and altitude
    p3 += "<div class=\"st\">pressione</div>\n\r";
    p3 += "<div id=\"p\" class=\"bo\" style=\"background-color:#339933\">";
    p3 += String(p,2);
    p3 += "hPa</div>\n\r";
    
    p3 += "<div id=\"c\" class=\"bo\" style=\"background-color:#339966; font-size:15pt;\">";
    p3 += "</div>\n\r";
       
    p3 += "<div id=\"a\" class=\"bo\" style=\"background-color:#669966; font-size:13pt;\">";
    p3 += String(a,1);
    p3 += "m";
    p3 += "</div>\n\r";

    p3 += "<a class=\"mi\" href=\"http://www.settorezero.com\">&copy;2017 Giovanni Bernardo</a>";
    
    // close document      
    p3 += "</div>\n\r";
    p3 += "</body>\n\r";
    p3 += "</html>";

    client.print(header);
    client.print(doctype);
    client.print(p1); // p1 defined in webpage.h, contains page header with meta-values (icons) and stylesheet
    client.print(p2); // p2 defined in webpage.h, contains javascript function for ajax refresh
    client.print(p3); // p3 generated here
    }
  
  Serial.println("Client disconnected");
  Serial.println("");
  delay(10);
  }
