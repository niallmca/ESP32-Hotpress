/*
  This a simple example of the aREST UI Library for the ESP8266.
  See the README file for more details.

  Written in 2014-2016 by Marco Schwartz under a GPL license.
*/

#define motd "Hotpress 8-Apr-2018"

// Import required libraries
#include <OneWire.h>
#include "DHTesp.h"
#include <WiFi.h>
#include <aREST.h>
#include <aREST_UI.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include "SSD1306.h"
#include <Wire.h>

DHTesp dhtBathroom;
DHTesp dhtBoiler;

// Initialize the OLED display using Wire library
SSD1306  display(0x3c,5, 4);

IPAddress ip(192, 168, 1, 115);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);

#define ONE_WIRE_BUS 15   // was 3
#define HeatingRelayPin  12  // this is the digital pin controlling the heating relay
#define KitchenTempPin 13  // this is the sensor in the kitchen

// Create aREST instance
aREST_UI rest = aREST_UI();

// WiFi parameters
const char* ssid = "TP-LINK_A005D1";
const char* password = "137KincoraRoad";

// The port to listen for incoming TCP connections
#define LISTEN_PORT           80

// Create an instance of the server
WiFiServer server(LISTEN_PORT);

// Variables to be exposed to the API
String temperature;
float BathroomTemp = 17;
float BathroomHumid = 67;
float immersionTemp = 25;
int immersionStatus = 0;
int BathroomFan = 0;


// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 15   // was 3
#define BoilerRelayPin  12  // this is the digital pin controlling the immersion relay
#define FanRelayPin     14  // this is the digital pin controlling the fan
#define BathroomTempPin 13  // this is the sensor in the bathroom
#define BoilerTempPin   15  // this is the sensor attached to the hot water tank   



OneWire ds(BoilerTempPin); // on pin a 4.7K resistor is necessary


void setup(void) {
  // Start Serial
  Serial.begin(115200);
  Serial.println(motd);

 dhtBathroom.setup(BathroomTempPin); 
  //dhtBoiler.setup(BoilerTempPin);
  
  
  pinMode(BoilerRelayPin, OUTPUT);
  pinMode(FanRelayPin, OUTPUT); 
  

  // Initialising the UI will init the display too.
  display.init();

  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);

  // Set the title
  rest.title("Heating");

  // Create button to control pin 5
  //rest.label("Digital Pin 5");
  rest.button(5);

  rest.variable("temperature",&temperature);
  rest.variable("BathroomTemp", &BathroomTemp);
  rest.variable("BathroomHumid", &BathroomHumid);
  rest.variable("immersionTemp", &immersionTemp);
  rest.variable("immersionStatus", &immersionStatus);
  rest.variable("BathroomFan", &BathroomFan);
    
  // Labels
  //rest.label("KitchenTemp");
  //rest.label("KitchenHumid");
  //rest.label("settemp");

  // Function to be exposed

  // Give name and ID to device
   rest.set_id("1");
  rest.set_name("hotpress");

  // Connect to WiFi
  WiFi.config(ip, gateway, subnet);  
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  // Start the server
  server.begin();
  Serial.println("Server started");

  // Print the IP address
  Serial.println(WiFi.localIP());

  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });
    
  ArduinoOTA.setPort(8084);
  ArduinoOTA.begin();

  showstatus();

}

void loop() {

  getTemps();

  doFailsafe(); // if boiler > 90 deg then turn off immersion; if humid > 90% then turn on fan
  showstatus();

  ArduinoOTA.handle();
  
  // Handle REST calls
  WiFiClient client = server.available();
  if (!client) {
    return;
  }
  while (!client.available()) {
    delay(1);
  }
  rest.handle(client);

}

float getBoilerTemp()
{
float bTemp = immersionTemp;

byte i;
byte present = 0;
byte type_s;
byte data[12];
byte addr[8];
float celsius, fahrenheit;
 
if ( !ds.search(addr))
{
ds.reset_search();
delay(250);
return bTemp;
}
 
if (OneWire::crc8(addr, 7) != addr[7])
{
Serial.println("CRC is not valid!");
return bTemp;
}
//Serial.println();
 
// the first ROM byte indicates which chip
switch (addr[0])
{
case 0x10:
type_s = 1;
break;
case 0x28:
type_s = 0;
break;
case 0x22:
type_s = 0;
break;
default:
Serial.println("Device is not a DS18x20 family device.");
return bTemp;
}
 
ds.reset();
ds.select(addr);
ds.write(0x44, 1); // start conversion, with parasite power on at the end
delay(1000);
present = ds.reset();
ds.select(addr);
ds.write(0xBE); // Read Scratchpad
 
for ( i = 0; i < 9; i++)
{
data[i] = ds.read();
}
 
// Convert the data to actual temperature
int16_t raw = (data[1] << 8) | data[0];
if (type_s) {
raw = raw << 3; // 9 bit resolution default
if (data[7] == 0x10)
{
raw = (raw & 0xFFF0) + 12 - data[6];
}
}
else
{
byte cfg = (data[4] & 0x60);
if (cfg == 0x00) raw = raw & ~7; // 9 bit resolution, 93.75 ms
else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
 
}
celsius = (float)raw / 16.0;
fahrenheit = celsius * 1.8 + 32.0;
Serial.print("Temperature = ");
Serial.print(celsius);
Serial.print(" Celsius, ");
Serial.print(fahrenheit);
Serial.println(" Fahrenheit");

bTemp = celsius;
return bTemp;
}

void getTemps()
{

  int bh;
  int bt;
  
   dhtBathroom.setup(BathroomTempPin); 
  dhtBoiler.setup(BoilerTempPin);
  
 delay(dhtBathroom.getMinimumSamplingPeriod());
  
  
   bh = dhtBathroom.getHumidity();
   if (bh > 5.00 ) { BathroomHumid = bh; }
   bt = dhtBathroom.getTemperature();
   if (bt > 5) { BathroomTemp = bt; }

//delay(dhtBoiler.getMinimumSamplingPeriod());
//   immersionTemp = dhtBoiler.getTemperature();

immersionTemp = getBoilerTemp();

}


void doFailsafe()
{

  if (BathroomHumid > 90)
    {
      Serial.println("humidity > 90% ... turning on fan");
      digitalWrite(FanRelayPin, LOW); 
    }

  if (BathroomHumid < 80)
    {
      Serial.println("humidity < 70% ... turning off fan");
      digitalWrite(FanRelayPin, HIGH); 
    }

  if (immersionTemp > 50)
    {
      Serial.println("boiler > 50 deg C ...turning off immersion");
      digitalWrite(BoilerRelayPin, LOW);
    }

}


void showstatus()
{  

display.clear();

immersionStatus = digitalRead(BoilerRelayPin);
BathroomFan = digitalRead(FanRelayPin);

//Serial.println(digitalRead(BoilerRelayPin));
//Serial.println(digitalRead(FanRelayPin));

 String statFan = "";
 String statimmersion = "";
 display.setTextAlignment(TEXT_ALIGN_LEFT);
 display.setFont(ArialMT_Plain_10);

 if (BathroomFan == 1) { statFan = "Off"; }
else { statFan = "On"; }

 if (immersionStatus == 0) { statimmersion = "Off"; }
 else { statimmersion = "On"; }

 String Btemp = " --- ";
 String BHumid = " --- ";
 String iTemp = " --- ";

 if(isnan(BathroomTemp)) { Btemp = " --- ";  }
 else { Btemp = String(BathroomTemp) + " C"; }

 if(isnan(BathroomHumid)) { BHumid = " --- ";  }
 else { BHumid = String(BathroomHumid) + "%"; }

if(isnan(immersionTemp)) { iTemp = " --- ";  }
 else { iTemp = String(immersionTemp) + " C"; }
  
  String myIP = WiFi.localIP().toString();

 display.drawString(0, 0, "IP: " + myIP);
 display.drawString(0, 10,  "Bathroom Temp: " +  Btemp);
 display.drawString(0, 20, "Bathroom Humid: " + BHumid );
 display.drawString(0, 30, "Bathroom Fan: " + statFan);
 display.drawString(0, 40, "Immersion Temp: " +  iTemp);
 display.drawString(0, 50, "Immersion Status: " + statimmersion );
 display.display(); 
}
// Custom function accessible by the API

