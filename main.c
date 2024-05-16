//This code is used to the Arduino IDE
#include <W5100lwIP.h>
/* Connections
MP3_module Tx D2
MP3_module Rx D1
Relay. D3
Soil.  A0
PIR.   D5
*/
//Include the library files
#include <LiquidCrystal_I2C.h>
#define BLYNK_PRINT Serial
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <DHT.h>

//Include MP3 tf module files
#include "SoftwareSerial.h"
#include "DFRobotDFPlayerMini.h"
// Create the Player object
DFRobotDFPlayerMini player;

// Use pins 2 and 3 to communicate with DFPlayer Mini
static const uint8_t PIN_MP3_TX = D1; // Connects to module's RX 
static const uint8_t PIN_MP3_RX = D2; // Connects to module's TX 
SoftwareSerial softwareSerial(PIN_MP3_RX, PIN_MP3_TX);


char auth[] = "F6k3j2heCAuxuLGqMGJqsxeWfmZ8iyYY";  //Enter your Blynk Auth token
char ssid[] = "Tech";  //Enter your WIFI SSID
char pass[] = "hariharo";  //Enter your WIFI Password

DHT dht(D4, DHT11);//(DHT sensor pin,sensor type)  D4 DHT11 Temperature Sensor
BlynkTimer timer;

//Define component pins
#define soil              A0      //A0 Soil Moisture Sensor
#define PIR               D5      //D5 PIR Motion Sensor
#define RELAY_PIN_1       D3      //D3 Relay

int PIR_ToggleValue;
int relay1State = LOW;
int pushButton1State = HIGH;
int value = 0; 

void checkPhysicalButton();
void soilMoistureSensor();
void DHT11sensor();

//Create three variables for pressure
double T, P;
char status;

void dfplayer(int a)
{
  // Start communication with DFPlayer Mini
  player.begin(softwareSerial);
  Serial.println(softwareSerial);
  if (softwareSerial) 
  {
   Serial.println("OK");
    // Set volume to maximum (0 to 30).
    player.volume(30);
    player.play(a);
  } 
 }


void setup() 
{
 Serial.begin(9600);
 pinMode(PIR, INPUT);
 pinMode(RELAY_PIN_1, OUTPUT);
 digitalWrite(RELAY_PIN_1, LOW);
 pinMode(PUSH_BUTTON_1, INPUT_PULLUP);
 digitalWrite(RELAY_PIN_1, relay1State);
 Blynk.begin(auth, ssid, pass, "blynk.cloud", 80);
 dht.begin();
 //Call the function
 timer.setInterval(100L, soilMoistureSensor);
 timer.setInterval(100L, DHT11sensor);
 timer.setInterval(500L, checkPhysicalButton);

 //DF Player programs
 softwareSerial.begin(9600);


 dfplayer(1);
 delay(3000);
 dfplayer(2);
 delay(3000);

}

// 1.Get the soil moisture values
void soilMoistureSensor()
 {
  value = analogRead(soil);
  Serial.println(value);

  value = map(value, 0, 1024, 0, 100);
  value = (value - 100) * -1;
  if(value < 30)
  {
  Blynk.logEvent("waterneed","WARNNG! Water Needed!");
  dfplayer(3);
  delay(2000);
  dfplayer(4);
  delay(1000);
  }
  Blynk.virtualWrite(V3, value);
}

//2. Get the DHT11 sensor values
void DHT11sensor()
 {
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  if (isnan(h) || isnan(t)) 
  {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }
  Blynk.virtualWrite(V0, t);
  Blynk.virtualWrite(V1, h);
}

//3.Check the physical Button values
void checkPhysicalButton()
{
  if (digitalRead(PUSH_BUTTON_1) == LOW) {
    // pushButton1State is used to avoid sequential toggles
    if (pushButton1State != LOW) {

      // Toggle Relay state
      relay1State = !relay1State;
      digitalWrite(RELAY_PIN_1, relay1State);

      // Update Button Widget
      Blynk.virtualWrite(VPIN_BUTTON_1, relay1State);
    }
    pushButton1State = LOW;
  } else {
    pushButton1State = HIGH;
  }
}

BLYNK_WRITE(V6)
{
 PIR_ToggleValue = param.asInt();  
}

BLYNK_CONNECTED() {
  // Request the latest state from the server
  Blynk.syncVirtual(VPIN_BUTTON_1);
}

BLYNK_WRITE(VPIN_BUTTON_1) 
{
  relay1State = param.asInt();
  digitalWrite(RELAY_PIN_1, relay1State);
  if(relay1State)
  {
    dfplayer(6);
    if(value>75)
    {
      Blynk.logEvent("waterlevel","WARNNG! Water is full!"); 
      dfplayer(5);
    }
  }
  
}

//4.Get the PIR sensor values
void PIRsensor() 
{
  bool value = digitalRead(PIR);
  if (value) {
    Blynk.logEvent("pirmotion","WARNNG! Motion Detected!"); //Enter your Event Name
    WidgetLED LED(V5);
    LED.on();
  } else {
    WidgetLED LED(V5);
    LED.off();
  }  
}

void loop() {
    if (PIR_ToggleValue == 1)
    {
    PIRsensor();
    }
    else
    {
    WidgetLED LED(V5);
    LED.off();
    }

  Blynk.run();//Run the Blynk library
  timer.run();//Run the Blynk timer
  }
