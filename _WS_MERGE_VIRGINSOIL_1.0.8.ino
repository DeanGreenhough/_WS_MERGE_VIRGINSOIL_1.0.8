/*
  This is an initial sketch to be used as a "blueprint" to create apps which can be used with IOTappstory.com infrastructure
  Your code can be filled wherever it is marked.

  Copyright (c) [2016] [Andreas Spiess]

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.

  virginSoilBasic V2.1.2
*/


// ================================================ SETUP ================================================
//v1.0_WSoftener includes 2 x SR04 US Sensors, DEBUG, LOOPTIME, DONEPIN
//v1.1 ADDED BLYNK issues with BLYNK crashing
//v1.2 Retry BLYNK - Added 1000uF CAP direct to board - issue appears to be USB isolator!!
//v1.3 Modify to get timing back down
//v1.3.1 Change DONEPIN from 21 to 4
//v1.3.2 Add INA219 for Vbatt move pin 22 to 32 tp clear up SDA/SCL
//v1.3.3 Changed GPIO4 to GPIO25 as unit keeps reseting and goes into download with 4R7  connected to grd
//v1.3.4 ADDDED TO ALLOW FOR CAPACITOR TO CHARGE BEFORE WIFI INIT****STABLE TO 3V**** IGNORE
//v1.3.5 CHANGED TO LOCAL BLYNK SERVER
//v1.3.6 PLAYING AROUND WITH BLYNK &  LOOPTIME
//v1.3.7 ADDED WDT  FOR 10S DUE TO SERVER BEING DOWN AND SKETCH STAYS IN A LOOP
//v1.3.8 ADDED MQTT TO THE MIX - READINGS STORED IN INFLUXDB AND DISPLAYED ON GRAFANA
//v1.3.9 DELAYS REMOVED - CODE TIDY - PRINT STATEMENTS SET TO DEBUG
//v1.4.0 ADJUSTED DELAYS AS BECAME UNSTABLE
//		 BECAME _WS_MERGE_VIRGINSOIL
//       ADDED PIN 17 PUSH BUTTON FOR EXTERNAL WAY TO PROGRAM
//       PUT IAS BACK IN LOOP AND CHANGED SETUP TO CHECK PROG STATE/MODE
//       PIN 5 ADDED TO INDICATE IN PROG MODE ON PUSH BUTTON
//V1.0.2
//       COMMETED OUT IAS.BEGIN
//       FITTED FLASHING LED TO PIN 2 PROG_LED
//V1.0.3 REMOVED DELAY IN NORMAL CODE AS RUNNING ON LOOP
//V1.0.5 ADDED EEPROM FOR MORE CONTROL OVER IOTAPPSTORY UPDATES AS POWER INTENSIVE - UPDATE FREQ 30 DAYS
//V1.0.6 OTHER
//V1.0.7 SWAP OUT EEPROM DUE TO CONFLICT, USING NVS FLASH PARTITION USING PREFERENCES

            


#include <IOTAppStory.h>                     // IotAppStory.com library
IOTAppStory IAS(COMPDATE, MODEBUTTON);       // Initialize IotAppStory
#define    SERIAL_SPEED   115200
#define COMPDATE __DATE__ __TIME__
#define MODEBUTTON 0      


#define BLYNK_PRINT Serial
//#define BLYNK_DEBUG                         // Optional, this enables more detailed prints
#define mqtt_server    "192.168.0.200"        // server name or IP
#define mqtt_user      "admin"                // username
#define mqtt_password  "admin"                // password
#define looptime_topic "WS/looptime"          // Topic looptime
#define battery_topic  "WS/battery"
#define current_topic  "WS/current"
//DEBUG
#define DEBUG            1                     // DEBUG OUTPUT 
#define LOOPTIME         0                     // LOOPTIME ONLY OUTPUT    
#define DONEPIN          25                    // TPL5110 SWITCH OFF PIN
#define PROG_LED         2					   //LED
#define PROG_BUTTON      17                    //PROGRAMMING BUTTON
//#define    BUILTIN_LED    2

#include <PubSubClient.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
//INA219
#include <Wire.h>
#include <Adafruit_INA219.h>
Adafruit_INA219 ina219;
// PREFERENCES NVS FLASH
#include <Preferences.h>
Preferences preferences;
//CREDENTIALS
char auth[]       = "5323a2d82e8c418fbe1bb13e78dbcc91";
char WIFI_SSID[]  = "Limitless";
char WIFI_PWD []  = "F41thl355";
//WDT SETUP
#include "esp_system.h"
const int wdtTimeout = 10000;  //WDT SET IN mS
hw_timer_t *timer = NULL;
//WDT CALL BACK FUNCTION
void IRAM_ATTR resetModule() {
  ets_printf("\n");
  ets_printf("****************************************\n");
  ets_printf("***** WDT ACTIVATED SET TO 10 SECS *****\n");
  ets_printf("****************************************\n");
  ets_printf("\n");
  delay(10);
  digitalWrite(DONEPIN, LOW);   //SEND DONE TO TPL5110 TO SHUT DOWN
  delay (10);
  //esp_restart();
}
//VARIABLES
unsigned int Button_State = 0;
//TIMERS
unsigned int loopTime     = 0;       //LOOP TIMER
unsigned long startMillis;           //LOOP TIMER
unsigned long currentMillis;         //LOOP TIMER
//INA219
//float shuntvoltage      = 0;
float   busvoltage        = 0;
float   current_mA        = 0;
//float  loadvoltage      = 0;
//SR04
//LEFT
const int leftTrigPin     =  19;
const int leftEchoPin     =  18;
unsigned long   durationLeft;
unsigned int    distanceLeft;
//RIGHT
const int rightTrigPin   =  23;
const int rightEchoPin   =  32;
unsigned long  durationRight;
unsigned int   distanceRight;
//WIFI & MQTT
WiFiClient espClient;
PubSubClient client(espClient);
//FUNCTIONS
void Vbatt()
{
  //shuntvoltage = ina219.getShuntVoltage_mV();
    busvoltage   = ina219.getBusVoltage_V();
    current_mA   = ina219.getCurrent_mA();
  //loadvoltage  = busvoltage + (shuntvoltage / 1000);

  if (DEBUG) Serial.print("Bus Voltage:   "); Serial.print(busvoltage, 3); Serial.println(" V");  
  if (DEBUG)  Serial.print("Current:       "); Serial.print(current_mA); Serial.println(" mA");
//if (DEBUG)  Serial.print("Shunt Voltage: "); Serial.print(shuntvoltage); Serial.println(" mV");
//if (DEBUG)  Serial.print("Load Voltage:  "); Serial.print(loadvoltage); Serial.println(" V");
  if (DEBUG)  Serial.println("");
}

void takeMeasurementLeft() 
  {
    digitalWrite(leftTrigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(leftTrigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(leftTrigPin, LOW);  
    durationLeft = pulseIn(leftEchoPin, HIGH);
    distanceLeft = durationLeft * 0.034 / 2;  
    if (DEBUG)Serial.print("Left:  ");
    if (DEBUG)Serial.println(distanceLeft);
  }

void takeMeasurementRight() 
 {
  digitalWrite(rightTrigPin, LOW);
  delayMicroseconds(2); 
  digitalWrite(rightTrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(rightTrigPin, LOW); 
  durationRight = pulseIn(rightEchoPin, HIGH);  
  distanceRight = durationRight * 0.034 / 2;
  if (DEBUG)Serial.print("Right: ");
  if (DEBUG)Serial.println(distanceRight);
 }

void reconnect() {

  while (!client.connected()) 
    {
      Serial.print    ("Connecting to MQTT broker ...");
  if (client.connect  ("ESP32Client", mqtt_user, mqtt_password)) {
      Serial.println  ("OK");
    } else {
      Serial.print    ("[Error] Not connected: ");
      Serial.print(client.state());
      Serial.println  ("Wait 0.5 seconds before retry.");
      delay(500);
    }
  }
}


void setup() 
{                   
                    pinMode(DONEPIN, OUTPUT);                           //MUST BE FIRST TO KEEP POWER ON
                    digitalWrite(DONEPIN, HIGH);                        //MUST BE FIRST TO KEEP POWER ON
                    pinMode(PROG_BUTTON,INPUT_PULLUP);
                    pinMode(PROG_LED,OUTPUT);
                    digitalWrite (PROG_LED, LOW);                  
                    Button_State = digitalRead (PROG_BUTTON);
                if (DEBUG)Serial.println(__FILE__);
                if (DEBUG)Serial.println(__DATE__);
                if (DEBUG)Serial.println(__TIME__);
                    
                if (DEBUG)Serial.print ("Button_State = ");          
                if (DEBUG)Serial.println (Button_State);
                     //NVS - PREFERENCES
                     preferences.begin("IAS_UPDATE", false); 
                     unsigned int counter = preferences.getUInt("counter", 0);
                     Serial.printf("Initial counter value: %u\n", counter);                                
                     
//CONTROL STARTS HERE
          
                 if (counter >= 14)                                      //CHECK  IAS_UPDATE LIMIT REACHED
                    {counter = 0;                                        //RESET COUNTER
                     Serial.printf(">=3 counter value: %u\n", counter);  //PRINT OUT VALUE
                     preferences.putUInt("counter", counter);            //WRITE TO NVS
                     preferences.end();                                  //END WRITE                 
                     // ACTUAL WORK BEGINS
                     IAS.begin('P');                                     //UPDATE FIRMWARE/SKETCH
                     delay(1000);
                     digitalWrite(DONEPIN, LOW);                         //SEND DONE TO TPL5110 TO SHUT DOWN                                       
                    }               
                                                         
               else   
                    {
                      counter++;                                          //UPDATE COUNTER
                      Serial.printf("ELSE counter value: %u\n", counter); //PRINT OUT CURRENT VALUE
                      preferences.putUInt("counter", counter);            //WRITE TO NVS
                      preferences.end();                                  //END WRITE
                      //WDT SETUP
                      timer = timerBegin(0, 80, true);                  //timer 0, div 80
                      timerAttachInterrupt(timer, &resetModule, true);  //attach callback
                      timerAlarmWrite(timer, wdtTimeout * 1000, false); //set time in us
                      timerAlarmEnable(timer);                          //enable interrupt
                      timerWrite(timer, 0);                             //reset timer (feed watchdog)
                    
                      unsigned long loopTime = millis();
                      Serial.println("");
                      Serial.println("**********START WDT COUNTER**********");
                      Serial.println("");
                      startMillis = millis();							 //START LOOPTIME                    
                    
                      uint32_t currentFrequency;
                      ina219.begin();
                      ina219.setCalibration_16V_400mA();
                    
                      if (DEBUG)Serial.println(__FILE__);
                      if (DEBUG)Serial.println(__DATE__);
                      if (DEBUG)Serial.println(__TIME__);
                      if (DEBUG)Serial.print("startMillis  ");
                      if (DEBUG)Serial.println(startMillis);
                                          
                      //INITIALISE SENSORS
                      //LEFT
                      pinMode(leftTrigPin, OUTPUT);
                      pinMode(leftEchoPin, INPUT);
                      //RIGHT
                      pinMode(rightTrigPin, OUTPUT);
                      pinMode(rightEchoPin, INPUT);
                    
                      //MEASURE VOLTAGE BATTERY
                      Vbatt();                                //TAKE POWER MEASUREMENTS
                      float Sensor_V1 = (busvoltage);         //MEASURE VBATT
                      int Sensor_V4 = (current_mA);           //MEASURE CURRENT
                      takeMeasurementLeft();                  //MEASURE LEFT
                      int Sensor_V2 = (distanceLeft);
                      takeMeasurementRight();                 //MEASURE RIGHT
                      int Sensor_V3 = (distanceRight);
                    
                      //LOCAL BLYNK SERVER
                      Blynk.begin(auth, WIFI_SSID, WIFI_PWD, IPAddress(192, 168, 0, 200), 8080);
                      //BLYNK CLOUD SERVER
                      //Blynk.begin(auth, WIFI_SSID, WIFI_PWD;
                      Blynk.run();
                      Blynk.virtualWrite(V1, Sensor_V1);
                      Blynk.virtualWrite(V2, Sensor_V2 );
                      Blynk.virtualWrite(V3, Sensor_V3 );
                      Blynk.virtualWrite(V4, Sensor_V4 );
                    
                      currentMillis = millis();                                  //END LOOPTIME
                      loopTime = (currentMillis - startMillis) ;
                      int Sensor_V5 = loopTime;
                      Blynk.virtualWrite(V5, Sensor_V5 );
                    
                      unsigned int looptime = loopTime;
                      
                      //SEND MQTT TO SERVER
                      client.setServer(mqtt_server, 1883);    
                      if (!client.connected()) {  reconnect();  }
                     
                      client.publish(looptime_topic, String(looptime).c_str(), true);   
                      delay(20);                      
                      if (DEBUG) { Serial.print(looptime); Serial.println("  looptime sent to MQTT.");delay(10); }
                      client.publish(battery_topic, String(busvoltage).c_str(), true); 
                      delay(20);  
                      if (DEBUG) { Serial.print(busvoltage);Serial.println("  battery sent to MQTT.");delay(10); }
                      client.publish(current_topic, String(current_mA).c_str(), true);  
                      delay(20);                     
                      if (DEBUG) { Serial.print(current_mA);Serial.println("  current sent to MQTT.");delay(10); }
                      //DEBUG
                      if (DEBUG)Serial.print ("loopTime = ");                 //DEBUG
                      if (DEBUG)Serial.println(loopTime);
                      delay(10);
                      
                      digitalWrite(DONEPIN, LOW);   //SEND DONE TO TPL5110 TO SHUT DOWN
                      
                 }
                                                                                       
                     
                   
                                  
}



// ================================================ LOOP =================================================
void loop() 
          {          
            //WALK ON BY NOTHING TO SEE HERE - LOW POWER PROJECT
          }
