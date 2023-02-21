#include <SoftwareSerial.h>
#include "LowPower.h"
#include "Arduino.h"
#include "uRTCLib.h"

SoftwareSerial sim800(7,8);
uRTCLib rtc(0x68);

const int pulse_pin=2;

char startTime[20], stopTime[22],Location[20];

float calibrationFactor = 4.5; 
volatile byte pulseCount =0;  

float flowRate = 0.0;
unsigned int flowMilliLitres =0;
unsigned long totalMilliLitres = 0;

unsigned long oldTime = 0;


void setup() {
  Serial.begin(9600);
  sim800.begin(9600);
  
  URTCLIB_WIRE.begin();
  pinMode(pulse_pin,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pulse_pin),wakeUp,FALLING);

  sprintf(Location,"%s,%s","52.979037","-2.136831");
  rtc.set(0, 56, 12, 5, 13, 1, 22);
  // rtc.set(second, minute, hour, dayOfWeek, dayOfMonth, month, year)

  // setting the SIM800L into text Mode

  sim800.println("AT");
  updateSerial();
  sim800.println("AT+CMGF=1");
  updateSerial();
  
}

void loop() {

  rtc.refresh();

  attachInterrupt(digitalPinToInterrupt(pulse_pin),wakeUp,FALLING);
  LowPower.powerDown(SLEEP_FOREVER,ADC_OFF,BOD_OFF);

    if((millis() - oldTime) > 1000)   
  { 
    detachInterrupt(pulse_pin);

    sprintf(startTime, "%02d/%02d/%02d, %02d:%02d:%02d",rtc.year(), rtc.month(), rtc.day(), rtc.hour(), rtc.minute(), rtc.second());

    flowRate = ((1000.0 / (millis() - oldTime)) * pulseCount) / calibrationFactor;
    oldTime = millis();
    
    flowMilliLitres = (flowRate / 60) * 1000;
    totalMilliLitres += flowMilliLitres;

    
    // Print the flow rate for this second in litres / minute
    Serial.print("Flow rate: ");
    Serial.print(flowMilliLitres, DEC); 
    Serial.print("mL/Second");
    Serial.print("\t");           

    Serial.print("Total Water Volume: ");        
    Serial.print(totalMilliLitres,DEC);
    Serial.println("mL"); 
    Serial.print("\t");   

    sprintf(stopTime, "%02d/%02d/%02d, %02d:%02d:%02d",rtc.year(), rtc.month(), rtc.day(), rtc.hour(), rtc.minute(), rtc.second());
    delay(1000);
    attachInterrupt(digitalPinToInterrupt(pulse_pin),wakeUp,FALLING);  
    pulseCount = 0;

  }

  sendSMS(String(flowMilliLitres,DEC),3000);
  sendSMS(String(totalMilliLitres,DEC),3000);
  sendSMS(startTime,3000);
  sendSMS(stopTime,3000);
  sendSMS(Location,3000);
  
}

// Send sms code 

void sendSMS(String sms, int Delay){

  sim800.println("AT+CMGS=\"+254113268646\"");
  updateSerial();
  sim800.print(sms);
  updateSerial();
  sim800.write(26);
  delay(Delay);
 
  }
// Update Serial

void updateSerial()
{
  delay(500);
  while (Serial.available()) 
  {
    sim800.write(Serial.read());//Forward what Serial received to Software Serial Port
  }
  while(sim800.available()) 
  {
    Serial.write(sim800.read());//Forward what Software Serial received to Serial Port
  }
}
void wakeUp()
{
  
  pulseCount++; 
}
