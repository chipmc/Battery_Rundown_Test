/*
  Battery Rundown Tester - Demo Code
  
  This is a project to use the Arduino to control the execution of vairous battery rundown
  or load tests.  
  
  Major components: 
    - P-FET to turn on and off the source
    - Connectors to attach and monitor the power supply or device under test
    - N-FET to control the application of loads - using PWM
    - Adafruit High-Side Current Tester
    - Sparkfun DS1307 Real Time Clock
    - 128x64 Adafruit compatible OLED display for readings
    - button for autonomous operation
   
This is a sample sketch to test the board and exercise the functions.  You will want to 
modify to match your application - examples will be posted to Hackster.io
  
Chip McClelland - Cellular Data Logger
BSD license, Please keep my name in any redistribution

This code will be part of a larger project to build a control center for Arduino- 
  - Project documentation here:https://www.hackster.io/chipmc/arduino-control-center
  - GitHub repo here: https://github.com/chipmc/Battery_Rundown_Test
  
  Universal 8bit Graphics Library, https://github.com/olikraus/u8glib/  - see copyright notice below
*/


#include "U8glib.h"
#include <Wire.h>
#include <SPI.h>
#include <Time.h>
#include <DS1307RTC.h>
#include <Adafruit_INA219.h>

// Instantiations of the libraries
U8GLIB_SSD1306_ADAFRUIT_128X64 u8g(4, 3, 5);		// HW SPI Com: CS = 4, A0 = 3 (Hardware Pins are  SCK = 13 and MOSI = 11)
Adafruit_INA219 ina219;


// Pin Assignments
const int Source = 7;      // pin to control the source P-FET
const int relayPin = 9;    // pin to contorl the AC-Load relay - ACTIVE LOW!
const int Load = 6;        // pin to control the load N-FET 
const int Button = 8;      // input pin for the button on the board
const int Analog = A0;     // External Analog connection header pin
const int loadGND = A1;    // Measures voltage between the N-FET and ground (high-side of the FET)
const int testInGnd = A2;  // Measures voltage at the test board input
const int testOutGnd  = A3;  // Measures voltage at the test board output

// Program vairables
boolean onState = false;
const int onTime = 5000;
float busvoltage = 0;              // Voltage supplied to Dc Dc converter - Vin
float current_mA = 0;
const char *monthName[12] = {      // For setting the clock
  "Jan", "Feb", "Mar", "Apr", "May", "Jun",
  "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
};
int MenuChoice=0;                   // Menu Selection
boolean inTest = 0;                 // Preserves in-test state
boolean refreshMenu = 1;            // Manage the LCD and Terminal Menus
int pulse = 75;                    // pulse Length in mSec
int pulseRate = 1000;              // Time between pulses in mSec
int sampleRate = 100;                // How often will we take a reading
unsigned long pulseCount = 0;      // Count the pulse events
unsigned long lastpulse = 0;        // Keeps track of the last time we sent a pulse
unsigned long lastSample = 0;      // Keeps track of data logging frequency
tmElements_t tm;  // For setting the clock
// Variables associated with smoothing the current measurements
const int numReadings = 16;    // The number of readings that will be averaged from the INA219
int readings[numReadings];      // the readings from the current input
int index = 0;                  // the index of the current reading
int total = 0;                  // the running total
int average = 0;                // the average
int dutyCycle = 0;             // the duty cycle
boolean loadON = false;        // Toggles the Load
boolean sourceON = false;      // Toggles the Source

void setup(void) {
  pinMode(Source, OUTPUT);
  digitalWrite(relayPin,1);  // This relay is on when you pull pin low
  pinMode(relayPin, OUTPUT);
  pinMode(Button,INPUT_PULLUP);
 // analogReference(INTERNAL);
  Serial.begin(19200);
  Wire.begin();
  ina219.begin();
  
  for (int thisReading = 0; thisReading < numReadings; thisReading++)
  readings[thisReading] = 0;    // Initialize the smoothing array

  if (RTC.chipPresent()) {
      Serial.println(F("The DS1307 is stopped.  Please run the SetTime"));
      Serial.println(F("example to initialize the time and begin running."));
      Serial.println();
  } 
}

void loop(void) 
{
  if (millis() > lastSample + sampleRate)  {
    lastSample = millis();
    busvoltage = ina219.getBusVoltage_V();       // Get Vin
    // Average the current to get a better reading with PWM
    total = total - readings[index];           // subtract the last reading:   
    readings[index] = abs(ina219.getCurrent_mA());   // read from the sensor: 
    total= total + readings[index];         // add the reading to the total:
    index++;                      // advance to the next position in the array
    if (index >= numReadings)     // if we're at the end of the array...           
      index = 0;            // ...wrap around to the beginning:                    
    average = total / numReadings;           // calculate the average:
    float temp=((((((analogRead(Analog)/1.024)*5.0)-500.0)/10.0)*9.0/5.0)+32.0);  // Read the Temp
 //   float temp=((((((analogRead(Analog)/1.024)*1.1)-500.0)/10.0)*9.0/5.0)+32.0);  // Read the Temp
    u8g.firstPage();  // Refresh the display
    do {
        testMenu(busvoltage, average, temp);
    } while( u8g.nextPage() );
  }

  if (refreshMenu == 1) {              // Puts the menu in the terminal window and initaites the LCD display
    Serial.println(F("Battery Rundown Tester Menu"));
    Serial.println(F("Press 1 to Set the Clock"));
    Serial.println(F("Press 2 to Toggle the Source"));
    Serial.println(F("Press 3 to Toggle the DC-Load"));
    Serial.println(F("Press 4 to Set the DC-Load Duty Cycle"));    
    Serial.println(F("Press 5 to Toggle the AC-Load"));  
    Serial.println(F("Press 6 to Start or Stop the battery run-down test"));
    refreshMenu = 0;
  }

  MenuChoice = Serial.read();          // Reads the serial input from the terminal and executes commands
  switch (MenuChoice) {
    case '1':                          //If command = "1" Set Date and Time
      setTime();
      refreshMenu = 1;
      break;
    case '2':                         //If command = "2" Toggle the Source
      if (!digitalRead(Source)) {
        Serial.println("Source connected");
        digitalWrite(Source,1);
      }
      else {
        Serial.println("Source disconnected");
        digitalWrite(Source,0);
      }
      refreshMenu = 0;
      break;
    case '3':                         //If command = "2" Toggle the Load
      loadON = !loadON;  // Toggle the load
      if (loadON) {
        Serial.println("Load connected");
        analogWrite(Load,dutyCycle);
      }
      else {
        Serial.println("Load disconnected");
        analogWrite(Load,0);
      }

      refreshMenu = 0;
      break;
    case '4':                         //If command = "4" Set the Duty Cycle
      Serial.println("Enter duty cycle % (0-100): ");
      while (Serial.available() == 0) {  // Look for char in serial queue and process if found
        continue;
      }
      dutyCycle = (255*Serial.parseInt())/100;
      analogWrite(Load,dutyCycle);
      refreshMenu = 0;
      break;
    case '5':                         //If command = "5" Toggle the AC-Load
      if (digitalRead(relayPin)) {
        Serial.println("AC-Load connected");
        digitalWrite(relayPin,0);
      }
      else {
        Serial.println("AC-Load disconnected");
        digitalWrite(relayPin,1);
      }
      refreshMenu = 0;
      break;      
    case '6':   //If command = "6" Start or Stop the Test
      if (inTest == 0) {
        Serial.println(F("Starting the Battery Rundown Test"));
        Serial.println(F("Press 3 Again to Stop Test"));
        inTest = 1;
        loadON = true;
        dutyCycle = 0; // Start with zero duty cycle
        digitalWrite(Source,1);  // Connect the Source and Load
        analogWrite(Load,dutyCycle);
      }
      else {      // If you select "3" and test is in progress, this will stop the test
        Serial.println(F("Stopping the Battery Rundown Test"));
        Serial.println(F("Press 3 Again to Start Test"));
        Serial.println();
        refreshMenu = 1;
        loadON=false;
        digitalWrite(Source,0);  // disonnect the Source and Load
        analogWrite(Load,0);
        inTest = 0;
      }   
      break;
  }   
  
  if (inTest == 1 && millis()>=(lastpulse+pulseRate)) {   // This is the service loop while test is in progress
    if (dutyCycle < 100) { dutyCycle++; }
    analogWrite(Load,dutyCycle);
    lastpulse = millis();
  } 
  
  if (!digitalRead(Button)) {    // This is here so you can run a test without the serial terminal
    if (inTest == 0) {            // Press the button to start and stop the test
      pulseCount = 0;
      inTest = 1;
      digitalWrite(Source,1);  // Connect the Battery
      digitalWrite(Load,1);  // Connect the Load
      delay(1000);  // debounce the switch
    }
    else {                       // Press the button again the test will stop
      inTest = 0;
      digitalWrite(Source,0);  // disonnect the Battery
      digitalWrite(Load,0);  // disonnect the Load
      delay(1000);  // debounce the switch
    }
  }

}

void testMenu(float voltage, int current, float temperature) {
  tmElements_t tm;
  int dutyPercent = (100*dutyCycle)/255;
  // graphic commands to redraw the complete screen should be placed here  
  u8g.setFont(u8g_font_unifont);
  u8g.setPrintPos(0, 15); 
  if (loadON) {
    u8g.print(F("Duty Cycle="));
    u8g.print(dutyPercent,DEC);
    u8g.print('%');
  }
  else u8g.print(F("Load Off"));
  u8g.setPrintPos(0, 30); 
  if (RTC.read(tm)) {
    u8g.print(tm.Month,DEC);
    u8g.write('/');
    u8g.print(tm.Day, DEC);
    u8g.print(' ');
    u8g.print(tm.Hour,DEC);
    u8g.write(':');
    if (tm.Minute <= 9) u8g.print('0');
    u8g.print(tm.Minute, DEC);
    u8g.write(':');
    if (tm.Second <= 9) u8g.print('0');
    u8g.print(tm.Second,DEC);
  }
  u8g.setPrintPos(0, 45); 
  if (digitalRead(Source)) { 
    u8g.print(F("Vi="));
    u8g.print(voltage,1); u8g.print(F("V"));
    u8g.print(F("  "));
    u8g.print(F("I="));
    u8g.print(current,DEC); u8g.print(F("mA"));
  }
  else u8g.print(F("Source Off"));
  u8g.setPrintPos(0, 60);
  if (digitalRead(Source)) {
    u8g.print(F("Temp="));
    u8g.print(temperature);
    u8g.print(F(" F"));
  }
  else u8g.print(F("Source Off"));
}


bool getTime(const char *str)
{
  int Hour, Min, Sec;
  if (sscanf(str, "%d:%d:%d", &Hour, &Min, &Sec) != 3) return false;
  tm.Hour = Hour;
  tm.Minute = Min;
  tm.Second = Sec;
  return true;
}

bool getDate(const char *str)
{
  char Month[12];
  int Day, Year;
  uint8_t monthIndex;

  if (sscanf(str, "%s %d %d", Month, &Day, &Year) != 3) return false;
  for (monthIndex = 0; monthIndex < 12; monthIndex++) {
    if (strcmp(Month, monthName[monthIndex]) == 0) break;
  }
  if (monthIndex >= 12) return false;
  tm.Day = Day;
  tm.Month = monthIndex + 1;
  tm.Year = CalendarYrToTm(Year);
  return true;
}

void setTime(void) 
{
  bool parse=false;
  bool config=false;
  if (getDate(__DATE__) && getTime(__TIME__)) {    // get the date and time the compiler was run
    parse = true;
    if (RTC.write(tm)) {     // and configure the RTC with this info
      config = true;
    }
  }
  if (parse && config) {
    Serial.print(F("DS1307 configured Time="));
    Serial.print(__TIME__);
    Serial.print(F(", Date="));
    Serial.println(__DATE__);
  } 
  else if (parse) {
    Serial.println(F("DS1307 Communication Error"));
    Serial.println(F("Please check your circuitry"));
  } 
  else {
    Serial.print(F("Could not parse info from the compiler, Time=\""));
    Serial.print(__TIME__);
    Serial.print(F("\", Date=\""));
    Serial.print(__DATE__);
    Serial.println(F("\""));
  }
}

/*
 Universal 8bit Graphics Library, https://github.com/olikraus/u8glib/
  
  Copyright (c) 2012, olikraus@gmail.com
  All rights reserved.

  Redistribution and use in source and binary forms, with or without modification, 
  are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice, this list 
    of conditions and the following disclaimer.
    
  * Redistributions in binary form must reproduce the above copyright notice, this 
    list of conditions and the following disclaimer in the documentation and/or other 
    materials provided with the distribution.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND 
  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, 
  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
  NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.  
*/
