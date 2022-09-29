#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include "RTClib.h"

#define CS_SD_PIN    4        // SD Card Chip Select Pin
#define MOTORPWM_PIN 5        // PWM Pin that is used for the motor controller board

#define PWM_VAL      125      // Sample pump strength Values Range from 0-255
#define SERIAL_BAUD  115200   // Serial port baud rate

/* Global Objects */
RTC_DS3231 rtc;               // Real time clock object
File myFile;                  // SD card datafile object

/* Function Declarations */
void RHTreading(float& humidity,float& temperature); // Humidity and temperature
void StartMotor(); // Start motor
void StopMotor(); // Stop motor
void ListenToSerial(); // Listen to Serial input

/* Global Variables */
const char daysOfTheWeek[7][12] = 
{
  "Sunday",
  "Monday",
  "Tuesday",
  "Wednesday",
  "Thursday",
  "Friday",
  "Saturday"
};

char filename[]      = "data.txt";  // SD Card file name
bool SDCardMounted   = false;       // whether has SD card mounted or not
bool SDCardFileReady = false;       // whether SD Card file is ready or not
bool MotorIsActive   = false;       // whether motor is active or not
char MotorStartChar  = 's';
char MotorStopChar   = 'r';

///////////////////////////////////////////
void setup () 
{
  /* Serial Init */
  Serial.begin(SERIAL_BAUD);
  while (!Serial) delay(10);
  Serial.println("Serial Initialized!");
  delay(500);

  /* Motor Init */
  TCCR3B = (TCCR3B & 0b11111000) | 0x01;
  pinMode(MOTORPWM_PIN, OUTPUT);
  StopMotor();
  Serial.println("Motor Initialized!");
  delay(500);

  /* I2C Init */
  Wire.begin();
  Serial.println("I2C Initialized!");
  delay(1000);

  /* RTC Init */
  while (!rtc.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    delay(500);
  }

  if (rtc.lostPower()) {
    Serial.println("RTC lost power, reset the time!");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); // use compile time
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));  // use manual time
  }
  Serial.println("RTC Initialized!");
  delay(500);

  /* SD Card Init */
  if (SD.begin(CS_SD_PIN)) {
    SDCardMounted = true;
    Serial.println("SD Card Initialized!");
  } else {
    SDCardMounted = false;
    Serial.println("Couldn't find SD Card. The data will not be logged locally!");
  }
  delay(500);

  /* SD Card File Init */
  if (SDCardMounted) {   
    myFile = SD.open(filename, FILE_WRITE);

    if (myFile) {
      SDCardFileReady = true;
      Serial.println("SD Card Ready to Log!");
      myFile.close();
    } else {
      SDCardFileReady = false;
      Serial.println("Error writing to the SD Card!");
    }
    delay(500);
  }
}

///////////////////////////////////////////
void loop () 
{
  ListenToSerial(); // listen to Serial Input Command
  
  if (MotorIsActive)
  {
    // RTC Update
    DateTime now = rtc.now();
    Serial.print(now.year(), DEC);
    Serial.print('/');
    Serial.print(now.month(), DEC);
    Serial.print('/');
    Serial.print(now.day(), DEC);
    Serial.print("(");
    Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
    Serial.print(")");
    Serial.print(',');
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.print(now.second(), DEC);
    Serial.print(',');

    // Humidity and Temperature update
    static float temp=0;  // Temperature Variable
    static float rh=0;    // Relative humidity variable
    RHTreading(rh, temp);
    Serial.print(rh);
    Serial.print(',');
    Serial.print(temp);
    Serial.println();

    // SD Card Update
    if (SDCardMounted && SDCardFileReady) 
    {
      myFile = SD.open(filename, FILE_WRITE);

      myFile.print(now.year(), DEC);
      myFile.print('/');
      myFile.print(now.month(), DEC);
      myFile.print('/');
      myFile.print(now.day(), DEC);
      myFile.print("(");
      myFile.print(daysOfTheWeek[now.dayOfTheWeek()]);
      myFile.print(")");
      myFile.print(',');
      myFile.print(now.hour(), DEC);
      myFile.print(':');
      myFile.print(now.minute(), DEC);
      myFile.print(':');
      myFile.print(now.second(), DEC);
      myFile.print(',');

      myFile.print(rh);
      myFile.print(',');
      myFile.print(temp);
      myFile.println();   
          
      myFile.close();
    }
  }
  delay(500);
}

///////////////////////////////////////////
void RHTreading(float& humidity,float& temperature)
{
  unsigned int reading=0;
  // Default address is 0x27, 39
  // Enter command mode and set to normal mode
  // Unable to send just the write byte that is needed to sample with Wire library
  // So send three bytes, 0x80, 0x00, 0x00
  Wire.beginTransmission(39);
  //Serial.println("Transmission Started");
  (Wire.write(byte(0x80)));
  (Wire.write(byte(0x00)));
  (Wire.write(byte(0x00)));
  (Wire.endTransmission());
  // Needs 42.5ms to exit command mode
  delay(50);

  Wire.requestFrom(39,4);
  if (4 <= Wire.available()) { // if two bytes were received
    reading = Wire.read();     // receive high byte (overwrites previous reading)
    reading = reading << 8;    // shift high byte to be high 8 bits
    reading |= Wire.read();    // receive low byte as lower 8 bits
    if (reading > 16384)
    {
      reading = -1; // Returns a negative number
    }
    humidity = reading / 163.82; // Equation from datasheet

    reading = Wire.read();    // receive high byte (overwrites previous reading)
    reading = reading << 8;  // shift high byte to be high 8 bits
    reading |= Wire.read();  // receive low byte as lower 8 bits
    reading = reading>>2;
    if (reading > 16384)
    {
      reading = -1;          // Will return below -40C
    }
    temperature = reading / (16382.0) * 165.0 - 40.0;//Equation from datasheet
  }
}

void StartMotor()
{
  analogWrite(MOTORPWM_PIN, PWM_VAL);
  MotorIsActive = true;
}

void StopMotor()
{
  analogWrite(MOTORPWM_PIN, 0);
  MotorIsActive = false;
}

void ListenToSerial()
{
  while (Serial.available() > 0) 
  {
    char incomingByte = Serial.read();
    if (incomingByte != '\n') 
    {
      if (incomingByte == MotorStartChar && !MotorIsActive)
      {
        Serial.println("Start Motor");
        StartMotor();
      } 
      else if (incomingByte == MotorStopChar && MotorIsActive)
      {
        Serial.println("Stop Motor");
        StopMotor();
      }
    }
  }
}
