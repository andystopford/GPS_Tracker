#include <Arduino.h>
#include <NMEAGPS.h>
//======================================================================
//  Program: NMEASDlog.ino
//
//  Description:  This program is an interrupt-driven GPS logger.
//    It uses the alternative serial port libraries NeoHWSerial,
//    NeoSWSerial, or NeoICSerial.
//
//  Prerequisites:
//     1) You have completed the requirements for NMEA_isr.ino
//     2) You have connected an SPI SD card and verified it is
//        working with other SD utilities.
//     3) For logging faster than the default 1Hz rate, you have
//        identified the required commands for your GPS device.
//
//  'Serial' is for debug output to the Serial Monitor window.
//  Arduino RX pin = 8
//  Arduino TX pin = 9
//======================================================================
// Check configuration
// If you are using NeoHWSerial, all Serial prints *must* use NeoSerial.print
// This define lets you use "DEBUG_PORT.print" everywhere below.  It is
// defined here to use the appropriate Serial or NeoSerial instance.
#ifdef NeoHWSerial_h
  #define DEBUG_PORT_TYPE NeoHWSerial
  #define DEBUG_PORT NeoSerial
#else
  #define DEBUG_PORT_TYPE HardwareSerial
  #define DEBUG_PORT Serial
#endif

//----------------------------------------------------------------
static NMEAGPS  gps;
static const int LED = 13;
const int recordPin = 2;  // Close switch to record

//----------------------------------------------------------------
#include <NeoICSerial.h>
//#include <NeoSWSerial.h>
#include "GPSport.h"
#include <Wire.h>
// For LCD
#include <U8glib.h>
// Pins - CHECK THESE - THEY ARE WRONG!!!
const int8_t RST_PIN = 8;
const int8_t CE_PIN = 7;
const int8_t DC_PIN = 9;
//const int8_t DIN_PIN = 11;  // uncomment for Software SPI
//const int8_t CLK_PIN = 13;  // uncomment for Software SPI
const int8_t BL_PIN = 10;
U8GLIB_TLS8204_84X48  u8g(13, 11, 7, 9 ,8);

//----------------------------------------------------------------
// Receive
const byte numChars = 32;
char receivedChars[numChars];
boolean newData = false;

//----------------------------------------------------------------
// Accelerometer
const int MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ;
int16_t prevAcY;
boolean recordEnable = true;

//----------------------------------------------------------------
// SD card includes and declarations
#include <SPI.h>
#include <SdFat.h>
SdFat SD;
const int chipSelect = 5;
boolean name_set = false;

//----------------------------------------------------------------
// For testing, it may be more convenient to simply print the
// GPS fix fields to the Serial Monitor.  Simply uncomment
// this define to skip all SD operations.  An SD card module
// does not have to be connected.
//#define SIMULATE_SD
#ifdef SIMULATE_SD
  DEBUG_PORT_TYPE &logfile = DEBUG_PORT;
#else
  File logfile;
#endif
//====================================
//====================================
void setup()
{
  // Start the normal trace output
  DEBUG_PORT.begin(9600);
  while (!DEBUG_PORT)
    ; // wait for serial port to connect.
  //DEBUG_PORT.println( F("NMEASDlog.ino started!") );
  // Start the GPS device
  gps_port.attachInterrupt(GPSisr);
  gps_port.begin(9600);
  pinMode(LED, OUTPUT); // Enable the LED for blinking feedback
  pinMode(recordPin, INPUT);
  initSD();
  waitForFix();
  setupGPS();
  // Start the Accelerometer
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
} // setup

//======================================
// Write settings to GPS
void sendUBX(uint8_t *MSG, uint8_t len)
{
  for(int i=0; i<len; i++)
  {
    gps_port.write(MSG[i]);
  }
} // sendUBX

//=======================================
// GPS Power Save Mode
void setupGPS()
{
  uint8_t setPSM[] = { 0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x01, 0x22, 0x92 }; // Setup for Power Save Mode (Default Cyclic 1s)
  sendUBX(setPSM, sizeof(setPSM)/sizeof(uint8_t));
} // setupGPS

//=======================================
// See if the sd card is present and can be initialized:
void initSD()
{
  if (!SD.begin(chipSelect))
  {
    DEBUG_PORT.println( F("  SD card failed, or not present") );
    // Flicker the LED
    while (true)
    {
      digitalWrite(LED,HIGH);
      delay(75);
      digitalWrite(LED,LOW);
      delay(75);
    }
  }
  //DEBUG_PORT.println( F("  SD card initialized.") );
} // initSD

//========================================
// Main loop
void loop()
{
  checkAccel();
  int recordState = digitalRead(recordPin);
  DEBUG_PORT.println( F("OK") );
  if (recordState == HIGH && recordEnable == true)
  {
    GPSloop();
    if (gps.overrun())
    {
      gps.overrun( false );
      DEBUG_PORT.println( F("DATA OVERRUN: fix data lost!") );
    }
  }
  recvWithStartEndMarkers();
  checkInstructions();
} // loop

//============================================
void GPSisr( uint8_t c )
{
  gps.handle( c );
} // GPSisr

//============================================
//  This routine waits for GPSisr to provide
//  a fix that has a valid location.
//  The LED is slowly flashed while it's waiting.
static void waitForFix()
{
  uint16_t lastToggle = millis();
  for (;;)
  {
    if (gps.available())
    {
      if (gps.read().valid.location)
      {
        //DEBUG_PORT.println("Got GPS");
        break; // Got it!
      }
    }
    // Slowly flash the LED until we get a fix
    if ((uint16_t) millis() - lastToggle > 500)
    {
      lastToggle += 500;
      digitalWrite( LED, !digitalRead(LED) );
      //DEBUG_PORT.write( '.' );
    }
  }
  DEBUG_PORT.println();
  digitalWrite( LED, LOW );
  gps.overrun( false ); // we had to wait a while...
} // waitForFix

//=======================================
// Get GPS data and save to SD card
static void GPSloop()
{
  if (gps.available())
  {
    gps_fix fix = gps.read();
    if (!name_set)
    {
      // Filename: derive from today's date
      char* fname = get_fname(fix);
      write_to_sd(fname);
      name_set = true;
    }
    // Log the fix information if we have a location and time
    if (fix.valid.location && fix.valid.time)
    {
      uint16_t startLoggingTime = millis();
      if (fix.dateTime.hours < 10)
        logfile.print(F("0"));
      logfile.print(fix.dateTime.hours);
      logfile.print(F(":"));
      if (fix.dateTime.minutes < 10)
        logfile.print(F("0"));
      logfile.print(fix.dateTime.minutes);
      logfile.print(F(":"));
      if (fix.dateTime.seconds < 10)
        logfile.print(F("0"));
      logfile.print(fix.dateTime.seconds);
      logfile.print(F(","));
      printL( logfile, fix.latitudeL() );
      logfile.print(F(","));
      printL( logfile, fix.longitudeL() );
      logfile.print(F(","));
      logfile.print(fix.speed_mph());
  		logfile.print(F(","));
  		//printL(logfile, fix.heading_cd());
      logfile.print(fix.heading());
      logfile.print(F(","));
  		logfile.print(fix.alt.whole);
  		logfile.print(F(","));
  		logfile.print(fix.satellites);
      //logfile.println();
      //logfile.print(F(","));
      //logfile.print(F("RAM:"));
      //logfile.print(freeRam());
      logfile.println();
      // flush() is used to empty the contents of the SD buffer to the SD.
      static uint16_t lastFlushTime = 0;
      if (startLoggingTime - lastFlushTime > 5000)
      {
        lastFlushTime = startLoggingTime; // close enough
        logfile.flush();
      }
    }
  }
} // GPSloop

//============================
// Check if movement detected
void checkAccel()
{
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  //Serial.println(AcY);
  if (AcY - prevAcY > 700 || AcY - prevAcY < -700)
  {
    recordEnable = true;
    prevAcY = AcY;
  }
  else
  {
    recordEnable = false;
  }
} // checkAccel

//===============================
// Test for incoming serial data
void recvWithStartEndMarkers()
{
  static boolean recvInProgress = false;
  static byte index = 0;
  char startMarker = '\n';
  char endMarker = '\r';
  char rc;

  while (Serial.available() > 0 && newData == false)
  {
    rc = Serial.read();
    if (recvInProgress == true)
    {
      if (rc != endMarker)
      {
        receivedChars[index] = rc;
        index++;
        if (index >= numChars)
        {
          index = numChars - 1;
        }
      }
      else
      {
        receivedChars[index] = '\0'; // terminate the string
        recvInProgress = false;
        index = 0;
        newData = true;
      }
    }
    else if (rc == startMarker)
    {
      recvInProgress = true;
    }
  }
} // recvWithStartEndMarkers

//==========================
void checkInstructions()
// Check if instructions have been sent over the serial port
{
	if (newData == true)
	{
    // Displays files on SD card
		File root;
    char* fname = (char*)malloc(10);
		if (strcmp(receivedChars, "hello") == 0)
		{
			root = SD.open("/");
			printDirectory(root);
		}

		else if (receivedChars[0] == 'D')
		{
      // Delete selected file
      int i;
      String fn;
      for ( i = 1; i < 11; i++)
      {
        fn.concat(receivedChars[i]);
      }
      fn.toCharArray(fname, 11);
      DEBUG_PORT.println(fname);
      SD.remove(fname);
		}

    else
    {
      // Opens selected file
      fname = receivedChars;
      read_sd(fname);
    }
		newData = false;
	}
} //checkInstructions

//=============================
// Send list of files on SD card
void printDirectory(File dir)
{
	while (true)
  {
		DEBUG_PORT.write('\n');
		File nextFile =  dir.openNextFile();
		char name[10];
		if (! nextFile)
		{
			// no more files
			DEBUG_PORT.write('\r');
			break;
		}
		nextFile.getName(name, 64);
		DEBUG_PORT.print(name);
    DEBUG_PORT.print(' ');
    DEBUG_PORT.print(nextFile.size());
    DEBUG_PORT.print(F("B"));
		nextFile.close();
	}
} // printDirectory

//============================
// Derive logfile name from today's date
char* get_fname(gps_fix fix)
{
  byte day = fix.dateTime.date; // Date NOT day (= of the week)
  byte mnth = fix.dateTime.month;
	byte year = fix.dateTime.year;
  char* fname = (char*)malloc(10);
  sprintf(fname, "%02u%02u%0u.log", day, mnth, year );
  return fname;
} // get_fname

//=============================
void write_to_sd(char* fname)
{
  //#ifndef SIMULATE_SD
  //  File logfile;
    logfile = SD.open(fname, FILE_WRITE);
  //#endif
  if (!logfile)
  {
    DEBUG_PORT.print( F("Couldn't create ") );
    DEBUG_PORT.println(fname);
    // If the file can't be created for some reason this leaves the LED on
    // so I know there is a problem
    digitalWrite(LED,HIGH);
    while (true) {}
  }
} // write_to_sd

//========================================
void read_sd(char* name)
  {
  	char inputChar;  // A byte from the file
  	char inputString [70]; // A line from the file
  	int stringIndex = 0; // Index of the byte in the line
  	String lineID;
    File logFile = SD.open(name, FILE_READ);
    if (logFile)
      {
        while(logFile.available())
          {
    				inputChar = logFile.read(); // Gets one byte from file
    				if (inputChar != 13) // Define breaking char here (carr. return)
			        {
      			  	inputString[stringIndex] = inputChar; // Append to the line
      			  	stringIndex++; // Increment where to write next
			        }
			      else // If end of line
			        {
				        if (lineID == inputString)
				          {
					          //Serial.write('\r');
					          break;
				          }
				        else
				          {
          					Serial.println(inputString);
          					lineID = inputString;
				          }
			          // delay (100);
                // Clear the value for the next cycle:
			          stringIndex = 0;
			          // now clear inputString:
			          memset(inputString, 0, sizeof(inputString));
			        }
			    }
        logFile.close();
      }
    else
        {
    			Serial.write('\n');
          Serial.println(F("Couldn't open log file..."));
    			Serial.println(name);
    			Serial.write('\r');
        }
  }   // read_sd

//----------------------------------------------------------------
// Utility to print a long integer like it's a float
// with 9 significant digits.
void printL( Print & outs, int32_t degE7 )
{
  // Extract and print negative sign
  if (degE7 < 0)
  {
    degE7 = -degE7;
    outs.print( '-' );
  }
  // Whole degrees
  int32_t deg = degE7 / 10000000L;
  outs.print( deg );
  outs.print( '.' );
  // Get fractional degrees
  degE7 -= deg*10000000L;
  // Print leading zeroes, if needed
  if (degE7 < 10L)
    outs.print( F("000000") );
  else if (degE7 < 100L)
    outs.print( F("00000") );
  else if (degE7 < 1000L)
    outs.print( F("0000") );
  else if (degE7 < 10000L)
    outs.print( F("000") );
  else if (degE7 < 100000L)
    outs.print( F("00") );
  else if (degE7 < 1000000L)
    outs.print( F("0") );
  // Print fractional degrees
  outs.print( degE7 );
} // printL

//===========================
// Utility to display/print available RAM
int freeRam ()
{
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
} // freeRam
