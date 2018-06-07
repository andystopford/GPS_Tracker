#include <TinyGPS++.h>
#include <SoftwareSerial.h>
//#include <NeoSWSerial.h>
static const int RXPin = 9, TXPin = 8;
static const uint32_t GPSBaud = 9600;
static const int LED = 6;
unsigned long currentMillis = 0;
unsigned long previousMillis = 0;
int ledState = LOW;

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);
//NeoSWSerial ss( RXPin, TXPin );
// Power saving
#include <LowPower.h>
#include <ADXL362.h>
ADXL362 adxl;
static const int interruptPin = 2;   //Setup ADXL362 interrupt output to Interrupt 0
int16_t interruptStatus = 0;
int recordState = 0;
// SD card includes and declarations
#include <SPI.h>
#include <SdFat.h>
SdFat SD;
const int SDSelect = 5;
const int recordPin = A0;  // Close switch to record
// Receive
const byte numChars = 32;
char receivedChars[numChars];
boolean newData = false;

static boolean recvInProgress = false;
static byte ndx = 0;
char startMarker = '<';
char endMarker = '>';
char rc;

double oldLat;
double oldLng;
double lat_dist;

char* fname = (char*)malloc(10);
//---------------------------------------------------------------
// GPS configuration
// U-BLOX
// Default 1000ms:
// B5 62 06 08 06 00 E8 03 01 00 01 00 01 39
// 5000ms:
// B5 62 06 08 06 00 88 13 01 00 01 00 B1 49
//10000ms:
// B5 62 06 08 06 00 10 27 01 00 01 00 4D DD

uint8_t gps_5000ms[63] = {
0xb5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x88, 0x13, 0x01, 0x00, 0x01, 0x00, 0xb1, 0x49,0xb5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x1D, 0xAB};

uint8_t gps_10000ms[63] = {
0xb5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x10, 0x27, 0x01, 0x00, 0x01, 0x00, 0x4d, 0xdd,0xb5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x1D, 0xAB};

uint8_t gps_cyclic_tracking[] = {
0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x01, 0x22, 0x92 };

//----------------------------------------------------------------
struct datastore
  {
    //char com;
    uint32_t hms;
    float lat;
    float lon;
    //float alt;
    //uint32_t sats;
    uint32_t ram;
  };
//===================================================
void setup()
  {
    Serial.begin(9600);
    ss.begin(GPSBaud);
    // Don't need to set pin mode for analogWrite
    //pinMode( LED, OUTPUT );
    pinMode(recordPin, INPUT);
    initSD();
    ss.write(gps_10000ms,sizeof(gps_10000ms));

    // Set up Accelerometer:
    adxl.begin(4);
    delay(1000);
    adxl.setupDCActivityInterrupt(300, 10);		// 300 code activity threshold.  With default ODR = 100Hz, time threshold of 10 results in 0.1 second time threshold
    adxl.setupDCInactivityInterrupt(80, 200);		// 80 code inactivity threshold.  With default ODR = 100Hz, time threshold of 30 results in 2 second time threshold
    // Map Awake status to Interrupt 1
    adxl.SPIwriteOneRegister(0x2A, 0x40);
    // Setup Activity/Inactivity register
    adxl.SPIwriteOneRegister(0x27, 0x3F); // Referenced Activity, Referenced Inactivity, Loop Mode
    // turn on Autosleep bit
    byte POWER_CTL_reg = adxl.SPIreadOneRegister(0x2D);
    POWER_CTL_reg = POWER_CTL_reg | (0x04);				// turn on POWER_CTL[2] - Autosleep bit
    adxl.SPIwriteOneRegister(0x2D, POWER_CTL_reg);
    // turn on Measure mode
    adxl.beginMeasure();    // DO LAST! enable measurement mode
    adxl.checkAllControlRegs();   // check some setup conditions
    delay(100);

  }

//===================================================
void loop()
  {
    recordState = digitalRead(recordPin);
    currentMillis = millis();
    interruptStatus = digitalRead(interruptPin);
    while (ss.available() > 0)
      gps.encode(ss.read());
    if (recordState == HIGH)
      {
        // Accelerometer
        //if ADXL362 is asleep, call LowPower.powerdown
        if(interruptStatus == 0)
        {
          digitalWrite(6, LOW);    // Turn off LED as visual indicator of sleep
          delay(100);
          LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
        }
        else
        {
          if (gps.location.isUpdated())
            {
              logFix();
            }
          if (gps.location.isValid())
            {
              flash(500);
            }
          else
            {
              flash(125);
            }
        }
        }

    if (recordState == LOW)
      {
        recvWithStartEndMarkers();
        checkInstructions();
        //digitalWrite(LED, LOW);
        analogWrite(LED, 0);
      }
  }
//=====================================================


//=====================================================
static void logFix()
  {
    lat_dist = gps.distanceBetween(gps.location.lat(), gps.location.lng(), oldLat, oldLng);
    if (lat_dist > 10)
      {
        struct datastore gpsData;
        // get file name
        //char* fname = (char*)malloc(10);  // Caused memory leak?
        sprintf(fname, "%ld.log", gps.date.value());
        File logfile;
        logfile = SD.open(fname, FILE_WRITE);
        gpsData.hms = gps.time.value();
        gpsData.lat = gps.location.lat();
        gpsData.lon = gps.location.lng();
        //gpsData.alt = gps.altitude.feet();
        //gpsData.sats = gps.satellites.value();
        gpsData.ram = freeRam();
        logfile.write((const uint8_t *)&gpsData, sizeof(gpsData));

        //Serial.print(gps.time.value()); // Raw time in HHMMSSCC format (u32)
        //Serial.print(F(","));
        //Serial.print(gps.location.lat(), 6);
        //Serial.print(F(","));
        //Serial.print(gps.location.lng(), 6);
        //Serial.print(F(","));
        //Serial.print(gps.speed.mph());
        //Serial.print(F(","));
        //Serial.print(gps.course.deg());
        //Serial.print(F(","));
        //Serial.print(gps.altitude.feet());
        //Serial.print(F(","));
        //Serial.print(gps.satellites.value());
        //Serial.print(F(","));
        //Serial.print(F("RAM:"));
        //Serial.print(freeRam());
        //Serial.println();
        logfile.close();
      }
      oldLat = gps.location.lat();
      oldLng = gps.location.lng();
  }

//===============================
void recvWithStartEndMarkers()
  {
    while (Serial.available() > 0 && newData == false)
    {
        rc = Serial.read();
        if (recvInProgress == true)
        {
            if (rc != endMarker)
            {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars)
                {
                    ndx = numChars - 1;
                }
            }
            else
            {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }
        else if (rc == startMarker)
        {
            recvInProgress = true;
        }
    }
  }
 // recvWithStartEndMarkers

//==========================
// Check if instructions have been sent over the serial port
void checkInstructions()
{
  if (newData == true)
  {
    // Displays files on SD card
    File root;
    //char* fname = (char*)malloc(10);
    if (strcmp(receivedChars, "hello") == 0)
    {
      root = SD.open("/");
      printDirectory(root);
      root.close();
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
      SD.remove(fname);
    }
    else
    {
      // Opens selected file
      fname = receivedChars;
      //Serial.println(fname);
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
      Serial.write('\n');
      File nextFile =  dir.openNextFile();
      char name[13];
      if (! nextFile)
      {
        // no more files
        Serial.write('\r');
        break;
      }
      nextFile.getName(name, 13);
      Serial.print(name);
      Serial.print(' ');
      Serial.print(nextFile.size());
      nextFile.close();
    }
  } // printDirectory

 //========================================
void read_sd(char* name)
  {
    File logFile = SD.open(name, FILE_READ);
    if (logFile)
      {
        while(logFile.available())
        {
          struct datastore gpsData;
          logFile.read((const uint8_t *)&gpsData, sizeof(gpsData));
          //Serial.write(gpsData.sats);
          Serial.print(gpsData.hms);
          Serial.print(',');
          Serial.print(gpsData.lat, 4);
          Serial.print(',');
          Serial.print(gpsData.lon, 4);
          Serial.print(',');
          //Serial.print(gpsData.alt, 0);
          //Serial.print(',');
          //Serial.print(gpsData.sats);
          //Serial.print(',');
          Serial.print(gpsData.ram);
          Serial.println();
        }
        logFile.close();
      }
      else
        {
          Serial.println("Couldn't open log file");
        }
  }
    // read_sd

//====================================================
// Flash LED
void flash(unsigned long interval)
  {
    if (currentMillis - previousMillis >= interval)
      {
        previousMillis = currentMillis;
        //digitalWrite( LED, !digitalRead(LED));
        // PWM to reduce LED power consumption:
        if (ledState == LOW)
        {
          analogWrite(LED, 1);
          ledState = HIGH;
        }
        else
        {
          analogWrite(LED, 0);
          ledState = LOW;
        }
      }
  }

//=======================================
// See if the sd card is present and can be initialized:
void initSD()
{
  if (!SD.begin(SDSelect))
  {
    Serial.println( F("  SD card failed, or not present") );
  }
  //DEBUG_PORT.println( F("  SD card initialized.") );
} // initSD

//===========================
int freeRam () {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
} // freeRam
