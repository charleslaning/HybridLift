#include <SdFat.h>
#include <TinyGPS++.h>
#include <SPI.h>
#include <ArduinoJson.h>

//SD Card
SdFat SD;
#define SD_CS_PIN SS
File myFile;

//TinyGPS++ object
SYSTEM_THREAD(ENABLED);
TinyGPSPlus gps;
unsigned long lastSerial = 0;
//unsigned long lastPublish = 0;
//unsigned long startFix = 0;
bool gettingFix = false;
void displayInfo();

char publishString[200];
char data[200];

//const unsigned long PUBLISH_PERIOD = 120000;z
const unsigned long SERIAL_PERIOD = 900000;
const unsigned long MAX_GPS_AGE_MS = 10000;

//for GPS
double lat;
double lon;
int satellites;

int DigitalInput[4] = {0, 0, 0, 0};
int buttonState[4] = {0, 0, 0, 0};
int lastButtonState[4] = {0, 0, 0, 0};
int digitalUse[4] = {0, 0, 0, 0};
// int startPress[4] = {0, 0, 0, 0};
// int endPress[4] = {0, 0, 0, 0};
// int timeHold[4] = {0, 0, 0, 0};
// int timeReleased[4] = {0, 0, 0, 0};
// int timeTotal[4] = {0, 0, 0, 0};

int xtime = millis();

//Analog input variables
int AnalogInput1 = 0;
int AnalogInput2 = 0;
int AnalogInput3 = 0;
int AnalogInput4 = 0;
int BatteryVolt = 0;

double anaverage1 = 0.0;
double anaverage2 = 0.0;
double anaverage3 = 0.0;
double anaverage4 = 0.0;
double anaverage5 = 0.0;

double digaverage1 = 0.0;
double digaverage2 = 0.0;
double digaverage3 = 0.0;
double digaverage4 = 0.0;

//Counter that keeps track of how many analog measurements have been taken in time period
int cnt = 0;
//this boolean allows the Particle.publish to execute since the function can not be used inside of the callback function aver_call
bool averfunc = false;
bool publishfunc = false;

//String data;
String dataString = "";
int led1 = D7;
String readCard;
//data = String::format("{\"lat\":%d, \"lon\":%d, \"digaverage1\":%d, \"anaverage1\":%d}", lat, lon, digaverage1, anaverage1);

//Calculates the averages of the digital analog inputs over the past minute and resets values
void aver_call()
{
  anaverage1 = AnalogInput1 / cnt;
  anaverage2 = AnalogInput2 / cnt;
  anaverage3 = AnalogInput3 / cnt;
  anaverage4 = AnalogInput4 / cnt;
  anaverage5 = BatteryVolt / cnt;

  digaverage1 = DigitalInput[0] / cnt;
  digaverage2 = DigitalInput[1] / cnt;
  digaverage3 = DigitalInput[2] / cnt;
  digaverage4 = DigitalInput[3] / cnt;

  averfunc = true;

  DigitalInput[0] = 0;
  DigitalInput[1] = 0;
  DigitalInput[2] = 0;
  DigitalInput[3] = 0;

  AnalogInput1 = 0;
  AnalogInput2 = 0;
  AnalogInput3 = 0;
  AnalogInput4 = 0;
  BatteryVolt = 0;

  cnt = 0;
}

void publish_call()
{
  publishfunc = true;
}

//Software timer which calls aver_call every 1 mins, runs in the background and allows main loop to continue running
Timer timer(60000, aver_call);
Timer pub(180000, publish_call);

void setup()
{
  Serial.begin();
  Serial1.begin(9600);

  timer.start();
  pub.start();
  gettingFix = true;

  pinMode(D0, INPUT);
  pinMode(D1, INPUT);
  pinMode(D2, INPUT);
  pinMode(D3, INPUT);
  pinMode(led1, OUTPUT);

  Particle.variable("dataString", dataString);

  while (!Serial)
  {
    ;
  }
  delay(1000);
  Serial.print("Initializing SD Card...");
  if (!SD.begin(SD_CS_PIN))
  {
    //SD.initErrorHalt();
    Serial.println("Initialization: Failed");
    return;
  }
  else
  {
    Serial.println("Initialization: Completed");
    myFile = SD.open("dataLog.txt", FILE_WRITE);
  }
}

void loop()
{
  //data = String::format("{\"lat\":%d, \"lon\":%d, \"digaverage1\":%d, \"anaverage1\":%d}", lat, lon, digaverage1, anaverage1);

  if (averfunc == true)
  {
    sprintf(data, "{\"lat\":%d, \"lon\":%d, \"digaverage1\":%d, \"anaverage1\":%d}", lat, lon, digaverage1, anaverage1);

    //store data to internal memory or SD storage
    myFile = SD.open("datalog.txt");
    if (myFile)
    {
      //write to SD card
      Serial.print("Writing to SD card...");
      myFile.println(data);
      //root.prettyPrintTo(myFile);
      myFile.close();
      Serial.println("Done writing to SD and closing file...");
    }
    else
    {
      Serial.println("ERROR opening file to write to...");
    }
    // Particle.publish("Average 1: ", String(average1, 2), PRIVATE);
    // Particle.publish("Average 2: ", String(average2, 2), PRIVATE);
    averfunc = false;
  }

  if (publishfunc == true)
  {
    if (Particle.connected)
    {
      //re-open SD file to read
      myFile = SD.open("dataLog.txt");
      if (myFile)
      {
        Serial.println("dataLog: ");

        //read from file until there's nothing on SD
        while (myFile.available())
        {
          Serial.write(myFile.read());
          readCard = myFile.read();
          sprintf(publishString, readCard);
        }

        Particle.publish("Collected Data: ", publishString);
        myFile.close();
      }
      else
      {
        Serial.println("ERROR opening file to read...");
      }
      //Particle.publish(data, PRIVATE);
      //Particle.publish(readCard);
    }
    else
    {
      Serial.println("NOT CONNECTED to Cloud, continuing to write to SD...");
    }
    publishfunc = false;
  }

  while (Serial1.available() > 0)
  {
    displayInfo();
    if (gps.encode(Serial1.read()))
    {
      //Serial.print("displayInfo conditional pass");
      displayInfo();
    }
  }

  if (millis() - xtime > 100)
  {
    AnalogInput1 += analogRead(A1);
    AnalogInput2 += analogRead(A0);
    AnalogInput3 += analogRead(B5);
    AnalogInput4 += analogRead(B4);
    BatteryVolt += analogRead(B3);

    DigitalInput[0] += digitalRead(D0);
    DigitalInput[1] += digitalRead(D1);
    DigitalInput[2] += digitalRead(D2);
    DigitalInput[3] += digitalRead(D3);

    cnt++;
    xtime = millis();

    // AnalogInput1 += AnalogInput1;
    // AnalogInput2 += AnalogInput2;
    // AnalogInput3 += AnalogInput3;
    // AnalogInput4 += AnalogInput4;
    // BatteryVolt += BatteryVolt;

    //Particle.publish("AnalogInput1: ", String(AnalogInput1, 2), PRIVATE);
    //Particle.publish("AnalogInput2: ", String(AnalogInput2, 2), PRIVATE);

    //For statement used to run through the 4 different digital inputs
    for (int i = 0; i < 4; i++)
    {
      //These if statements are used to check the state change of the digital inputs and record time pressed
      if (DigitalInput[i] != lastButtonState[i])
      {
        digitalUse[i]++;
      }
      lastButtonState[i] = DigitalInput[i];

      // Particle.publish("Time", String(timeHold, 2), PRIVATE);
    }
  }
}

void displayInfo()
{
  if (millis() - lastSerial >= SERIAL_PERIOD)
  {
    lastSerial = millis();

    char buf[128];
    if (gps.location.isValid() && gps.location.age() < MAX_GPS_AGE_MS)
    {
      snprintf(buf, sizeof(buf), "%f,%f,%f", gps.location.lat(), gps.location.lng(), gps.altitude.meters());
    }
    else
    {
      strcpy(buf, "no location");
    }

    Serial.println(buf);
  }
}

//____________________________________________________________________________________________

/*
#include <SPI.h>
//#include <SD.h>
#include "SdFat.h"
SdFat SD;

#define SD_CS_PIN SS
File myFile;

void setup()
{
   // Open serial communications and wait for port to open:
   Serial.begin(9600);
   SPI2.begin(SS);

   Serial.print("Initializing SD card...");

   if (!SD.begin(SD_CS_PIN))
   {
      Serial.println("initialization failed!");
      return;
   }
   Serial.println("initialization done.");

   // open the file. note that only one file can be open at a time,
   // so you have to close this one before opening another.
   myFile = SD.open("test.txt", FILE_WRITE);

   // if the file opened okay, write to it:
   if (myFile)
   {
      Serial.print("Writing to test.txt...");
      myFile.println("testing 1, 2, 3.");
      // close the file:
      myFile.close();
      Serial.println("done.");
   }
   else
   {
      // if the file didn't open, print an error:
      Serial.println("error opening test.txt");
   }

   // re-open the file for reading:
   myFile = SD.open("test.txt");
   if (myFile)
   {
      Serial.println("test.txt:");

      // read from the file until there's nothing else in it:
      while (myFile.available())
      {
         Serial.write(myFile.read());
      }
      // close the file:
      myFile.close();
   }
   else
   {
      // if the file didn't open, print an error:
      Serial.println("error opening test.txt");
   }
}

void loop()
{
   // nothing happens after setup
}
*/

//____________________________________________________________________________________________

/*
#include <SdFat.h>

// Pick an SPI configuration.
// See SPI configuration section below (comments are for photon).
#define SPI_CONFIGURATION 3
//------------------------------------------------------------------------------
// Setup SPI configuration.
#if SPI_CONFIGURATION == 0
// Primary SPI with DMA
// SCK => A3, MISO => A4, MOSI => A5, SS => A2 (default)
SdFat sd;
const uint8_t chipSelect = SS;
#elif SPI_CONFIGURATION == 1
// Secondary SPI with DMA
// SCK => D4, MISO => D3, MOSI => D2, SS => D1
SdFat sd(1);
const uint8_t chipSelect = D1;
#elif SPI_CONFIGURATION == 2
// Primary SPI with Arduino SPI library style byte I/O.
// SCK => A3, MISO => A4, MOSI => A5, SS => A2 (default)
SdFatLibSpi sd;
const uint8_t chipSelect = SS;
#elif SPI_CONFIGURATION == 3
// Software SPI.  Use any digital pins.
// MISO => D5, MOSI => D6, SCK => D7, SS => D0
SdFat sd;
const uint8_t chipSelect = D5;
#endif  // SPI_CONFIGURATION
//------------------------------------------------------------------------------

File myFile;

void setup() {
  Serial.begin(9600);
  SPI2.begin(D5);
  // Wait for USB Serial 
  while (!Serial) {
    SysCall::yield();
  }
  
  Serial.println("Type any character to start");
  while (Serial.read() <= 0) {
    SysCall::yield();
  }

  // Initialize SdFat or print a detailed error message and halt
  // Use half speed like the native library.
  // Change to SPI_FULL_SPEED for more performance.
  if (!sd.begin(chipSelect, SPI_HALF_SPEED)) {
    sd.initErrorHalt();
  }

  // open the file for write at end like the "Native SD library"
  if (!myFile.open("test.txt", O_RDWR | O_CREAT | O_AT_END)) {
    sd.errorHalt("opening test.txt for write failed");
  }
  // if the file opened okay, write to it:
  Serial.print("Writing to test.txt...");
  myFile.println("testing 1, 2, 3.");
  myFile.printf("fileSize: %d\n", myFile.fileSize());
  
  // close the file:
  myFile.close();
  Serial.println("done.");

  // re-open the file for reading:
  if (!myFile.open("test.txt", O_READ)) {
    sd.errorHalt("opening test.txt for read failed");
  }
  Serial.println("test.txt content:");

  // read from the file until there's nothing else in it:
  int data;
  while ((data = myFile.read()) >= 0) {
    Serial.write(data);
  }
  // close the file:
  myFile.close();
}

void loop() {
  // nothing happens after setup
}

*/
