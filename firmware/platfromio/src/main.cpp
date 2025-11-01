#include <Arduino.h>
#include <IIS3WDB_Func.h>
#include <ADS_Func.h>
#include <ISM330DLC_Func.h>
#include <ArduinoJson.h>
#include "esp_system.h"

// Define the start byte
String START_BYTE = "start";
String RESET_BYTE = "reset";
#define DATA_RATE 1 // data points per second
bool initialize_data = false;

unsigned long lastReadingTime = 0; // variable to keep track of the last time a reading was taken

//Sensor activation
bool High_Freq_acc_sensor = true;
bool ADS_active = false;
bool ISM330_active = false;

//mode selection based variable
int axis = 2;  // 0 for x, 1 for y, 2 for z
bool sendAllAxes = true; // To control whether to send all axes data or only selected axis
bool full_odr = true;

// Global variables or modify as needed to fit your design
// 1 for every data point, 2 for every 2nd, 4 for every 4th
unsigned int collectionInterval = 4; 

void setup() 
{
  Serial.begin(115200);
  pinMode(InitLed,OUTPUT);
  pinMode(ReadyLed,OUTPUT);
  pinMode(AccelLed,OUTPUT);
  // digitalWrite(ReadyLed,HIGH);
  // put your setup code here, to run once
  Serial.println(" starting...");  // Add a debug print

  if(High_Freq_acc_sensor == true)
  {
    iis_init(sendAllAxes); //includes calibration/ self_test_calibration
  }

  else if(ADS_active==true)
  {
    //Init ADS
    ADSinit();
  }

  else if(ISM330_active==true)
  {
    init_ISM330DLC();
  }

  else 
  {
    //Do nothing
  }
}

void loop()
{
  // Read serial data from USB
  while (Serial.available()>0)
  {
    String receivedString = Serial.readStringUntil('\n');  // read the incoming string,
    Serial.println( "You sent: " + receivedString );  // and echo it back.

    if (receivedString == START_BYTE) 
    {  // If the received char is the start byte,
      Serial.println("yipikayay");
      initialize_data = true;
      delay(1000); // Add a delay to allow the system to perform the restart
    }

    if (receivedString == "mode_1")  //RMS, STD Values
    {
      full_odr = false;
      sendAllAxes = true;
      collectionInterval = 1;
      maxPacketSize = 75000;
      bufferSize = 150000;
      
      if(buffer != NULL) {
        free(buffer);  // Free the previously allocated memory for buffer
        buffer = NULL;  // Reset the pointer to NULL
      }

      buffer = (byte*) malloc(bufferSize);  // Allocate memory

      if (buffer == NULL) {
        Serial.println("Error");
      }
      delay(1000);
    }

    if (receivedString == "mode_2")  //Full ODR, all axes
    {
      full_odr = true;
      sendAllAxes = true;
      collectionInterval = 1;
      maxPacketSize = 75000;
      bufferSize = 150000;
      
      if(buffer != NULL) {
        free(buffer);  // Free the previously allocated memory for buffer
        buffer = NULL;  // Reset the pointer to NULL
      }

      buffer = (byte*) malloc(bufferSize);  // Allocate memory

      if (buffer == NULL) {
        Serial.println("Error");
      }
      delay(1000);
    }

    if (receivedString == "mode_3")  //half ODR, all axes
    {
      full_odr = true;
      sendAllAxes = true;
      collectionInterval = 2;
      maxPacketSize = 75000/2;
      bufferSize = 150000/2;
      if(buffer != NULL) {
        free(buffer);  // Free the previously allocated memory for buffer
        buffer = NULL;  // Reset the pointer to NULL
      }

      buffer = (byte*) malloc(bufferSize);  // Allocate memory

      if (buffer == NULL) {
        Serial.println("Error");
      }
      delay(1000);
    }

    if (receivedString == "mode_4")  //quarter ODR, all axes
    {
      full_odr = true;
      sendAllAxes = true;
      collectionInterval = 4;
      maxPacketSize = 75000/4;
      bufferSize = 150000/4;
      if(buffer != NULL) {
        free(buffer);  // Free the previously allocated memory for buffer
        buffer = NULL;  // Reset the pointer to NULL
      }

      buffer = (byte*) malloc(bufferSize);  // Allocate memory

      if (buffer == NULL) {
        Serial.println("Error");
      }
      delay(1000);
    }

    if (receivedString == "reset") { //reset ESP32
      Serial.println(" restarting...");  // Add a debug print
      esp_restart();
    } 
  }

  if((High_Freq_acc_sensor == true && initialize_data == true))
  {
    // Read data from accelerometer, accumulate and send it
    readDataAndAccumulate(sendAllAxes,collectionInterval,axis, maxPacketSize);

    // Send data to the master device
    sendData(full_odr,sendAllAxes,axis,maxPacketSize,buffer);
  }

  else if(ADS_active==true)
  {
    String data_ads = ADSread();
    if (data_ads.length() > 0)
     {
      Serial.println(data_ads);
     }
	  //delay(100); // Adjust delay as needed
  }
  
  else if(ISM330_active==true)
  {
    //send data of ISM330DLC
    // Serial.println("Failed to initialize the ISM330DLC sensor!");
    readDataAndAccumulate_ISM330DLC();
    sendData_ISM330DLC();
    
  }

    else 
    {
      //Do nothing
 //     Serial.println("do nothing");
    }
}
