#include <Arduino.h>
#include <SPI.h>
#include "IIS3DWB.h"
#include "IIS3WDB_Func.h"
#include "debug.h"


short* dataPacket;
int packetSize = 0;
int maxPacketSize = 0;
byte* buffer = NULL;
int bufferSize = 0;

IIS3DWB IIS3DWB_ins(SPI_CS); // instantiate IIS3DWB class
uint8_t Ascale = AFS_4G;
float aRes;                              // scale resolutions per LSB for the accel 
float accelBias[3] = {0.0f, 0.0f, 0.0f}; // offset biases for the accel 
int16_t IIS3DWBData[4] = {0};                  // Stores the 16-bit signed sensor output
uint8_t IIS3DWBstatus;

/*******************************************************************************
* interrupt pins related declarations
*******************************************************************************/
void wakeUp(void);
void dataReady(void);
bool DataReady = false;
bool Wakeup = false;

/*******************************************************************************
* functions for initializing data pins as per axis selection
*******************************************************************************/
void initializeDataPacket(bool sendAllAxes) {
  if(dataPacket != NULL) {  // Check if dataPacket already points to an allocated block of memory
    free(dataPacket);  // Free the previously allocated memory
    dataPacket = NULL;  // Reset the pointer to NULL
  }

  
  if(buffer != NULL) {
    free(buffer);  // Free the previously allocated memory for buffer
    buffer = NULL;  // Reset the pointer to NULL
  }
  
  if(sendAllAxes) {
    maxPacketSize = 75000/4;
  } else {
    maxPacketSize = 25000/4;
  }
  
  dataPacket = (short*) malloc(maxPacketSize * sizeof(short));  // Allocate memory
  
  if(dataPacket == NULL) {
    debugPrint("Failed to allocate memory for dataPacket","");
    while(1);  // Trap the program here in case of a memory allocation failure
  }

  if(sendAllAxes) {
    bufferSize = 150000/4;
  } else {
    bufferSize = 50000/4;
  }

  buffer = (byte*) malloc(bufferSize);  // Allocate memory

  if(buffer == NULL) {
    debugPrint("Failed to allocate memory for buffer","");
    while(1);  // Trap the program here in case of a memory allocation failure
  }
}

/*******************************************************************************
* init function call upon bootup to set pins and other sensor configuration
*******************************************************************************/
void iis_init(bool sendAllAxes)
{
  pinMode(ReadyLed,OUTPUT);
  pinMode(InitLed,OUTPUT);
  digitalWrite(InitLed, HIGH);

  SPI.begin();
  pinMode(SPI_CS, OUTPUT);
  digitalWrite(SPI_CS,HIGH);

  initializeDataPacket(sendAllAxes);

    // Configure interrupt pins
  pinMode(INT1, INPUT); // enable IIS3DWB interrupt1
  pinMode(INT2, INPUT); // enable IIS3DWB interrupt2

  
  // debugPrint("Initializing","");
  // debugPrint("IIS3DWB accel...","");
  uint8_t c = IIS3DWB_ins.getChipID();  // Read CHIP_ID register for IIS3DWB
  //Serial.print("IIS3DWB "); Serial.print("I AM "); Serial.print(c, HEX); Serial.print(" I should be "); Serial.println(0x7B, HEX);
  //Serial.println(" ");

  if(c == 0x7B) // check if all SPI sensors have acknowledged
  {
    // debugPrint("IIS3DWB is online...","");  

    // reset IIS3DWB to start fresh
    IIS3DWB_ins.reset();
      
    digitalWrite(InitLed, HIGH); // indicate passed the ID check

    // get accel sensor resolution, only need to do this once
    aRes = IIS3DWB_ins.getAres(Ascale);

    IIS3DWB_ins.selfTest();

    IIS3DWB_ins.init(Ascale); // configure IIS3DWB  

    IIS3DWB_ins.offsetBias(accelBias);
    Serial.println("accel biases (mg)"); Serial.println(1000.0f * accelBias[0]); Serial.println(1000.0f * accelBias[1]); Serial.println(1000.0f * accelBias[2]);
    Serial.println(" ");
    // delay(1000); 
    debugPrint("START_AXIS_DATA","");
    Serial.println("START_AXIS_DATA");
    
    digitalWrite(InitLed, LOW); // turn off led when sensor configuration is finished
    digitalWrite(ReadyLed, HIGH);
  }
  else 
  {
    if(c != 0x6A) Serial.println(" IIS3DWB not functioning!"); 
    while(1){};
  }

  attachInterrupt(INT1, dataReady, RISING);   // define interrupt for intPin1 output of IIS3DWB
  attachInterrupt(INT2, wakeUp, FALLING);  // define interrupt for intPin2 output of IIS3DWB
}

/*******************************************************************************
* Local Function to compute standard deviation
*******************************************************************************/

 float computeStandardDeviation(short *data, int size, int step) {
     float mean = 0.0;
     for (int i = 0; i < size; i += step) {
         mean += data[i];
     }
     mean /= (size / step);  // Divide by number of data points, not the length of the array

     float variance = 0.0;
     for (int i = 0; i < size; i += step) {
         variance += (data[i] - mean) * (data[i] - mean);
     }
     variance /= (size / step);  // Divide by number of data points, not the length of the array

     return sqrt(variance);
 }

/*******************************************************************************
* Local Function to compute RMS
*******************************************************************************/

 float computeRMS(short *data, int size, int step) {
     float sumOfSquares = 0.0;
    
     for (int i = 0; i < size; i += step) {
         sumOfSquares += data[i] * data[i];
     }
    
     float meanSquare = sumOfSquares / (size / step);  // Divide by number of data points, not the length of the array
    
     return sqrt(meanSquare);
}

/*******************************************************************************
* Global Function to send accumulated data after computing odr/ std/ rms to serial port
*******************************************************************************/
void sendData(bool full_odr, bool sendAllAxes, int axis, int maxPacketSize, byte* buffer) {
    if (packetSize >= maxPacketSize) {
        int bufferIndex = 0;
    if(full_odr == true)
    {
      if (sendAllAxes) 
      {
          buffer[bufferIndex++] = 0xAA;  // Header for full_odr and 1 axis - first byte
          buffer[bufferIndex++] = 0x55;  // The second byte will represent the axis (0, 1, or 2)
          // Populate buffer with all axes data
          for (int i = 0; i < packetSize; i += 3) {
            memcpy(&buffer[bufferIndex], &dataPacket[i], sizeof(dataPacket[i]));
            bufferIndex += sizeof(dataPacket[i]);

            memcpy(&buffer[bufferIndex], &dataPacket[i + 1], sizeof(dataPacket[i + 1]));
            bufferIndex += sizeof(dataPacket[i + 1]);

            memcpy(&buffer[bufferIndex], &dataPacket[i + 2], sizeof(dataPacket[i + 2]));
            bufferIndex += sizeof(dataPacket[i + 2]);

            //buffer[bufferIndex++] = 0xFF;  // Delimiter
          }
      }
         else 
          {
            buffer[bufferIndex++] = 0xAB;  // Header for full_odr and 1 axis - first byte
            buffer[bufferIndex++] = axis;  // The second byte will represent the axis (0, 1, or 2)

            // Populate buffer with single axis data
            for (int i = 0; i < packetSize; i++) {
              memcpy(&buffer[bufferIndex], &dataPacket[i], sizeof(dataPacket[i]));
              bufferIndex += sizeof(dataPacket[i]);
            }
          }
    }
    else if(full_odr == false)
    {
        if (sendAllAxes) 
        {
            buffer[bufferIndex++] = 0xBA;  // Header for rms_std and 3 axes - first byte
            buffer[bufferIndex++] = 0x56;  // Header for rms_std and 3 axes - second byte
            // Compute standard deviation and RMS for all axes
            float stdDeviationX = computeStandardDeviation(dataPacket, packetSize / 3, 3);
            float rmsX = computeRMS(dataPacket, packetSize / 3, 3);
            float stdDeviationY = computeStandardDeviation(dataPacket + 1, packetSize / 3, 3);
            float rmsY = computeRMS(dataPacket + 1, packetSize / 3, 3);
            float stdDeviationZ = computeStandardDeviation(dataPacket + 2, packetSize / 3, 3);
            float rmsZ = computeRMS(dataPacket + 2, packetSize / 3, 3);

          // Headers for all axes
          

          // Add standard deviation and RMS for all axes to buffer in byte format - Data pattern rmsX, rmsY, rmsZ, stdDeviationX, stdDeviationY, stdDeviationZ
            memcpy(&buffer[bufferIndex], &rmsX, sizeof(rmsX));
            bufferIndex += sizeof(rmsX);

            memcpy(&buffer[bufferIndex], &rmsY, sizeof(rmsY));
            bufferIndex += sizeof(rmsY);

            memcpy(&buffer[bufferIndex], &rmsZ, sizeof(rmsZ));
            bufferIndex += sizeof(rmsZ);

            memcpy(&buffer[bufferIndex], &stdDeviationX, sizeof(stdDeviationX));
            bufferIndex += sizeof(stdDeviationX);

            memcpy(&buffer[bufferIndex], &stdDeviationY, sizeof(stdDeviationY));
            bufferIndex += sizeof(stdDeviationY);

            memcpy(&buffer[bufferIndex], &stdDeviationZ, sizeof(stdDeviationZ));
            bufferIndex += sizeof(stdDeviationZ); 
        }
        else 
        {
                // Compute standard deviation and RMS for selected axis
                float stdDeviation;
                float rms;
                stdDeviation = computeStandardDeviation(dataPacket, packetSize, 3);
                rms = computeRMS(dataPacket, packetSize, 3);

                // Header for a single axis
                buffer[bufferIndex++] = 0xBB;  // Header for rms_std and 1 axis - first byte
                buffer[bufferIndex++] = axis;  // The second byte will represent the axis (0, 1, or 2)

                // Add standard deviation and RMS for the selected axis to buffer in byte format - Data pattern rms, stdDeviation
                 memcpy(&buffer[bufferIndex], &rms, sizeof(rms));
                 bufferIndex += sizeof(rms);
                // Serial.print(sizeof(rms));

                 memcpy(&buffer[bufferIndex], &stdDeviation, sizeof(stdDeviation));
                 bufferIndex += sizeof(stdDeviation);
        }
    }

    else 
    {
      //Do nothing
    }
     // Send the entire buffer over Serial
     //debugPrint("packetsizemax_reached: ", String(packetSize));
     Serial.write(buffer, bufferIndex);
     //debugPrint("buffer_length: ", String((int)(sizeof(buffer)/sizeof(buffer[0]))));
     //debugPrint("bufferIndex_length: ", String(bufferIndex));
     //debugPrint("packetsizemax_reached: ", String(packetSize));
    
      // Reset packet size
      packetSize = 0;
    }
}

/*******************************************************************************
* Local Function to get data for iis3wdb
*******************************************************************************/

void getData(float *data, bool *valid, bool sendAllAxes,int axis)
{
  static unsigned long last_time = millis();
  static int counter = 0;
  if(DataReady)
  {
    digitalWrite(AccelLed,HIGH);
    DataReady = false;
    *valid = true;
    IIS3DWB_ins.readAccelData(IIS3DWBData);  
   
    if(sendAllAxes) {
      data[0] = 1000 * ((float)IIS3DWBData[0]*aRes );
      data[1] = 1000 * ((float)IIS3DWBData[1]*aRes );
      data[2] = 1000 * ((float)IIS3DWBData[2]*aRes );
    }
    else {
      data[0] = (int)1000 * ((float)IIS3DWBData[axis]*aRes);
    }

    digitalWrite(AccelLed,LOW);
  }

  else 
  {
      *valid = false;
  }
}

void dataReady()
{
  DataReady = true;
}


void wakeUp()
{
  Wakeup = true;
}

/*******************************************************************************
* Global Function to read data from a sensor and accumulate
*******************************************************************************/
void readDataAndAccumulate(bool sendAllAxes, int collectionInterval, int axis, int maxPacketSize)
{
  float data[3] = {0.0f, 0.0f, 0.0f};
  bool valid;
  // Counter to track data points processed
  static unsigned int dataPointCounter = 0; 
  
  //debugPrint("packetsize_readDataAndAccumulate: ", String(packetSize));
  while(packetSize < maxPacketSize)
  {
    getData(data, &valid,sendAllAxes,axis);
    if(valid)
    {
      // Increment counter each time valid data is received
      dataPointCounter++;
      //debugPrint("packetsize_valid_loop: ", String(packetSize));

      // Check if the current data point matches the collection interval
      if(dataPointCounter % collectionInterval == 0)
      {
        // Add data to packet based on mode (all axes or a specific axis)
        // Ensure there's room for all 3 data points
        if(sendAllAxes && (packetSize + 2 < maxPacketSize)) 
        {
          //debugPrint("packetsize_datapointer_counter: ", String(packetSize));
          dataPacket[packetSize++] = data[0];
          dataPacket[packetSize++] = data[1];
          dataPacket[packetSize++] = data[2];
        }
        else if(!sendAllAxes && packetSize < maxPacketSize)
        {
          // Assuming 'axis' variable determines which axis to send
          dataPacket[packetSize++] = data[0]; 
        }
      }
    }

    // If the counter reaches the collection interval, reset it
    if(dataPointCounter >= collectionInterval)
    {
      // Reset counter after matching the collection interval
      dataPointCounter = 0; 
    }
  }
   //debugPrint("packetsize_after_while: ", String(packetSize));
}
