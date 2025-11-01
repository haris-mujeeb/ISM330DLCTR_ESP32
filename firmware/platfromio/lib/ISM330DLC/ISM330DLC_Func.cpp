#include "ISM330DLC_Func.h"
#include <Arduino.h>
#include "ISM330DLCSensor.h"
#include "debug.h"

// Pin number for attached interrupt to sensor
#define INT1_ISM330        15
#define INT2_ISM330        16

//interrupt for wake_up pin
void wakeUp_ISM330(void);

//Flag for dataready which detects new of samples avalaible
bool DataReady_ISM330 = false;

//Flag to wakeup sesnor
bool Wakeup_ISM330 = false;

// 0 for x, 1 for y, 2 for z
int axis_ISM330 = 0; 
// To control whether to send all axes data or only selected axis
bool sendAllAxes_ISM330 = true; 

//short datatype to send data into bytes
short* dataPacket_ISM330DLC;

//Dynamic datasize to send via serial port based on axis selection it will be setup
int maxPacketSize_ISM330DLC = 0;
int maxPacketSize_ISM330DLC_all_axis  = 1000;
int maxPacketSize_ISM330DLC_single_axis = 1000;

//Dynamic packetsize to accumulate in one go, based on axis selection it will be setup
int packetSize_ISM330DLC = 0;
int bufferSize_ISM330DLC_all_axis  = 2000;
int bufferSize_ISM330DLC_single_axis = 2000;

//byte to write to serial port
byte* buffer_ISM330DLC = NULL;

//Boolean to set data for full oder or not
bool full_odr_ISM330DLC = true;

// Uncomment to set I2C pins to use else default instance will be used
#define ISM330DLC_I2C_SCL  9
#define ISM330DLC_I2C_SDA  8
#define ISM330DLC_I2C_CLK  400000

// Global variables
//unsigned long lastMillis_ISM330DLC = 0;
int enterCounter_drdy = 0;
int enterCounter_ISM330DLC = 0;

//Varible related to configure accelerometer
float value_X_ODR_ISM330DLC = 6500.0f;
float value_X_FS_ISM330DLC =2.0f;

//Init for accelrometer data array
int32_t accelerometer_ISM330[3]= {0,0,0};

// offset biases for the accel 
int32_t accelBias_ISM330[3] = {0,0,0}; 

 //make 6 when going for accel and gyro both
int8_t acc_g_array_ISM330DLC = 3;

// Using the default I2C bus of the ESP32
TwoWire MyWire(0);  

//ISM330DLCSensor sensor(&MyWire, ISM330DLC_ACC_GYRO_I2C_ADDRESS_HIGH);  // Using the address 0x6A (7-bit format)
ISM330DLCSensor imu_ISM330(&MyWire, ISM330DLC_ACC_GYRO_I2C_ADDRESS_LOW);

/*******************************************************/
/*   Debugging purpose variables declarations here     */
unsigned long lastMillis_ISM330DLC, lastMillis_drdy = 0;
/*******************************************************/


/*******************************************************************************
* interrupt functions for the pins connected to ism330dlc
*******************************************************************************/
// This function will be called when the interrupt is triggered
void IRAM_ATTR isr() {
  // Handle the interrupt
  // Note: Keep this function short and fast; avoid complex operations here
  DataReady_ISM330 = true;
}

void wakeUp_ISM330() {
  Wakeup_ISM330 = true;
}

/*******************************************************************************
* Function to set dynamic datapacket based on input of send all axis or single axis
*******************************************************************************/

void initializeDataPacket_ism330dlc(bool sendAllAxes_ISM330DLC) {
  if(dataPacket_ISM330DLC != NULL) {  // Check if dataPacket already points to an allocated block of memory
    free(dataPacket_ISM330DLC);  // Free the previously allocated memory
    dataPacket_ISM330DLC = NULL;  // Reset the pointer to NULL
  }

  if(buffer_ISM330DLC != NULL) {
    free(buffer_ISM330DLC);  // Free the previously allocated memory for buffer
    buffer_ISM330DLC = NULL;  // Reset the pointer to NULL
  }
  
  if(sendAllAxes_ISM330DLC) {
    maxPacketSize_ISM330DLC = maxPacketSize_ISM330DLC_all_axis;
  } else {
    maxPacketSize_ISM330DLC = maxPacketSize_ISM330DLC_single_axis;
  }
  
  dataPacket_ISM330DLC = (short*) malloc(maxPacketSize_ISM330DLC * sizeof(short));  // Allocate memory
  
  if(dataPacket_ISM330DLC == NULL) {
    Serial.println("Failed to allocate memory for dataPacket");
    while(1);  // Trap the program here in case of a memory allocation failure
  }

  int bufferSize_ISM330DLC;
  if(sendAllAxes_ISM330DLC) {
    bufferSize_ISM330DLC = bufferSize_ISM330DLC_all_axis;
  } else {
    bufferSize_ISM330DLC = bufferSize_ISM330DLC_single_axis;
  }

  buffer_ISM330DLC = (byte*) malloc(bufferSize_ISM330DLC);  // Allocate memory

  if(buffer_ISM330DLC == NULL) {
    Serial.println("Failed to allocate memory for buffer");
    while(1);  // Trap the program here in case of a memory allocation failure
  }
}

/*******************************************************************************
* Function to set calibrate sensor
*******************************************************************************/
void calibrateAccelerometer_ISM330() {
  int32_t accelData_ISM330[3];

  for (int i = 0; i < 1000; i++) 
  {
    imu_ISM330.Get_X_Axes(accelData_ISM330);
    accelBias_ISM330[0] += accelData_ISM330[0];
    accelBias_ISM330[1] += accelData_ISM330[1];
    accelBias_ISM330[2] += accelData_ISM330[2];
    delay(1); 
  }
}

/*******************************************************************************
* Global Function to initialize sensor and it's other configurations
*******************************************************************************/
void init_ISM330DLC(void) {

  MyWire.begin(ISM330DLC_I2C_SDA, ISM330DLC_I2C_SCL, ISM330DLC_I2C_CLK);  // SDA=8, SCL=9
  imu_ISM330.begin();
  initializeDataPacket_ism330dlc(sendAllAxes_ISM330);
 
  while (imu_ISM330.begin() != ISM330DLC_STATUS_OK) {
    Serial.println("Failed to initialize the ISM330DLC sensor! Retrying...");
    delay(1000); // Wait for 1 second before retrying. This prevents rapid spamming and allows potential issues like voltage stabilization.
  }
delay(2000);
debugPrint("ISM330DLC sensor initialized successfully! ","");

imu_ISM330.Enable_DataReady_Duration();

imu_ISM330.Enable_X();
imu_ISM330.Disable_G();
delay(1000);
imu_ISM330.Set_X_ODR(value_X_ODR_ISM330DLC);
imu_ISM330.Set_X_FS(value_X_FS_ISM330DLC);
delay(3000);

pinMode(INT1_ISM330, INPUT); // enable ISM330DLC interrupt1
pinMode(INT2_ISM330, INPUT); // enable ISM330DLC interrupt2
Serial.println("START_AXIS_DATA");

attachInterrupt(INT1_ISM330, isr, RISING); // define interrupt for intPin1 output of ISM330DLC
attachInterrupt(INT2_ISM330, wakeUp_ISM330, FALLING);   // define interrupt for intPin2 output of ISM330DLC

DataReady_ISM330 = false;
Wakeup_ISM330 = false;

}

/*******************************************************************************
* Local Function to compute standard deviation
*******************************************************************************/

float computeStandardDeviation_ISM330DLC(short *data_ISM330DLC, int size_ISM330DLC, int step_ISM330DLC) {
    float mean_ISM330DLC = 0.0;
    for (int i = 0; i < size_ISM330DLC; i += step_ISM330DLC) {
        mean_ISM330DLC += data_ISM330DLC[i];
    }
    mean_ISM330DLC /= (size_ISM330DLC / step_ISM330DLC);  // Divide by number of data points, not the length of the array

    float variance_ISM330DLC = 0.0;
    for (int i = 0; i < size_ISM330DLC; i += step_ISM330DLC) {
        variance_ISM330DLC += (data_ISM330DLC[i] - mean_ISM330DLC) * (data_ISM330DLC[i] - mean_ISM330DLC);
    }
    variance_ISM330DLC /= (size_ISM330DLC / step_ISM330DLC);  // Divide by number of data points, not the length of the array

    return sqrt(variance_ISM330DLC);
}

/*******************************************************************************
* Local Function to compute rms
*******************************************************************************/
float computeRMS_ISM330DLC(short *data_ISM330DLC, int size_ISM330DLC, int step_ISM330DLC) {
    float sumOfSquares_ISM330DLC = 0.0;
  
    for (int i = 0; i < size_ISM330DLC; i += step_ISM330DLC) {
        sumOfSquares_ISM330DLC += data_ISM330DLC[i] * data_ISM330DLC[i];
    }
  
    float meanSquare_ISM330DLC = sumOfSquares_ISM330DLC / (size_ISM330DLC / step_ISM330DLC);  // Divide by number of data points, not the length of the array
  
    return sqrt(meanSquare_ISM330DLC);
}

/*******************************************************************************
* Local Function to get data for ism330dlc
*******************************************************************************/
void getData_ISM330(float *data_ISM330, bool *valid_ISM330) {
    
  checkIntervalAndPrint(lastMillis_drdy, 1000, "Entered if condition for Data ready: ", enterCounter_drdy);

  if (DataReady_ISM330) 
  {
      DataReady_ISM330 = false;
      
      *valid_ISM330 = true;
      imu_ISM330.Get_X_Axes(accelerometer_ISM330);
    
      // Apply accelerometer calibration offsets
      data_ISM330[0] = ((float)accelerometer_ISM330[0] - accelBias_ISM330[0]) ;
      data_ISM330[1] = ((float)accelerometer_ISM330[1] - accelBias_ISM330[1]) ;
      data_ISM330[2] = ((float)accelerometer_ISM330[2] - accelBias_ISM330[2]) ;
  }
  checkIntervalAndPrint(lastMillis_ISM330DLC, 1000, "Entered if condition after Data ready: ", enterCounter_ISM330DLC);
}

/*******************************************************************************
* Global Function to send accumulated data after computing odr/ std/ rms to serial port
*******************************************************************************/
void sendData_ISM330DLC() {
    if (packetSize_ISM330DLC >= maxPacketSize_ISM330DLC) {
        int bufferIndex_ISM330DLC = 0;
 
    if(full_odr_ISM330DLC == true)
    {
            if (sendAllAxes_ISM330) {
                // Correctly adding individual bytes to the buffer
                buffer_ISM330DLC[bufferIndex_ISM330DLC++] = 0xAA;  // Header byte
                buffer_ISM330DLC[bufferIndex_ISM330DLC++] = 0x55;  // Axis byte

                // Assuming dataPacket contains float values (4 bytes each)
                for (int i = 0; i < packetSize_ISM330DLC; i += 3) {
                    // Assuming dataPacket is an array of floats
                    memcpy(&buffer_ISM330DLC[bufferIndex_ISM330DLC], &dataPacket_ISM330DLC[i], sizeof(dataPacket_ISM330DLC[i]));
                    bufferIndex_ISM330DLC += sizeof(dataPacket_ISM330DLC[i]);

                    memcpy(&buffer_ISM330DLC[bufferIndex_ISM330DLC], &dataPacket_ISM330DLC[i + 1], sizeof(dataPacket_ISM330DLC[i+1]));
                    bufferIndex_ISM330DLC += sizeof(dataPacket_ISM330DLC[i+1]);

                    memcpy(&buffer_ISM330DLC[bufferIndex_ISM330DLC], &dataPacket_ISM330DLC[i + 2], sizeof(dataPacket_ISM330DLC[i+2]));
                    bufferIndex_ISM330DLC += sizeof(dataPacket_ISM330DLC[i+2]);
                }
            } else {
                // For single axis
                buffer_ISM330DLC[bufferIndex_ISM330DLC++] = 0xAB;
                buffer_ISM330DLC[bufferIndex_ISM330DLC++] = axis_ISM330;

                for (int i = 0; i < packetSize_ISM330DLC; i++) {
                    memcpy(&buffer_ISM330DLC[bufferIndex_ISM330DLC], &dataPacket_ISM330DLC[i], sizeof(dataPacket_ISM330DLC[i]));
                    bufferIndex_ISM330DLC += sizeof(dataPacket_ISM330DLC[i]);
                }
            }
    }
    else if(full_odr_ISM330DLC == false)
    {
        if (sendAllAxes_ISM330) 
        {
            buffer_ISM330DLC[bufferIndex_ISM330DLC++] = 0xBA;  // Header for rms_std and 3 axes - first byte
            buffer_ISM330DLC[bufferIndex_ISM330DLC++] = 0x56;  // Header for rms_std and 3 axes - second byte
            // Compute standard deviation and RMS for all axes
            float stdDeviationX_ISM330DLC = computeStandardDeviation_ISM330DLC(dataPacket_ISM330DLC, packetSize_ISM330DLC / 3, 3);
            float rmsX_ISM330DLC = computeRMS_ISM330DLC(dataPacket_ISM330DLC, packetSize_ISM330DLC / 3, 3);
            float stdDeviationY_ISM330DLC = computeStandardDeviation_ISM330DLC(dataPacket_ISM330DLC + 1, packetSize_ISM330DLC / 3, 3);
            float rmsY_ISM330DLC = computeRMS_ISM330DLC(dataPacket_ISM330DLC + 1, packetSize_ISM330DLC / 3, 3);
            float stdDeviationZ_ISM330DLC = computeStandardDeviation_ISM330DLC(dataPacket_ISM330DLC + 2, packetSize_ISM330DLC / 3, 3);
            float rmsZ_ISM330DLC = computeRMS_ISM330DLC(dataPacket_ISM330DLC + 2, packetSize_ISM330DLC / 3, 3);
 
          // Add standard deviation and RMS for all axes to buffer in byte format - Data pattern rmsX, rmsY, rmsZ, stdDeviationX, stdDeviationY, stdDeviationZ
            memcpy(&buffer_ISM330DLC[bufferIndex_ISM330DLC], &rmsX_ISM330DLC, sizeof(rmsX_ISM330DLC));
            bufferIndex_ISM330DLC += sizeof(rmsX_ISM330DLC);

            memcpy(&buffer_ISM330DLC[bufferIndex_ISM330DLC], &rmsY_ISM330DLC, sizeof(rmsY_ISM330DLC));
            bufferIndex_ISM330DLC += sizeof(rmsY_ISM330DLC);

            memcpy(&buffer_ISM330DLC[bufferIndex_ISM330DLC], &rmsZ_ISM330DLC, sizeof(rmsZ_ISM330DLC));
            bufferIndex_ISM330DLC += sizeof(rmsZ_ISM330DLC);

            memcpy(&buffer_ISM330DLC[bufferIndex_ISM330DLC], &stdDeviationX_ISM330DLC, sizeof(stdDeviationX_ISM330DLC));
            bufferIndex_ISM330DLC += sizeof(stdDeviationX_ISM330DLC);

            memcpy(&buffer_ISM330DLC[bufferIndex_ISM330DLC], &stdDeviationY_ISM330DLC, sizeof(stdDeviationY_ISM330DLC));
            bufferIndex_ISM330DLC += sizeof(stdDeviationY_ISM330DLC);

            memcpy(&buffer_ISM330DLC[bufferIndex_ISM330DLC], &stdDeviationZ_ISM330DLC, sizeof(stdDeviationZ_ISM330DLC));
            bufferIndex_ISM330DLC += sizeof(stdDeviationZ_ISM330DLC); 
            //Serial.print("stdDeviationZ: "); Serial.print(stdDeviationZ); Serial.println();
        }
        else 
        {
                // Compute standard deviation and RMS for selected axis
                float stdDeviation_ISM330DLC;
                float rms_ISM330DLC;
                stdDeviation_ISM330DLC = computeStandardDeviation_ISM330DLC(dataPacket_ISM330DLC, packetSize_ISM330DLC, 3);
                rms_ISM330DLC = computeRMS_ISM330DLC(dataPacket_ISM330DLC, packetSize_ISM330DLC, 3);
                 

                // Header for a single axis
                buffer_ISM330DLC[bufferIndex_ISM330DLC++] = 0xBB;  // Header for rms_std and 1 axis - first byte
                buffer_ISM330DLC[bufferIndex_ISM330DLC++] = axis_ISM330;  // The second byte will represent the axis (0, 1, or 2)

                // Add standard deviation and RMS for the selected axis to buffer in byte format - Data pattern rms, stdDeviation
                 memcpy(&buffer_ISM330DLC[bufferIndex_ISM330DLC], &rms_ISM330DLC, sizeof(rms_ISM330DLC));
                 bufferIndex_ISM330DLC += sizeof(rms_ISM330DLC);
                // Serial.print(sizeof(rms));

                 memcpy(&buffer_ISM330DLC[bufferIndex_ISM330DLC], &stdDeviation_ISM330DLC, sizeof(stdDeviation_ISM330DLC));
                 bufferIndex_ISM330DLC += sizeof(stdDeviation_ISM330DLC);
        }
    }
	
	else 
	{
		
	}

    // Print the buffer content
    Serial.write(buffer_ISM330DLC, bufferIndex_ISM330DLC);
    
    // Reset packet size
    packetSize_ISM330DLC = 0;
  }
}

/*******************************************************************************
* Global Function to read data from a sensor and accumulate
*******************************************************************************/
void readDataAndAccumulate_ISM330DLC() {
  float data_ISM330[acc_g_array_ISM330DLC]; // Array to store both accelerometer and gyroscope data
  bool valid_ISM330;
  
 while(packetSize_ISM330DLC < maxPacketSize_ISM330DLC)
  {
    getData_ISM330(data_ISM330, &valid_ISM330);
    if(valid_ISM330)
    {
      if(sendAllAxes_ISM330 && (packetSize_ISM330DLC < maxPacketSize_ISM330DLC))
      {
        dataPacket_ISM330DLC[packetSize_ISM330DLC++] = data_ISM330[0] ;
        dataPacket_ISM330DLC[packetSize_ISM330DLC++] = data_ISM330[1] ;
        dataPacket_ISM330DLC[packetSize_ISM330DLC++] = data_ISM330[2] ;
      }
      else if(!sendAllAxes_ISM330 && packetSize_ISM330DLC < maxPacketSize_ISM330DLC)
      {
        dataPacket_ISM330DLC[packetSize_ISM330DLC++] = data_ISM330[0];
      }
    }
  }
}

