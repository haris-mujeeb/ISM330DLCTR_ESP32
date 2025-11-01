#include <ADS1X15.h>
#include <ADS_Func.h>
#include <Wire.h>
#include "ArduinoJson.h"

ADS1114 ADS(0x48);
#define DATA_POINTS 256  // or 512, depending on how many data points you want

struct AdsData {
  int16_t val_0;
  int16_t val_1;
  float voltage;
};

static AdsData adsData[DATA_POINTS];
static int dataIndex = 0;

void ADSinit(void)
{
    ADS.begin();
    ADS.setMode(1);
    ADS.setDataRate(7);
    ADS.setGain(2);
    //Wire.end();
}

String ADSread(void)
{
  if (dataIndex < DATA_POINTS) {
    // Read and store data
    adsData[dataIndex].val_0 = ADS.readADC(0);
    //adsData[dataIndex].val_1 = ADS.readADC(1);
    //adsData[dataIndex].voltage = ADS.toVoltage(1);  // voltage factor
    dataIndex++;
    return "";  // Return an empty string
  } else {
    // Construct JSON formatted data
    String readings = "AdsData:[";
    for (int i = 0; i < DATA_POINTS; i++) {
      readings += String(adsData[i].val_0);
      if (i < DATA_POINTS - 1) {
        readings += ",";
      }
    }
    readings += "]";
    
    //Serial.print("Memory Usage: ");
    //Serial.println(sizeof(adsData) * DATA_POINTS);  // Print out memory usage

    dataIndex = 0;  // Reset index to start gathering data again

    return readings;
  }
}