#ifndef IIS3DWB_FUNC_H
#define IIS3DWB_FUNC_H


/*Defines*/
#define SPI_MISO    13
#define SPI_MOSI    11
#define SPI_SCK     12
#define SPI_CS      10

#define INT1        2
#define INT2        7
#define AccelLed    6
#define ReadyLed    5
#define InitLed    4

extern int maxPacketSize;
extern int bufferSize;
extern byte* buffer;
void iis_init(bool sendAllAxes);
void getData(float *data, bool *valid);
void sendData(bool full_odr, bool sendAllAxes,int axis, int maxPacketSize, byte* buffer);
void iis_calibration();
void readDataAndAccumulate(bool sendAllAxes, int collectionInterval,int axis, int maxPacketSize);
#endif /*IIS3DWB_FUNC_H*/
