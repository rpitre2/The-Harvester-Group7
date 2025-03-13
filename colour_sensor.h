#ifndef COLOUR_SENSOR_H
#include <xc.h>


#define APDS_I2C_ADDRESS 0x39 // address of rgb sensor
#define APDS_ENABLE   0x80 // ENABLE register
#define APDS_ATIME    0x82 // ATIME register
#define APDS_CONTROL  0x8F // CONTROL register

#define APDS_PON      0x01 // PON bit value (power on)
#define APDS_AEN      0x02 // AEN bit value (rgb enable)

#define APDS_ADC      0xDB // ALS ADC value (103ms integration time)
#define APDS_GAIN     0x01 // AEN value (4x gain)
#define APDS_CLEARL   0x94 // clear data low byte
#define APDS_CLEARH   0x95 // clear data high byte

#define APDS_RDATAL     0x96 // red data low byte
#define APDS_RDATAH     0x97 // red data high byte
#define APDS_GDATAL   0x98 // green data low byte
#define APDS_GDATAH   0x99 // green data high byte
#define APDS_BDATAL    0x9A // blue data low byte
#define APDS_BDATAH    0x9B // blue data high byte
#define APDS_STATUS   0x93 // status register


#define COLOUR_THRESHOLD -1

typedef struct _ColourReading
{
    uint16_t red_channel;
    uint16_t green_channel;
    uint16_t blue_channel;
} ColourReading;

void ColourSensor_Enable();

ColourReading ColourSensor_ReadColours();

#define COLOUR_SENSOR_H
#endif