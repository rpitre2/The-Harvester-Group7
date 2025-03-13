// rgb_sensor.c
#include "colour_sensor.h"
#include "i2c.h"
  
void ColourSensor_Enable()
{
    I2C_WriteByte(APDS_I2C_ADDRESS, APDS_ENABLE, (APDS_PON | APDS_AEN));
}

ColourReading ColourSensor_ReadColours()
{
    ColourReading reading;
    
    uint16_t red  = I2C_ReadByte(APDS_I2C_ADDRESS, APDS_RDATAH) << 8;
    red = reading.red_channel | I2C_ReadByte(APDS_I2C_ADDRESS, APDS_RDATAL);
   
    
    reading.green_channel = I2C_ReadByte(APDS_I2C_ADDRESS, APDS_GDATAH) << 8;
    reading.green_channel = reading.green_channel | I2C_ReadByte(APDS_I2C_ADDRESS, APDS_GDATAL);
    
    reading.blue_channel = I2C_ReadByte(APDS_I2C_ADDRESS, APDS_BDATAH) << 8;
    reading.blue_channel = reading.blue_channel | I2C_ReadByte(APDS_I2C_ADDRESS, APDS_BDATAL);
    
    reading.red_channel = red;
    return reading;
}