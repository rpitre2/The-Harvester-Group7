// i2c.c
#include "i2c.h"
 
void I2C_Init() {    
    SSP2CON1bits.SSPEN = 0; // disable I2C
    
    //SDA (I and O) RC3
    SSP2DATPPS = 0x13;
    RC3PPS = 0x17;
    TRISCbits.TRISC3 = 1; // input
    ANSELCbits.ANSC3 = 0; // digital
    
    //SCL (I and O) RC4
    SSP2CLKPPS = 0x14;
    RC4PPS = 0x16;
    TRISCbits.TRISC4 = 1; // input
    ANSELCbits.ANSC4 = 0; // digital
    
    SSP2CON1bits.SSPM = 0b1000;
    SSP2ADD = (_XTAL_FREQ / (4 * I2C_SCL_SPEED)) - 1;;//0x13; // decimal 19
    
    SSP2CON1bits.SSPEN = 1; // enable I2C
}
 
void I2C_Start()
{
    SSP2CON2bits.SEN = 1;
    while(SSP2CON2bits.SEN);
}

void I2C_Write(uint8_t data)
{
    SSP2BUF = data;
    while(SSP2STATbits.BF);
}

uint8_t I2C_ACK_Status()
{
    return !SSP2CON2bits.ACKSTAT;
}

void I2C_Stop()
{
    SSP2CON2bits.PEN = 1;
    while(SSP2CON2bits.PEN);
}

void I2C_RepeatedStart()
{
    SSP2CON2bits.RSEN = 1;
    while(SSP2CON2bits.RSEN);
}

uint8_t I2C_Read()
{
    SSP2CON2bits.RCEN = 1;
    while(!SSP2STATbits.BF);
    return SSP2BUF;
}

void I2C_NACK()
{
    SSP2CON2bits.ACKDT = 1;
    SSP2CON2bits.ACKEN = 1;
    while(SSP2CON2bits.ACKEN);
}

uint8_t I2C_WriteByte(uint8_t address, uint8_t reg, uint8_t data)
{
    uint8_t ack = 1;
    
    I2C_Start();
//    __delay_ms(1);
    
    I2C_Write(address << 1);
    if(!I2C_ACK_Status())
    {
        ack = 0;
    }
    
//    __delay_ms(1);
        
    I2C_Write(reg);
    if(!I2C_ACK_Status())
    {
        ack = 0;
    }
//    __delay_ms(1);
   
    I2C_Write(data);
    if(!I2C_ACK_Status())
    {
        ack = 0;
    }
//    __delay_ms(1);
    
    I2C_Stop();
    
    return ack;
}

uint8_t I2C_ReadByte(uint8_t address, uint8_t reg)
{
    uint8_t ack = 1;
    
    I2C_Start();
//    __delay_ms(1);
    
    I2C_Write(address << 1);
    if(!I2C_ACK_Status())
    {
        ack = 0;
    }
//    __delay_ms(1);
        
    I2C_Write(reg);
    if(!I2C_ACK_Status())
    {
        ack = 0;
    }
//    __delay_ms(1);
    
    I2C_RepeatedStart();
//    __delay_ms(1);
    
    I2C_Write((address << 1) | 0x01);
    if(!I2C_ACK_Status())
    {
        ack = 0;
    }
//    __delay_ms(1);
    
    uint8_t data = I2C_Read();
    if(!I2C_ACK_Status())
    {
        ack = 0;
    }
//    __delay_ms(1);
    
    I2C_NACK();
//    __delay_ms(1);
    I2C_Stop();
//    __delay_ms(1);
    
    return data;
}