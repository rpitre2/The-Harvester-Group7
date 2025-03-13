// i2c.h
#pragma once
#ifndef I2C_H
#define I2C_H
 
#include <xc.h>
 
#define _XTAL_FREQ 32000000 // system oscillator frequency (Fosc)
#define I2C_SCL_SPEED 400000

void I2C_Init();
void I2C_Start();
void I2C_Write(uint8_t data);
uint8_t I2C_WriteByte(uint8_t address, uint8_t reg, uint8_t data);
uint8_t I2C_Read();
uint8_t I2C_ReadByte(uint8_t address, uint8_t reg);
uint8_t I2C_ACK_Status();
void I2C_NACK();
void I2C_RepeatedStart();
void I2C_Stop();
#endif