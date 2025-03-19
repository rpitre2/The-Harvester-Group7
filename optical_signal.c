/*
 * File:   optical_signal.c
 * Author: willm
 *
 * Created on February 3, 2025, 2:53 PM
 */

#include "optical_signal.h"

//#define EnableGlobalInterrupt() (INTCONbits.GIE = 1)
//#define DisableGloablInterupt() (INTCONbits.GIE = 0)
//
//#define EnablePeripheralInterrupt() (INTCONbits.PEIE = 1)

void OSD_Read_Colours(void)
{    
    ColourSensor_Enable();
    ColourReading read = ColourSensor_ReadColours();

    if(read.red_channel > read.green_channel && read.red_channel > read.blue_channel)
    {
        LATAbits.LATA0 = 1;
        LATAbits.LATA1 = 0;
        LATAbits.LATA2 = 0;
        PWM1_Initialize(PWM_PERIOD_RED, PWM_PRESCALER);
        PWM1_SetDuty(512); // 0-1024
    }
    else if(read.green_channel > read.red_channel && read.green_channel > read.blue_channel)
    {
        LATAbits.LATA0 = 0;
        LATAbits.LATA1 = 1;
        LATAbits.LATA2 = 0;
        PWM1_Initialize(PWM_PERIOD_GREEN, PWM_PRESCALER);
        PWM1_SetDuty(512); // 0-1024
    }
    else if(read.blue_channel > read.red_channel && read.blue_channel > read.green_channel)
    {
        LATAbits.LATA0 = 0;
        LATAbits.LATA1 = 0;
        LATAbits.LATA2 = 1;
        PWM1_Initialize(PWM_PERIOD_BLUE, PWM_PRESCALER);
        PWM1_SetDuty(512); // 0-1024
    }
    else
    {
        LATAbits.LATA0 = 0;
        LATAbits.LATA1 = 0;
        LATAbits.LATA2 = 0;
        PWM1_SetDuty(0);
    }


    LATAbits.LATA3 = 1;
}
