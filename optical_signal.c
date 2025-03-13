/*
 * File:   optical_signal.c
 * Author: willm
 *
 * Created on February 3, 2025, 2:53 PM
 */

// CONFIG1
#pragma config FEXTOSC = OFF    // External Oscillator mode selection bits (Oscillator not enabled)
#pragma config RSTOSC = HFINT32 // Power-up default value for COSC bits (HFINTOSC with OSCFRQ= 32 MHz and CDIV = 1:1)
#pragma config CLKOUTEN = OFF   // Clock Out Enable bit (CLKOUT function is disabled; i/o or oscillator function on OSC2)
#pragma config CSWEN = ON       // Clock Switch Enable bit (Writing to NOSC and NDIV is allowed)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (FSCM timer disabled)

// CONFIG2
#pragma config MCLRE = ON       // Master Clear Enable bit (MCLR pin is Master Clear function)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config LPBOREN = OFF    // Low-Power BOR enable bit (ULPBOR disabled)
#pragma config BOREN = ON       // Brown-out reset enable bits (Brown-out Reset Enabled, SBOREN bit is ignored)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (VBOR) set to 1.9V on LF, and 2.45V on F Devices)
#pragma config ZCD = OFF        // Zero-cross detect disable (Zero-cross detect circuit is disabled at POR.)
#pragma config PPS1WAY = ON     // Peripheral Pin Select one-way control (The PPSLOCK bit can be cleared and set only once in software)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable bit (Stack Overflow or Underflow will cause a reset)

// CONFIG3
#pragma config WDTCPS = WDTCPS_31// WDT Period Select bits (Divider ratio 1:65536; software control of WDTPS)
#pragma config WDTE = OFF       // WDT operating mode (WDT Disabled, SWDTEN is ignored)
#pragma config WDTCWS = WDTCWS_7// WDT Window Select bits (window always open (100%); software control; keyed access not required)
#pragma config WDTCCS = SC      // WDT input clock selector (Software Control)

// CONFIG4
#pragma config WRT = OFF        // UserNVM self-write protection bits (Write protection off)
#pragma config SCANE = available// Scanner Enable bit (Scanner module is available for use)
#pragma config LVP = ON         // Low Voltage Programming Enable bit (Low Voltage programming enabled. MCLR/Vpp pin function is MCLR.)

// CONFIG5
#pragma config CP = OFF         // UserNVM Program memory code protection bit (Program Memory code protection disabled)
#pragma config CPD = OFF        // DataNVM code protection bit (Data EEPROM code protection disabled)

#include <xc.h>
#include "i2c.h"
#include "colour_sensor.h"
#include "buzzer.h"

#define EnableGlobalInterrupt() (INTCONbits.GIE = 1)
#define DisableGloablInterupt() (INTCONbits.GIE = 0)

#define EnablePeripheralInterrupt() (INTCONbits.PEIE = 1)

void notmain(void) 
{
    
//    EnableGlobalInterrupt();
//    EnablePeripheralInterrupt();
    

    
    
    // Set LED pins as outputs
    TRISAbits.TRISA0 = 0;
    TRISAbits.TRISA1 = 0;
    TRISAbits.TRISA2 = 0;
    TRISAbits.TRISA3 = 0;
    
    // Set LED pins to digital
    ANSELAbits.ANSA0 = 0;
    ANSELAbits.ANSA1 = 0;
    ANSELAbits.ANSA2 = 0;
    ANSELAbits.ANSA3 = 0;
    
    LATAbits.LATA0 = 0;
    LATAbits.LATA1 = 0;
    LATAbits.LATA2 = 0;
    LATAbits.LATA3 = 0;
    
    //Switch S2 setup
    TRISAbits.TRISA5 = 1; // input
    ANSELAbits.ANSA5 = 0; // digital
    
    I2C_Init();
    ColourSensor_Enable();
    
    while(1)
    {
        if(PORTAbits.RA5) // S2 on
        {   
//            ColourSensor_Enable();
            ColourReading read = ColourSensor_ReadColours();

            if(read.red_channel > read.green_channel && read.red_channel > read.blue_channel)
            {
                LATAbits.LATA0 = 1;
                LATAbits.LATA1 = 0;
                LATAbits.LATA2 = 0;
                PWM1_Initialize(PWM_PERIOD_RED, PWM_PRESCALER);
                PWM1_SetDuty(512); // 0-127
            }
            else if(read.green_channel > read.red_channel && read.green_channel > read.blue_channel)
            {
                LATAbits.LATA0 = 0;
                LATAbits.LATA1 = 1;
                LATAbits.LATA2 = 0;
                PWM1_Initialize(PWM_PERIOD_GREEN, PWM_PRESCALER);
                PWM1_SetDuty(512); // 0-127
            }
            else if(read.blue_channel > read.red_channel && read.blue_channel > read.green_channel)
            {
                LATAbits.LATA0 = 0;
                LATAbits.LATA1 = 0;
                LATAbits.LATA2 = 1;
                PWM1_Initialize(PWM_PERIOD_BLUE, PWM_PRESCALER);
                PWM1_SetDuty(512); // 0-127
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
        else
        {
            LATAbits.LATA3 = 0;
        }
    }

    return;
}
