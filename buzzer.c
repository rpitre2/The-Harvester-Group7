/*
 * File:   buzzer.c
 * Author: willm
 *
 * Created on February 11, 2025, 11:14 PM
 */

//// CONFIG1
//#pragma config FEXTOSC = OFF     // External Oscillator mode selection bits (EC above 8MHz; PFM set to high power)
//#pragma config RSTOSC = HFINT32  // Power-up default value for COSC bits (EXTOSC operating per FEXTOSC bits)
//#pragma config CLKOUTEN = OFF    // Clock Out Enable bit (CLKOUT function is disabled; i/o or oscillator function on OSC2)
//#pragma config CSWEN = ON        // Clock Switch Enable bit (Writing to NOSC and NDIV is allowed)
//#pragma config FCMEN = OFF       // Fail-Safe Clock Monitor Enable bit (FSCM timer enabled)
//
//// CONFIG2
//#pragma config MCLRE = ON       // Master Clear Enable bit (MCLR pin is Master Clear function)
//#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
//#pragma config LPBOREN = OFF    // Low-Power BOR enable bit (ULPBOR disabled)
//#pragma config BOREN = ON       // Brown-out reset enable bits (Brown-out Reset Enabled, SBOREN bit is ignored)
//#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (VBOR) set to 1.9V on LF, and 2.45V on F Devices)
//#pragma config ZCD = OFF        // Zero-cross detect disable (Zero-cross detect circuit is disabled at POR.)
//#pragma config PPS1WAY = ON     // Peripheral Pin Select one-way control (The PPSLOCK bit can be cleared and set only once in software)
//#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable bit (Stack Overflow or Underflow will cause a reset)
//
//// CONFIG3
//#pragma config WDTCPS = WDTCPS_31// WDT Period Select bits (Divider ratio 1:65536; software control of WDTPS)
//#pragma config WDTE = OFF        // WDT operating mode (WDT enabled regardless of sleep; SWDTEN ignored)
//#pragma config WDTCWS = WDTCWS_7// WDT Window Select bits (window always open (100%); software control; keyed access not required)
//#pragma config WDTCCS = SC      // WDT input clock selector (Software Control)
//
//// CONFIG4
//#pragma config WRT = OFF        // UserNVM self-write protection bits (Write protection off)
//#pragma config SCANE = available// Scanner Enable bit (Scanner module is available for use)
//#pragma config LVP = ON         // Low Voltage Programming Enable bit (Low Voltage programming enabled. MCLR/Vpp pin function is MCLR.)
//
//// CONFIG5
//#pragma config CP = OFF         // UserNVM Program memory code protection bit (Program Memory code protection disabled)
//#pragma config CPD = OFF        // DataNVM code protection bit (Data EEPROM code protection disabled)
//
#include <xc.h>
#include "buzzer.h"
//#include "mcc_generated_files/pwm/pwm6.h"





void PWM1_Initialize(uint16_t period, uint8_t prescaler) {
    // Step 1: Configure PWM output pin (Example: RC2 for CCP1)
    RC2PPS = 0x09;      // Assign CCP1 output to RC2 (check PPS table for your specific pin)
    CCP1PPS = 0x12;
    TRISCbits.TRISC2 = 1;         // Disable output driver temporarily (set as input)

    // Step 2: Set PWM period
    PR2 = period;       // Load period value into PR2

    // Step 3: Configure CCP1 for PWM mode
    CCP1CON = 0x80;     // CCP1FMT=0 (left-aligned), PWM mode (CCP1M3:CCP1M0 = 1100)
    CCP1CONbits.MODE = 0xF; // Explicitly set PWM mode

    // Step 4: Initialize duty cycle to 0
    CCPR1L = 0xFF;         // Clear lower 8 bits
    CCPR1H = 0xFF;         // Clear upper bits (alignment depends on CCP1FMT)

    // Step 5: Configure and start Timer2
    T2CLKCON = 0x01;    // Timer2 clock source = Fosc/4 (adjust as needed)
    T2CON = 0;          // Clear Timer2 settings
    T2CONbits.T2CKPS = prescaler;  // Set prescaler (0b00=1:1, 0b01=1:4, 0b10=1:16, 0b11=1:64)
    T2CONbits.OUTPS = 0x0;
    PIR4bits.TMR2IF = 0;         // Clear Timer2 interrupt flag
    T2CONbits.ON = 1;   // Enable Timer2

    // Step 6: Wait for first overflow and enable pin
    while (!PIR4bits.TMR2IF); // Wait until Timer2 overflows
    TRISCbits.TRISC2 = 0;         // Enable RC2 output driver
}

void PWM1_SetDuty(uint16_t duty) {
    duty &= 0x03FF; // Ensure duty is within 10-bit range

    if (!CCP1CONbits.CCP1FMT) { // Right-aligned mode
        CCPR1H = (duty >> 8) & 0x03; // Upper 2 bits
        CCPR1L = duty & 0xFF;        // Lower 8 bits
    } else { // Left-aligned mode
        uint16_t duty_shifted = duty << 6;
        CCPR1H = (duty_shifted >> 8) & 0xFF; // Upper 8 bits of shifted duty
        CCPR1L = duty_shifted & 0xFF;        // Lower 8 bits of shifted duty
    }
}


void notmain(void) 
{
    TRISCbits.TRISC2 = 0;
    ANSELCbits.ANSC2 = 0;
    
    PWM1_Initialize(PWM_PERIOD, 0b11);
    PWM1_SetDuty(1); // 0-127
    
    while(1){
        PWM1_Initialize(PWM_PERIOD, 0b11);
        PWM1_SetDuty(1); // 0-127
        __delay_ms(100);
        PWM1_Initialize(PWM_PERIOD*2, 0b11); 
        PWM1_SetDuty(1); // 0-127
        __delay_ms(100);
        PWM1_Initialize(PWM_PERIOD*4, 0b11);
        PWM1_SetDuty(1); // 0-127
        __delay_ms(100);
        PWM1_Initialize(PWM_PERIOD*8, 0b11);
        PWM1_SetDuty(1); // 0-127
        __delay_ms(100);
    }
    
    
    // debug LED
    TRISAbits.TRISA0 = 0;
    ANSELAbits.ANSA0 = 0;
    LATAbits.LATA0 = 0;
}