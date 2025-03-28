/*
 * File:   main.c
 * Author: Dennis Cristian
 *
 * Created on February 3, 2025, 2:27 PM
 */


// PIC16F18855 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1
#pragma config FEXTOSC = HS     // External Oscillator mode selection bits (HS (crystal oscillator) above 4MHz; PFM set to high power)
#pragma config RSTOSC = HFINT32   // Power-up default value for COSC bits (HFINTOSC with OSCFRQ= 32 MHz and CDIV = 1:1)
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
#pragma config WDTE = OFF        // WDT operating mode (WDT enabled regardless of sleep; SWDTEN ignored)
#pragma config WDTCWS = WDTCWS_7// WDT Window Select bits (window always open (100%); software control; keyed access not required)
#pragma config WDTCCS = SC      // WDT input clock selector (Software Control)

// CONFIG4
#pragma config WRT = OFF        // UserNVM self-write protection bits (Write protection off)
#pragma config SCANE = available// Scanner Enable bit (Scanner module is available for use)
#pragma config LVP = ON         // Low Voltage Programming Enable bit (Low Voltage programming enabled. MCLR/Vpp pin function is MCLR.)

// CONFIG5
#pragma config CP = OFF         // UserNVM Program memory code protection bit (Program Memory code protection disabled)
#pragma config CPD = OFF        // DataNVM code protection bit (Data EEPROM code protection disabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdint.h>
#include "movement.h"

#define _XTAL_FREQ 32000000

#define TX_BUFFER_SIZE 64 // Transmission buffer size

uint8_t btnReleased = 1; // Check for if button was pressed and then released

enum Ore {
    SOLAR_FLARE = 1, // Red
    AURORIUM = 2, // Yellow
    COBALTITE = 3, // Blue
    SOLARIUM = 4, // Red and Yellow
    AUROTITE = 5 // Blue and Yellow
};

typedef struct {
    uint8_t isStartCommunication;
    uint8_t sync[2];
    uint16_t msgID;
    uint16_t payloadSize;
    uint8_t* payload;
    uint16_t dataReadCount;
} DataUART;

static const DataUART EMPTY_DATA_RESET; // Empty UART data packet

DataUART receivedData; // UART Receiver data

uint8_t txBuffer[TX_BUFFER_SIZE]; // Transmission buffer

uint8_t txHead = 0; // Index for adding data
uint8_t txTail = 0; // Index for transmitting data
uint8_t txCount = 0; // Number of bytes in the buffer

uint8_t solarArrayBlockEnabled = 0;

void __interrupt() interruptReceiver(void);

void initializeUARTBaud115200(void);

void setupUART(void);

void setupReceiverUART(void);

void setupTransmitterUART(void);

void sendUARTMessage(DataUART *msg);

void setupInterrupt(void);

////// UART Commands ///// 
DataUART getPCUInfo(); // 1.3.2. Message 0401h: Get PCU Info Command

DataUART getUserData(); // 1.3.4. Message 0501h: Get User Data Command

void setMotorSettings(uint8_t motorADirection, uint8_t motorAPWM, uint8_t motorBDirection, uint8_t motorBPWM); // 1.3.6. Message 0601h: Set Motor Settings Command

void setServoPulses(DataUART *msg); // 1.3.7. Message 0701h: Set Servo Pulses Command

void setLaserScope(DataUART *msg); // 1.3.8. Message 0801h: Set Laser Scope Command

void shootLaserDamage(DataUART *msg); // 1.3.9. Message 0901h: Shoot Laser (Damage) Command

void shootLaserTurretShieldCode(DataUART *msg); // 1.3.10. Message 0902h: Shoot Laser (Turret Shield Code) Command

void shootLaserRequestRepairCode(DataUART *msg); // 1.3.11. Message 0903h: Shoot Laser (Request Repair Code) Command

void shootLaserTransmitRepairCode(DataUART *msg); // 1.3.12. Message 0904h: Shoot Laser (Transmit Repair Code) Command

void processingPlantOreType(Ore oreValue); // 1.3.14. Message 0A03h: Processing Plant Ore Type Command
////// UART Commands /////

///// Solar Array /////
void setupSolarArrayBlock();

void toggleSolarArrayBlock();
///// Solar Array /////

void main(void) {
    initializeUARTBaud115200();
    
    setupUART();
    
    setupReceiverUART();
    
    setupTransmitterUART();
    
    setupInterrupt();
    
    // Switch S2 (RA5)
    TRISAbits.TRISA5 = 1;
    ANSELAbits.ANSA5 = 0;
    
    while(1)
    {
        if (!PORTAbits.RA5) {
            if (btnReleased) {
                DataUART x = getPCUInfo();
                DataUART y = x;
                
                btnReleased = 0;
            }
        }
        else {
            btnReleased = 1;
        }
    }
    
    return;
}

void __interrupt() mainISR(void) {    
    // Transmitter Interrupt Flag
    if (PIE3bits.TXIE && PIR3bits.TXIF) {
        if (txCount > 0) {
            TX1REG = txBuffer[txTail];
            txTail = (txTail + 1) % TX_BUFFER_SIZE;
            txCount--;
        } else {
            // Disable transmitter interrupt when buffer is empty
            PIE3bits.TXIE = 0;
        }
    }
    // Receiver Interrupt Flag
    if (PIE3bits.RCIE && PIR3bits.RCIF) {
        // Read RC1STA to check for errors
        if (RC1STAbits.OERR) {
            // Clear overrun error
            RC1STAbits.CREN = 0;
            
            // Re-enable receiver
            RC1STAbits.CREN = 1;
        }
        else {
            // Read received byte from RCREG
            uint8_t data = RCREG;

            // Handle synchronization
            if (!receivedData.isStartCommunication) {
                if (data == 0xFE) {
                    receivedData.sync[0] = data;
                    receivedData.sync[1] = 0; // Reset sync[1]
                }
                else if (data == 0x19 && receivedData.sync[0] == 0xFE) {
                    receivedData.sync[1] = data;
                    receivedData.isStartCommunication = 1; // Start communication
                    receivedData.dataReadCount = 0; // Reset data counter
                }
                else {
                    // Reset sync if not synchronized
                    receivedData.sync[0] = 0;
                    receivedData.sync[1] = 0;
                }
            }
            else {
                // Message reception logic
                switch (receivedData.dataReadCount) {
                    case 0: // LSB of message ID
                        receivedData.msgID = data;
                        break;
                    case 1: // MSB of message ID
                        receivedData.msgID |= ((uint16_t)data << 8);
                        break;
                    case 2: // LSB of payload size
                        receivedData.payloadSize = data;
                        break;
                    case 3: // MSB of payload size
                        receivedData.payloadSize |= ((uint16_t)data << 8);
                        if (receivedData.payloadSize > 0) {
                            receivedData.payload = (uint8_t*)malloc(receivedData.payloadSize);
                            if (!receivedData.payload) {
                                // Handle memory allocation failure
                                receivedData.isStartCommunication = 0; // Reset communication
                                return;
                            }
                        }
                        break;
                    default:
                        // Store payload data
                        if ((receivedData.dataReadCount - 4) < receivedData.payloadSize) {
                            receivedData.payload[receivedData.dataReadCount - 4] = data;
                        }
                        break;
                }
                
                receivedData.dataReadCount++;
                
                // Check if entire packet (header + payload) has been received
                if (receivedData.dataReadCount == (receivedData.payloadSize + 4)) {
                    PIE3bits.RCIE = 0; // Mark data as ready by disabling receiver interrupt
                    receivedData.isStartCommunication = 0; // End communication
                }
            }
        }
    }
}

// Enable Interrupts
void setupInterrupt(void) {
    // Global Interrupt Enable
    INTCONbits.GIE = 1;
    
    // Peripheral Interrupt Enable
    INTCONbits.PEIE = 1;
    
    // Receiver Interrupt Enable
    PIE3bits.RCIE = 1;
}

/////////////////////////////// UART ///////////////////////
// Initialize UART at 115200 baud
void initializeUARTBaud115200(void) {
    // Add BRG value 68
    SP1BRGHbits.SP1BRGH = (68 >> 8) & 0xFF;
    SP1BRGLbits.SP1BRGL = 68 & 0xFF;
    
    // Baud Rate formula Fosc/[4 (n+1)]]
    BAUD1CONbits.BRG16 = 1;
    TX1STAbits.BRGH = 1; 
}

// Setup Serial UART
void setupUART(void) {
    // Setup Serial port 
    TX1STAbits.SYNC = 0;
    RC1STAbits.SPEN = 1;
}

// Setup UART receiver
void setupReceiverUART(void) {
    // Set RX pin as Digital
    ANSELCbits.ANSC6 = 0;
    
    // Configure RX/DT I/O pin as input
    TRISCbits.TRISC6 = 1;
    
    // Configure to set receiver input to pin RC6
    RXPPS = 0x16;
    
    // Enable Receiver
    RC1STAbits.CREN = 1;
}

// Setup UART transmitter
void setupTransmitterUART(void) {
    // Set TX pin as Digital
    ANSELCbits.ANSC5 = 0;
    
    // Configure TX I/O pin as output
    TRISCbits.TRISC5 = 0;
    
    // Configure to set transmitter output to pin RC5
    RC5PPS = 0x10;
    
    // Enabling Transmitter
    TX1STAbits.TXEN = 1;
}

void sendUARTMessage(DataUART *msg) {
    // Add Sync Bytes to buffer
    for (uint8_t i = 0; i < 2; i++) {
        while (txCount >= TX_BUFFER_SIZE); // Wait if buffer is full
        txBuffer[txHead] = msg->sync[i];
        txHead = (txHead + 1) % TX_BUFFER_SIZE;
        txCount++;
    }

    // Add Message ID (LSB and MSB)
    while (txCount >= TX_BUFFER_SIZE);
    txBuffer[txHead] = (msg->msgID & 0xFF); // LSB
    txHead = (txHead + 1) % TX_BUFFER_SIZE;
    txCount++;

    while (txCount >= TX_BUFFER_SIZE);
    txBuffer[txHead] = (msg->msgID >> 8); // MSB
    txHead = (txHead + 1) % TX_BUFFER_SIZE;
    txCount++;

    // Add Payload Size (LSB and MSB)
    while (txCount >= TX_BUFFER_SIZE);
    txBuffer[txHead] = (msg->payloadSize & 0xFF); // LSB
    txHead = (txHead + 1) % TX_BUFFER_SIZE;
    txCount++;

    while (txCount >= TX_BUFFER_SIZE);
    txBuffer[txHead] = (msg->payloadSize >> 8); // MSB
    txHead = (txHead + 1) % TX_BUFFER_SIZE;
    txCount++;

    // Add Payload
    for (uint16_t i = 0; i < msg->payloadSize; i++) {
        while (txCount >= TX_BUFFER_SIZE);
        txBuffer[txHead] = msg->payload[i];
        txHead = (txHead + 1) % TX_BUFFER_SIZE;
        txCount++;
    }

    // Enable UART Transmitter Interrupt
    PIE3bits.TXIE = 1;
}

/////// Commands ////////////
// 1.3.2. Message 0401h: Get PCU Info Command
DataUART getPCUInfo(){
    receivedData = EMPTY_DATA_RESET;
    
    DataUART msg = {
                    .isStartCommunication = 0,
                    .sync = {0xFE, 0x19},
                    .msgID = 0x0401,
                    .payloadSize = 0x0000,
                    .payload = NULL,
                    .dataReadCount = 0
    };
    
    sendUARTMessage(&msg);
    
    // Wait until received data is ready
    while (PIE3bits.RCIE);
    
    // Re-enable receiver interrupt
    PIE3bits.RCIE = 1;
    
    return receivedData;
}

// 1.3.4. Message 0501h: Get User Data Command
DataUART getUserData(){
    receivedData = EMPTY_DATA_RESET;
    
    DataUART msg = {
                    .isStartCommunication = 0,
                    .sync = {0xFE, 0x19},
                    .msgID = 0x0501,
                    .payloadSize = 0x0000,
                    .payload = NULL,
                    .dataReadCount = 0
    };
    
    sendUARTMessage(&msg);
    
    // Wait until received data is ready
    while (PIE3bits.RCIE);
    
    // Re-enable receiver interrupt
    PIE3bits.RCIE = 1;
    
    return receivedData;
}

// 1.3.6. Message 0601h: Set Motor Settings Command
void setMotorSettings(uint8_t motorADirection, uint8_t motorAPWM, uint8_t motorBDirection, uint8_t motorBPWM) {
    UARTMessage* msg = (UARTMessage*) malloc(sizeof(UARTMessage));
    msg->sync[0] = 0xFE;       // Sync Byte 1
    msg->sync[1] = 0x19;       // Sync Byte 2
    msg->msgID[0] = 0x01;      // Msg ID LSB (01h)
    msg->msgID[1] = 0x06;      // Msg ID MSB (06h)
    msg->payloadSize[0] = 0x04; // Payload size LSB (04h)
    msg->payloadSize[1] = 0x00; // Payload size MSB (00h)
    
    // Allocate space for payload and fill it with motor settings
    msg->payload = (uint8_t*) malloc(4 * sizeof(uint8_t));
    msg->payload[0] = motorADirection;  // Motor A Direction 0 = brake, 1 = forward, 2 = reverse
    msg->payload[1] = motorAPWM;        // Motor A PWM (0-100)
    msg->payload[2] = motorBDirection;  // Motor B Direction 
    msg->payload[3] = motorBPWM;        // Motor B PWM (0-100)
    
    sendUARTMessage(msg);
    
    free(msg->payload);
    free(msg);
}

// 1.3.7. Message 0701h: Set Servo Pulses Command
void setServoPulses(DataUART *msg){
    
}

// 1.3.8. Message 0801h: Set Laser Scope Command
void setLaserScope(DataUART *msg){
    
}

// 1.3.9. Message 0901h: Shoot Laser (Damage) Command
void shootLaserDamage(DataUART *msg){
    
}

// 1.3.10. Message 0902h: Shoot Laser (Turret Shield Code) Command
void shootLaserTurretShieldCode(DataUART *msg){
    
}

// 1.3.11. Message 0903h: Shoot Laser (Request Repair Code) Command
void shootLaserRequestRepairCode(DataUART *msg){
    
}

// 1.3.12. Message 0904h: Shoot Laser (Transmit Repair Code) Command
void shootLaserTransmitRepairCode(DataUART *msg){
    
}

// 1.3.14. Message 0A03h: Processing Plant Ore Type Command
void processingPlantOreType(Ore oreValue){
    receivedData = EMPTY_DATA_RESET;
    
    DataUART msg = {
                    .isStartCommunication = 0,
                    .sync = {0xFE, 0x19},
                    .msgID = 0x0A03,
                    .payloadSize = 0x0001,
                    .payload = oreValue,
                    .dataReadCount = 0
    };
    
    sendUARTMessage(&msg);
}
/////// Commands ////////////
/////////////////////////////// UART ///////////////////////

/////////////////////////////// Solar Array ////////////////////
void setupSolarArrayBlock() {
    // Control Solar Array with pin RB0
    // Output
    TRISBbits.TRISB0 = 0;
    // Digital
    ANSELBbits.ANSB0 = 0;
    return;
}

void toggleSolarArrayBlock() {
    solarArrayBlockEnabled = !solarArrayBlockEnabled;
    PORTBbits.RB0 = solarArrayBlockEnabled;
    return;
}
/////////////////////////////// Solar Array ////////////////////
