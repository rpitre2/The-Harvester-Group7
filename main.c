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
#include "optical_signal.h"
//#include "movement.h"

#define _XTAL_FREQ 32000000
#define TX_BUFFER_SIZE 64 // Transmission buffer size
#define MAX_PAYLOAD_SIZE 50
#define VALUE_2000 0x7D0
#define VALUE_1500 0x5DC
#define VALUE_1000 0x3E8

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
    uint8_t payload[MAX_PAYLOAD_SIZE];
    uint16_t dataReadCount;
} DataUART;

typedef struct {
    uint8_t teamID;
    uint8_t playerID;
    uint16_t overallHealth;
    uint8_t shieldCodeFlag;
    uint8_t repairCodeFlag;
} PCUInfo;

typedef struct {
    uint16_t rightJoystickX;
    uint16_t rightJoystickY;
    uint16_t leftJoystickY;
    uint16_t leftJoystickX;
    uint16_t switchA;
    uint16_t switchB;
    uint16_t switchC;
    uint16_t switchD;
    uint16_t potentiometerVRA;
    uint16_t potentiometerVRB;
} UserDataResponse;

static const DataUART EMPTY_DATA_RESET; // Empty UART data packet

DataUART receivedData; // UART Receiver data

uint8_t txBuffer[TX_BUFFER_SIZE]; // Transmission buffer

uint8_t txHead = 0; // Index for adding data
uint8_t txTail = 0; // Index for transmitting data
uint8_t txCount = 0; // Number of bytes in the buffer

void movementControl(uint16_t y_stick, uint16_t x_stick);

void __interrupt() interruptReceiver(void);

void initializeUARTBaud115200(void);

void setupUART(void);

void setupReceiverUART(void);

void setupTransmitterUART(void);

void sendUARTMessage(DataUART *msg);

void setupInterrupt(void);

DataUART* createUARTMessage(uint8_t isStartCommunication, uint16_t msgId, uint16_t payloadSize, uint8_t payload[], uint8_t dataReadCount);

void setupOpticalSignalDecoding(void);

////// UART Commands ///// 
PCUInfo getPCUInfo(void); // 1.3.2. Message 0401h: Get PCU Info Command

UserDataResponse getUserData(void); // 1.3.4. Message 0501h: Get User Data Command

void setMotorSettings(uint8_t motorADirection, uint8_t motorAPWM, uint8_t motorBDirection, uint8_t motorBPWM);

void setServoPulses(DataUART *msg); // 1.3.7. Message 0701h: Set Servo Pulses Command

void setLaserScope(uint8_t enable); // 1.3.8. Message 0801h: Set Laser Scope Command

void shootLaserDamage(); // 1.3.9. Message 0901h: Shoot Laser (Damage) Command

void shootLaserTurretShieldCode(); // 1.3.10. Message 0902h: Shoot Laser (Turret Shield Code) Command

void shootLaserRequestRepairCode(); // 1.3.11. Message 0903h: Shoot Laser (Request Repair Code) Command

void shootLaserTransmitRepairCode(); // 1.3.12. Message 0904h: Shoot Laser (Transmit Repair Code) Command

void processingPlantOreType(enum Ore oreValue); // 1.3.14. Message 0A03h: Processing Plant Ore Type Command

////// UART Commands /////

/////////////////////////////// Solar Array ////////////////////
void setupSolarArrayBlock(void);

void enableSolarArrayBlock(void);

void disableSolarArrayBlock(void);
/////////////////////////////// Solar Array ////////////////////

void main(void) {
    initializeUARTBaud115200();
    
    setupUART();
    
    setupReceiverUART();
    
    setupTransmitterUART();
    
    setupInterrupt();
    
    setupSolarArrayBlock();
    setupOpticalSignalDecoding();
    
    // Switch S2 (RA5)
    TRISAbits.TRISA5 = 1;
    ANSELAbits.ANSA5 = 0;
    
    while(1)
    {
        if (!PORTAbits.RA5) {
            if (btnReleased) {
                
                
//                if(x.switchC == VALUE_2000) // Optical Signal Decoding (2000 = On)
//                {
//                    LATAbits.LATA0 = 1;
//                    OSD_Read_Colours();
//                }
//                else 
//                {
//                    LATAbits.LATA0 = 0;
//                }
                
                
                btnReleased = 0;
            }
        }
        else {
            btnReleased = 1;
        }
        
        
        PCUInfo x = getPCUInfo();
        PCUInfo y = x;
        
        UserDataResponse a = getUserData();
        UserDataResponse b = a;
        
        // Enable Solar Array Block
        if (a.switchB == 0x03E8) {
            LATAbits.LATA0 = 1;
            enableSolarArrayBlock();
        }
        else {
            LATAbits.LATA0 = 0;
            disableSolarArrayBlock();
        }
        
        movementControl(a.rightJoystickY, a.leftJoystickX);
        
        if (a.switchA == 2000) {
            setLaserScope(1);
        }

        
        if (a.switchD == 2000) {
            if (a.potentiometerVRB >= 1000 && a.potentiometerVRB <= 1199) {
                processingPlantOreType(SOLAR_FLARE);
            } else if (a.potentiometerVRB >= 1200 && a.potentiometerVRB <= 1399) {
                processingPlantOreType(AURORIUM);
            } else if (a.potentiometerVRB >= 1400 && a.potentiometerVRB <= 1599) {
                processingPlantOreType(COBALTITE);
            } else if (a.potentiometerVRB >= 1600 && a.potentiometerVRB <= 1799) {
                processingPlantOreType(SOLARIUM);
            } else if (a.potentiometerVRB >= 1800) {
                processingPlantOreType(AUROTITE);
            }
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
//                            receivedData.payload = (uint8_t*)malloc(receivedData.payloadSize);
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

void setupOpticalSignalDecoding(void)
{
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
    
    // Clear LED LATs
    LATAbits.LATA0 = 0;
    LATAbits.LATA1 = 0;
    LATAbits.LATA2 = 0;
    LATAbits.LATA3 = 0;
    
    // APDS-9960 Setup
//    I2C_Init();
//    ColourSensor_Enable();
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

DataUART* createUARTMessage(uint8_t isStartCommunication, uint16_t msgId, uint16_t payloadSize, uint8_t payload[], uint8_t dataReadCount) {
    DataUART* msg = (DataUART*) malloc(sizeof(DataUART));
    msg->isStartCommunication = isStartCommunication;
    msg->sync[0] = 0xFE;
    msg->sync[1] = 0x19;
    msg->msgID = msgId;
    msg->payloadSize = payloadSize;
    for (uint16_t i = 0; i < payloadSize; i++) {
        msg->payload[i] = payload[i];
    }
    msg->dataReadCount = dataReadCount;
    return msg;
}

/////// Commands ////////////
// 1.3.2. Message 0401h: Get PCU Info Command
PCUInfo getPCUInfo(){
    DataUART msgReceived;
    
    PCUInfo userResponseValue;
    
    receivedData = EMPTY_DATA_RESET;
    
    DataUART msg = {
                    .isStartCommunication = 0,
                    .sync = {0xFE, 0x19},
                    .msgID = 0x0401,
                    .payloadSize = 0x0000,
//                    .payload = NULL,
                    .dataReadCount = 0
    };
    
    sendUARTMessage(&msg);
    
    // Wait until received data is ready
    while (PIE3bits.RCIE);
//    __delay_ms(10);
    
    // Re-enable receiver interrupt
    PIE3bits.RCIE = 1;
    
    // Get received data
    msgReceived = receivedData;
    
    // Set user data response
    if (msgReceived.payloadSize == 20) { //msgReceived.payload != NULL && 
        userResponseValue.teamID = msgReceived.payload[0];

        userResponseValue.playerID = msgReceived.payload[1];

        userResponseValue.overallHealth = (uint16_t) ((msgReceived.payload[3] << 8) | msgReceived.payload[2]);

        userResponseValue.shieldCodeFlag = msgReceived.payload[4];

        userResponseValue.repairCodeFlag = msgReceived.payload[5];
    }
    
    return userResponseValue;
}

// 1.3.4. Message 0501h: Get User Data Command
UserDataResponse getUserData(){
    DataUART msgReceived;
    
    UserDataResponse userResponseValue;
    
    receivedData = EMPTY_DATA_RESET;
    
    DataUART msg = {
                    .isStartCommunication = 0,
                    .sync = {0xFE, 0x19},
                    .msgID = 0x0501,
                    .payloadSize = 0x0000,
//                    .payload = NULL,
                    .dataReadCount = 0
    };
    
    sendUARTMessage(&msg);
    
    // Wait until received data is ready
    while (PIE3bits.RCIE);
//    __delay_ms(10);
    
    // Re-enable receiver interrupt
    PIE3bits.RCIE = 1;
    
    // Get received data
    msgReceived = receivedData;
    
    // Set user data response
    if (msgReceived.payloadSize == 20) { //msgReceived.payload != NULL && 
        userResponseValue.rightJoystickX = (uint16_t) ((msgReceived.payload[1] << 8) | msgReceived.payload[0]);

        userResponseValue.rightJoystickY = (uint16_t) ((msgReceived.payload[3] << 8) | msgReceived.payload[2]);

        userResponseValue.leftJoystickY = (uint16_t) ((msgReceived.payload[5] << 8) | msgReceived.payload[4]);

        userResponseValue.leftJoystickX = (uint16_t) ((msgReceived.payload[7] << 8) | msgReceived.payload[6]);

        userResponseValue.switchA = (uint16_t) ((msgReceived.payload[9] << 8) | msgReceived.payload[8]);

        userResponseValue.switchB = (uint16_t) ((msgReceived.payload[11] << 8) | msgReceived.payload[10]);

        userResponseValue.switchC = (uint16_t) ((msgReceived.payload[13] << 8) | msgReceived.payload[12]);

        userResponseValue.switchD = (uint16_t) ((msgReceived.payload[15] << 8) | msgReceived.payload[14]);

        userResponseValue.potentiometerVRA = (uint16_t) ((msgReceived.payload[17] << 8) | msgReceived.payload[16]);

        userResponseValue.potentiometerVRB = (uint16_t) ((msgReceived.payload[19] << 8) | msgReceived.payload[18]);
    }
    
    return userResponseValue;
}

// 1.3.6. Message 0601h: Set Motor Settings Command

// I am going to fix how this is called after the demo
void setMotorSettings(uint8_t motorADirection, uint8_t motorAPWM, uint8_t motorBDirection, uint8_t motorBPWM) {
<<<<<<< Updated upstream
    DataUART* msg = (DataUART*) malloc(sizeof(DataUART));
    
    // Initialize the communication start flag
    msg->isStartCommunication = 0x01;   // Set to 1 to start communication
=======
    uint8_t motorSettings[] = {motorADirection, motorAPWM, motorBDirection, motorBPWM};
    DataUART* msg = createUARTMessage(0, 0x0601, 0x0004, motorSettings, 0);
>>>>>>> Stashed changes
    
    // Sync bytes (these are hardcoded, adjust if needed)
    msg->sync[0] = 0xFE;
    msg->sync[1] = 0x19;
    
    // Message ID (example: 0x01 0x06)
    msg->msgID = 0x0601;  // Combine the MSB and LSB for 16-bit msgID
    
    // Payload size (4 bytes for the motor settings)
    msg->payloadSize = 4; // Four bytes for motor directions and PWMs
    
    // Fill the payload with motor settings
    msg->payload[0] = motorADirection;  // Motor A Direction
    msg->payload[1] = motorAPWM;        // Motor A PWM
    msg->payload[2] = motorBDirection;  // Motor B Direction
    msg->payload[3] = motorBPWM;        // Motor B PWM
    
    // Set dataReadCount (usually indicates the amount of data read so far, set to 0 initially)
    msg->dataReadCount = 0;
    
    // Send the message via UART (assuming sendUARTMessage takes the DataUART structure)
    sendUARTMessage(msg);
<<<<<<< Updated upstream
    
    // Free allocated memory
=======
   
>>>>>>> Stashed changes
    free(msg);
}


// 1.3.7. Message 0701h: Set Servo Pulses Command
void setServoPulses(DataUART *msg){
    
}

// 1.3.8. Message 0801h: Set Laser Scope Command
void setLaserScope(uint8_t enable){
    DataUART* msg = createUARTMessage(0, 0x0801, 0x0001, &enable, 0);
    
    sendUARTMessage(msg);
    free(msg);
}

// 1.3.9. Message 0901h: Shoot Laser (Damage) Command
void shootLaserDamage(){
    uint8_t shotType = 0x2;
    DataUART* msg = createUARTMessage(0, 0x0901, 0x0001, &shotType, 0);
    
    sendUARTMessage(msg);
    free(msg);
}

// 1.3.10. Message 0902h: Shoot Laser (Turret Shield Code) Command
void shootLaserTurretShieldCode(){
    DataUART* msg = createUARTMessage(0, 0x0902, 0x0000, NULL, 0);
    
    sendUARTMessage(msg);
    free(msg);
}

// 1.3.11. Message 0903h: Shoot Laser (Request Repair Code) Command
void shootLaserRequestRepairCode(){
    DataUART* msg = createUARTMessage(0, 0x0903, 0x0000, NULL, 0);
    
    sendUARTMessage(msg);
    free(msg);
}

// 1.3.12. Message 0904h: Shoot Laser (Transmit Repair Code) Command
void shootLaserTransmitRepairCode(){
    DataUART* msg = createUARTMessage(0, 0x0904, 0x0000, NULL, 0);
    
    sendUARTMessage(msg);
    free(msg);
}

// 1.3.14. Message 0A03h: Processing Plant Ore Type Command
void processingPlantOreType(enum Ore oreValue){
    DataUART* msg = createUARTMessage(0, 0x0A03, 0x0001, &oreValue, 0);
    
    sendUARTMessage(msg);
    free(msg);
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

void enableSolarArrayBlock(void) {
    PORTBbits.RB0 = 1;
    return;
}

void disableSolarArrayBlock(void) {
    PORTBbits.RB0 = 0;
    return;
}
/////////////////////////////// Solar Array ////////////////////


void movementControl(uint16_t y_stick, uint16_t x_stick){
    uint8_t motorADirection = 0;
    uint8_t motorBDirection = 0;
    uint8_t motorAPWM = 0;
    uint8_t motorBPWM = 0;
    
    // Handle Forward/Reverse Movement based on joystick input
    if (y_stick < 1480) {
        // Reverse (PWM increases as joystick value decreases)
        motorADirection = 1;   // Reverse
        motorBDirection = 2;   // Reverse
        motorAPWM = (uint8_t)(100 - (y_stick - 1000) / 4.8); // PWM increases as joystick value decreases
        motorBPWM = (uint8_t)(100 - (y_stick - 1000) / 4.8); // PWM increases as joystick value decreases
    } else if (y_stick > 1520) {
        // Forward (PWM increases as joystick value increases)
        motorADirection = 2;   // Forward
        motorBDirection = 1;   // Forward
        motorAPWM = (uint8_t)((y_stick - 1520) / 4.8); // PWM increases as joystick value increases
        motorBPWM = (uint8_t)((y_stick - 1520) / 4.8); // PWM increases as joystick value increases
    }
    
    uint8_t turnFactor = 0;
    if ((y_stick > 1400 && y_stick < 1600) && (x_stick > 1400 && x_stick < 1600)){
        motorADirection = 0; // Default to brake
        motorBDirection = 0; // Default to brake
        motorAPWM = 0;       // Default PWM
        motorBPWM = 0;       // Default PWM
    }
    else if (x_stick < 1450) {
        // Left turn (PWM decreases based on proximity to 1000)
        turnFactor = (uint8_t)((1450 - x_stick) / 4.5);  // Calculate turn factor (closer to 1000 = stronger left turn)
         // Apply turning if joystick is in forward or reverse movement
        if (y_stick < 1480 || y_stick > 1520) {
            motorAPWM -= turnFactor/2;  // Slow down motor A (left side)
            motorBPWM += turnFactor/2;  // Speed up motor B (right side)
        } else {
            motorADirection = 2;  // Reverse
            motorBDirection = 1;  // Forward
            motorAPWM += turnFactor/2;  // Slow down motor A (left side)
            motorBPWM += turnFactor/2;  // Speed up motor B (right side)
        }
    } else if (x_stick > 1550) {
        // Right turn (PWM decreases based on proximity to 2000)
        turnFactor = (uint8_t)((x_stick - 1550) / 4.5);  // Calculate turn factor (closer to 2000 = stronger right turn)
        if (y_stick < 1480 || y_stick > 1520) {
            motorAPWM += turnFactor/2;  // Slow down motor A (left side)
            motorBPWM -= turnFactor/2;  // Speed up motor B (right side)
        } else {
            motorADirection = 1;  // Reverse
            motorBDirection = 2;  // Forward
            motorAPWM += turnFactor/2;  // Slow down motor A (left side)
            motorBPWM += turnFactor/2;  // Speed up motor B (right side)

        }
    }

    if (motorAPWM < 25){
        motorAPWM = 25;
    } else if (motorAPWM > 100){
        motorAPWM = 100;
    }
    if (motorBPWM < 25){
        motorBPWM = 25;
    } else if (motorBPWM > 100){
        motorBPWM = 100;
    }
    


    // Send the motor settings
    setMotorSettings(motorADirection, motorAPWM, motorBDirection, motorBPWM);
}