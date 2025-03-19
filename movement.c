#include "movement.h"

// External function from main.c
extern void setMotorSettings(uint8_t motorADirection, uint8_t motorAPWM,
                             uint8_t motorBDirection, uint8_t motorBPWM);
           
void movementControl(uint16_t y_stick, uint16_t x_stick){
    uint8_t motorADirection = 0;
    uint8_t motorBDirection = 0;
    uint8_t motorAPWM = 0;
    uint8_t motorBPWM = 0;
    
    // Handle Forward/Reverse Movement based on joystick input
    if (y_stick < 1480) {
        // Reverse (PWM increases as joystick value decreases)
        motorADirection = 2;   // Reverse
        motorBDirection = 2;   // Reverse
        motorAPWM = (uint8_t)(100 - (y_stick - 1000) / 4.8); // PWM increases as joystick value decreases
        motorBPWM = (uint8_t)(100 - (y_stick - 1000) / 4.8); // PWM increases as joystick value decreases
    } else if (y_stick > 1520) {
        // Forward (PWM increases as joystick value increases)
        motorADirection = 1;   // Forward
        motorBDirection = 1;   // Forward
        motorAPWM = (uint8_t)((y_stick - 1520) / 4.8); // PWM increases as joystick value increases
        motorBPWM = (uint8_t)((y_stick - 1520) / 4.8); // PWM increases as joystick value increases
    }
    
    uint8_t turnFactor = 0;
    if (x_stick < 1450) {
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
    
    if ((y_stick < 1480 || y_stick > 1520) && (x_stick < 1450 || x_stick > 1550)){
        motorADirection = 0; // Default to brake
        motorBDirection = 0; // Default to brake
        motorAPWM = 0;       // Default PWM
        motorBPWM = 0;       // Default PWM
    }


    if (motorAPWM < 30){
        motorAPWM = 30;
    } else if (motorAPWM > 100){
        motorAPWM = 100;
    }
    if (motorBPWM < 30){
        motorBPWM = 30;
    } else if (motorBPWM > 100){
        motorBPWM = 100;
    }

    // Send the motor settings
    setMotorSettings(motorADirection, motorAPWM, motorBDirection, motorBPWM);
}