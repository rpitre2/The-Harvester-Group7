#ifndef BUZZER_H

#define _XTAL_FREQ 32000000
#define PWM_PERIOD  0xFF  // (64MHz / 4) / (200 * 1) = 20kHz
#define PWM_PERIOD_RED 238
#define PWM_PERIOD_GREEN 178
#define PWM_PERIOD_BLUE 141
#define PWM_PRESCALER 0b111

void PWM1_Initialize(uint16_t period, uint8_t prescaler);
void PWM1_SetDuty(uint16_t duty);

#define BUZZER_H
#endif