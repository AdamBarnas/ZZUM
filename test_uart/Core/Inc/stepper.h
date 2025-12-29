#ifndef STEPPER_H
#define STEPPER_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

typedef struct {
	// porty
    GPIO_TypeDef* DIR_Port;
    uint16_t DIR_Pin;
    GPIO_TypeDef* EN_Port;
    uint16_t EN_Pin;

    // Timer
    TIM_HandleTypeDef* htim;
    uint32_t tim_channel;

    // parametry ruchu
    float accel;         // steps/s^2
    float decel;         // steps/s^2
    float maxSpeed;      // steps/s
    float currSpeed;     // steps/s
    int32_t targetPos;   // cel (w krokach)
    int32_t currPos;     // aktualna pozycja (w krokach)
    uint8_t moving;      // flaga ruchu
    int8_t dir;          // +1 / -1
} Stepper_t;

void Stepper_Init(Stepper_t* m);
void Stepper_Enable(Stepper_t* m);
void Stepper_Disable(Stepper_t* m);
void Stepper_SetSpeed(Stepper_t* m, float speed);
void Stepper_SetAcceleration(Stepper_t* m, float accel, float decel);
void Stepper_MoveTo(Stepper_t* m, int32_t position);
void Stepper_Move(Stepper_t* m, int32_t steps);
void Stepper_Tick(Stepper_t* m);   // wywołuj w przerwaniu timera

#endif
