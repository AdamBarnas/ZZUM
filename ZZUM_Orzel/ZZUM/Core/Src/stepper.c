#include "stepper.h"
#include <math.h>

static uint32_t calcARR(Stepper_t* m, float speed)
{
    if (speed < 1.0f) speed = 1.0f;
    uint32_t arr = (uint32_t)((SystemCoreClock / ((m->htim->Init.Prescaler + 1) * speed)) - 1);
    return arr;
}

void Stepper_Init(Stepper_t* m)
{
    m->currSpeed = 0;
    m->currPos = 0;
    m->targetPos = 0;
    m->moving = 0;
}

void Stepper_Enable(Stepper_t* m)
{
    HAL_GPIO_WritePin(m->EN_Port, m->EN_Pin, GPIO_PIN_RESET);
}

void Stepper_Disable(Stepper_t* m)
{
    HAL_GPIO_WritePin(m->EN_Port, m->EN_Pin, GPIO_PIN_SET);
}

void Stepper_SetMicrostep(Stepper_t* m, uint8_t microstep)
{
    uint8_t M0 = 0, M1 = 0, M2 = 0;
    switch (microstep) {
        case 1:  M0=0; M1=0; M2=0; break;
        case 2:  M0=1; M1=0; M2=0; break;
        case 4:  M0=0; M1=1; M2=0; break;
        case 8:  M0=1; M1=1; M2=0; break;
        case 16: M0=0; M1=0; M2=1; break;
        case 32: M0=1; M1=0; M2=1; break;
        default: return;
    }
    HAL_GPIO_WritePin(m->M0_Port, m->M0_Pin, M0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(m->M1_Port, m->M1_Pin, M1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(m->M2_Port, m->M2_Pin, M2 ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

// ustawienie aktualnej predkosci
void Stepper_SetSpeed(Stepper_t* m, float speed) { m->maxSpeed = speed; }

// ustawienie przyspieszenia
void Stepper_SetAcceleration(Stepper_t* m, float accel, float decel) {
    m->accel = accel;
    m->decel = decel;
}

// zadanie pozycji
void Stepper_MoveTo(Stepper_t* m, int32_t position)
{
    m->targetPos = position;
    m->moving = 1;
    m->dir = (position > m->currPos) ? 1 : -1;
    HAL_GPIO_WritePin(m->DIR_Port, m->DIR_Pin, (m->dir > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

// ruch o x krokow
void Stepper_Move(Stepper_t* m, int32_t steps)
{
    Stepper_MoveTo(m, m->currPos + steps);
}

// główna logika sterowania, wywoływana w przerwaniu timera
void Stepper_Tick(Stepper_t* m)
{
    if (!m->moving) return;

    // odległość do celu
    int32_t stepsRemaining = abs(m->targetPos - m->currPos);
    if (stepsRemaining == 0) {
        m->moving = 0;
        m->currSpeed = 0;
        return;
    }

    // dystans potrzebny do zatrzymania
    float Sbrake = (m->currSpeed * m->currSpeed) / (2.0f * m->decel);

    // przyspieszanie / hamowanie
    if (Sbrake >= stepsRemaining) {
        // hamuj
        m->currSpeed -= m->decel * 0.001f;  // dt ~1ms dla typowego timera
        if (m->currSpeed < 1.0f) m->currSpeed = 1.0f;
    } else if (m->currSpeed < m->maxSpeed) {
        // przyspieszaj
        m->currSpeed += m->accel * 0.001f;
        if (m->currSpeed > m->maxSpeed) m->currSpeed = m->maxSpeed;
    }

    // wykonaj krok
    HAL_GPIO_WritePin(m->STEP_Port, m->STEP_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(m->STEP_Port, m->STEP_Pin, GPIO_PIN_RESET);

    // aktualizuj pozycję
    m->currPos += m->dir;

    // zmień ARR (czyli częstotliwość)
    __HAL_TIM_SET_AUTORELOAD(m->htim, calcARR(m, m->currSpeed));
}
