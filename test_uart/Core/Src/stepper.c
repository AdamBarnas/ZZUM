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
    HAL_TIM_PWM_Stop_IT(m->htim, m->tim_channel);
}

void Stepper_Stop(Stepper_t* m)
{
	HAL_TIM_PWM_Stop_IT(m->htim, m->tim_channel);
}

void Stepper_Start(Stepper_t* m){
	HAL_TIM_Base_Start_IT(m->htim);
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
    if (position == m->currPos) return;

    m->targetPos = position;
    m->moving = 1;
    m->dir = (position > m->currPos) ? 1 : -1;

    HAL_GPIO_WritePin(
        m->DIR_Port,
        m->DIR_Pin,
        (m->dir > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET
    );

    uint32_t arr = calcARR(m, m->currSpeed);
    __HAL_TIM_SET_AUTORELOAD(m->htim, arr);
    __HAL_TIM_SET_COMPARE(m->htim, m->tim_channel, arr / 2);

    HAL_TIM_PWM_Start_IT(m->htim, m->tim_channel);
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

    m->currPos += m->dir;

    int32_t diff = m->targetPos - m->currPos;
    int32_t stepsRemaining = (diff >= 0) ? diff : -diff;

    if (stepsRemaining == 0) {
        m->moving = 0;
        HAL_TIM_PWM_Stop_IT(m->htim, m->tim_channel);
        return;
    }

    float Sbrake = (m->currSpeed * m->currSpeed) / (2.0f * m->decel);

    if (Sbrake >= stepsRemaining) {
        // decelerate
        m->currSpeed -= (m->decel / m->currSpeed);
        if (m->currSpeed < 1.0f)
            m->currSpeed = 1.0f;
    } else if (m->currSpeed < m->maxSpeed) {
        // accelerate
        m->currSpeed += (m->accel / m->currSpeed);
        if (m->currSpeed > m->maxSpeed)
            m->currSpeed = m->maxSpeed;
    }

    uint32_t arr = calcARR(m, m->currSpeed);
    __HAL_TIM_SET_AUTORELOAD(m->htim, arr);
    __HAL_TIM_SET_COMPARE(m->htim, m->tim_channel, arr / 2);
}

void Stepper_get_enc_pos(Stepper_t* m, uint16_t* raw)
{
	int32_t pos;
	pos  = (int32_t)(*raw * FULL_REVOLUTION / ENCODER_RESOLUTION);
	m->currPos = pos;
}
