/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "gcode_rx.h"
#include "usart.h"
#include "as5600_hal.h"
#include "stepper.h"
#include "tim.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern Stepper_t motor;

uint16_t raw;
extern volatile int16_t rotations;
volatile uint16_t hope;
float angle;

static float current_x = 0.0f;
static float feedrate0 = 1000.0f;
static float feedrate1 = 1000.0f;
static float acceleration = 1000.0f;      // mm/s^2 default
static bool absolute_mode = true;  // G90 = true, G91 = false
static float steps_per_mm = 6400.0/360.0f; // one mm <==> one deg
static float inactivity_time = 0.0f; // inactivity time in seconds
static uint32_t idle_time = 0;

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId Parser_taskHandle;
osThreadId Encoder_taskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void print_pos()
{
	hope = (int16_t)((float)motor.currPos)*ENCODER_RESOLUTION/FULL_REVOLUTION;
	hope = hope%ENCODER_RESOLUTION;
	int16_t curpos = motor.currPos;
	if(hope < 0)
	{
		hope += ENCODER_RESOLUTION;
	}
//	printf("rot %d  ", rotations);
//	printf("enc %u  ", raw);
//	printf("hope %u  ", hope);
//    printf("pos %d\n", curpos);
}
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void Start_Parser_task(void const * argument);
void Start_Encoder_task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityIdle, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of Parser_task */
  osThreadDef(Parser_task, Start_Parser_task, osPriorityNormal, 0, 128);
  Parser_taskHandle = osThreadCreate(osThread(Parser_task), NULL);

  /* definition and creation of Encoder_task */
  osThreadDef(Encoder_task, Start_Encoder_task, osPriorityBelowNormal, 0, 128);
  Encoder_taskHandle = osThreadCreate(osThread(Encoder_task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_Start_Parser_task */
/**
* @brief Function implementing the Parser_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Parser_task */
void Start_Parser_task(void const * argument)
{
  /* USER CODE BEGIN Start_Parser_task */

  uint8_t c;
  static char line[96];
  static uint8_t idx = 0;

	void wait_for_stop()
	{
		while(motor.moving){
		  osDelay(10);
		}
	}

	int32_t get_diff()
	{
		int32_t diff = (motor.targetPos - motor.currPos);
		if(diff < 0){diff = -diff;}
		return diff;
	}

	void goto_enc(int32_t target_pos)
	{
		motor.targetPos = target_pos;
		Stepper_get_enc_pos(&motor, &raw);
		int32_t diff = get_diff();
		while(diff > 2)
		{
			print_pos();
			Stepper_MoveTo(&motor, target_pos);
			wait_for_stop();
			print_pos();
			Stepper_get_enc_pos(&motor, &raw);
			print_pos();
			diff = get_diff();
		}
	}

	void process_gcode_line(char *line)
	{
	    char *p = line;

	    /* Strip comments */
	    for (char *c = line; *c; c++)
	    {
	        if (*c == ';' || *c == '(')
	        {
	            *c = '\0';
	            break;
	        }
	    }

	    while (isspace((unsigned char)*p)) p++;
	    if (*p == '\0')
	        return;

	    int gcode = -1;
	    int mcode = -1;
	    bool has_x = false;
	    bool has_f = false;
	    bool has_s = false;
	    float x_value = 0.0f;
	    float f_value = 0.0f;
	    float s_value = 0.0f;

	    /* Token parsing */
	    while (*p)
	    {
	        if (*p == 'G' || *p == 'g')
	        {
	            p++;
	            gcode = strtol(p, &p, 10);
	        }
	        if (*p == 'M' || *p == 'm')
			{
				p++;
				mcode = strtol(p, &p, 10);
			}
	        else if (*p == 'X' || *p == 'x')
	        {
	            p++;
	            x_value = strtof(p, &p);
	            has_x = true;
	        }
	        else if (*p == 'F' || *p == 'f')
	        {
	            p++;
	            f_value = strtof(p, &p);
	            has_f = true;
	        }
	        else if (*p == 'S' || *p == 's')
	        {
	            p++;
	            s_value = strtof(p, &p);
	            has_s = true;
	        }
	        else
	        {
	            p++;
	        }
	    }

	    switch (gcode)
	    {
	        case 0: /* G0 */
	    	    /* Feedrate is modal */
	    	    if (has_f && f_value > 0.0f)
	    	        feedrate0 = f_value;

	            if (has_x)
	            {
	                float target = absolute_mode ? x_value*steps_per_mm : (current_x + x_value*steps_per_mm);
//	                motion_move_x(target, feedrate);
	                Stepper_SetSpeed(&motor, feedrate0);
	                goto_enc((int32_t)target);
	                current_x = target;
	            }
	            break;

	        case 1: /* G1 */
	    	    /* Feedrate is modal */
	    	    if (has_f && f_value > 0.0f)
	    	        feedrate1 = f_value;

	            if (has_x)
	            {
	                float target = absolute_mode ? x_value*steps_per_mm : (current_x + x_value*steps_per_mm);
//	                motion_move_x(target, feedrate);
	                Stepper_SetSpeed(&motor, feedrate1);
	                goto_enc((int32_t)target);
	                current_x = target;
	            }
	            break;

	        case 28: /* G28 */
	            goto_enc(0);
	            current_x = 0.0f;
	            break;

	        case 90: /* G90 */
	            absolute_mode = true;
	            break;

	        case 91: /* G91 */
	            absolute_mode = false;
	            break;

	        default:
	            break;
	    }

	    switch (mcode)
	    {
			case 80: /* M80 power on*/
				Stepper_Enable(&motor);
				break;

			case 81: /* M81 power on*/
				Stepper_Disable(&motor);
				break;

			case 85: /* M85 inactivity shutdown period */
				if (has_s && s_value >= 0.0f)
				{
					inactivity_time = s_value;
				}
				break;

	    	case 92: /* M92 set steps per mm*/
				if (has_x)
				{
					if (x_value < 1)
						x_value = 1;
					steps_per_mm = x_value;
				}
				break;

	        case 204: /* M204 - Set acceleration */
	            if (has_s && s_value > 0.0f)
	            {
	                acceleration = s_value;
	                Stepper_SetAcceleration(&motor, acceleration, acceleration);
	            }
	            break;

	        default:
	        	break;
	    }
	}


	motor.DIR_Port  = GPIOB;
	motor.DIR_Pin   = GPIO_PIN_4;
	motor.EN_Port   = GPIOA;
	motor.EN_Pin    = GPIO_PIN_9;
	motor.htim      = &htim1;
	motor.tim_channel = TIM_CHANNEL_3;

	// INIT MOTOR
	Stepper_Init(&motor);
	// MOTOR TIMER
//	HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_3);
	Stepper_SetSpeed(&motor, 1000.0f);
	Stepper_SetAcceleration(&motor, 1000.0f, 1000.0f);
	osDelay(100);
	print_pos();

	goto_enc(0);
	rotations = 0;
//	Stepper_Disable(&motor);
  /* Infinite loop */
  for(;;)
  {
	  while (gcode_rx_get_char(&c))
	  {
	     if (c == '\r')
		 {
		    /* Ignore CR */
		    continue;
		 }
	     if (c == '\n')
	     {
	        line[idx] = 0;
//	        HAL_UART_Transmit(&huart6, (uint8_t*)line, idx, HAL_MAX_DELAY);
	        wait_for_stop();
	        process_gcode_line(line);
	        idx = 0;
	     }
	     else if (idx < sizeof(line) - 1)
	     {
	        line[idx++] = c;
	     }
	     idle_time = 0;
	  }

	  if (motor.moving == 0)
	  {
		  Stepper_get_enc_pos(&motor, &raw);
		  int32_t idle_diff = get_diff();
		  if (idle_diff < 0) idle_diff = -idle_diff;
		  if (idle_diff > 5) goto_enc(current_x);
	  }
	  idle_time += 10;
	  if (inactivity_time != 0 && (inactivity_time*1000) < idle_time) Stepper_Disable(&motor);
    osDelay(10);
  }
  /* USER CODE END Start_Parser_task */
}

/* USER CODE BEGIN Header_Start_Encoder_task */
/**
* @brief Function implementing the Encoder_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Encoder_task */
void Start_Encoder_task(void const * argument)
{
  /* USER CODE BEGIN Start_Encoder_task */
  /* Infinite loop */
	  for(;;)
	  {
	//	  AS5600_ReadRaw12(&raw);
		  uint16_t old_raw = raw;
		  if (AS5600_ReadRaw12(&raw))
		  {
//			  printf("%u\r\n", raw);  // wypisz samą wartość surową
	////		  printf("%f\r\n", raw2angle(raw));
			  if(raw < 1000 && old_raw > 3000)
			  {
				  rotations = rotations + 1;
			  }
			  if(old_raw < 1000 && raw > 3000)
			  {
				  rotations = rotations - 1;
			  }
		  }
		  else
		  {
			  printf("error\r\n");
		  }
		  osDelay(10);
	  }
  /* USER CODE END Start_Encoder_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
