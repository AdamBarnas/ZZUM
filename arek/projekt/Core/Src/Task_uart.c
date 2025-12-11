/*
 * Task_uart.c
 *
 *  Created on: Dec 7, 2025
 *      Author: arekc
 */
#include "main.h"
#include "cmsis_os2.h"
#include "usart.h"
#include <string.h>

extern osMessageQueueId_t kolejka_handle;

HAL_StatusTypeDef status_uart;
uint8_t data;
uint8_t odebrana_komenda[ROZMIAR_TABLICY];
uint8_t i=0;
void Task_uart(void *argument)
{
	HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

	for(;;)
	{
		status_uart = HAL_UART_Receive(&huart2, &data, sizeof(uint8_t), 10);
		if (status_uart == HAL_OK)
		{
			//zachowac to co odebralism
			odebrana_komenda[i]= data;
			i = i + 1;

			// jak to co odebralismy jest /n
			if(data== '\n')
			{
				//dekoduj komende i cos zrob
				HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
				i = 0;

				//wrzucanie na kolejkę,/ obsługa przed ewentualnym usunieciem
				osMessageQueuePut(kolejka_handle,odebrana_komenda,0,0);

				memset(odebrana_komenda, '\0', ROZMIAR_TABLICY);
			}

		}
		else
		{
			osDelay(1);
		}

//		osDelay(1);
	}

}

