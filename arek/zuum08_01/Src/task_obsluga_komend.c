/*
 * task_obsluga_komend.c
 *
 *  Created on: Dec 7, 2025
 *      Author: arekc
 */

#include "main.h"
#include "cmsis_os2.h"
#include <string.h>
#include <stdio.h>

extern osMessageQueueId_t kolejka_handle;
osStatus_t status;
uint8_t komenda[ROZMIAR_TABLICY];


const uint8_t const_m114[] = "M114";

struktura_t dane = {
		.obecy_kod = niezdefiniowany,
};

void task_obsluga_komend(void *argument)
{



	for(;;)
	{
		status = osMessageQueueGet(kolejka_handle, komenda, NULL, 100);

		if(osOK == status)
		{
			while(dane.status_obecnego_zadania == w_tracie){
				osDelay(20);
			}

			if (komenda[0] == 'M' && komenda[1] == '1' )
			{
				// tu co pipisać do zmiennych
				dane.obecy_kod = M1;
				dane.kroki = 2;
			}
			if (komenda[0] == 'M' && komenda[1] == '2' )
			{
				int x = 1;
			}
			if (komenda[0] == 'G' && komenda[1] == '0' )
			{
				int tmp = 0; //tmp - tym czasowe
				if (sscanf(komenda, "X%d", &tmp)==1){
					dane.os_x = tmp;

				}
				if (sscanf(komenda, "F%d", &tmp)==1){


				}

			}
			if (komenda[0] == 'G' && komenda[1] == '1')
			{
				int x = 3;
			}
			if (komenda[0] == 'G' && komenda[1] == '9' && komenda[2] == '0' )
			{
				int x = 3;
			}
			if (komenda[0] == 'G' && komenda[1] == '9' && komenda[2] == '1' )
			{
				int x = 3;
			}
			if (komenda[0] == 'G' && komenda[1] == '2' && komenda[2] == '8' )
			{
				int x = 3;
			}

		}

	}

}

