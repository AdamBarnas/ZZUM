/*
 * task_obsluga_komend.c
 *
 *  Created on: Dec 7, 2025
 *      Author: arekc
 */

#include "main.h"
#include "cmsis_os2.h"
#include <string.h>


extern osMessageQueueId_t kolejka_handle;
osStatus_t status;
uint8_t komenda[ROZMIAR_TABLICY];


const uint8_t const_m114[] = "M114";

void task_obsluga_komend(void *argument)
{

	for(;;)
	{
		status = osMessageQueueGet(kolejka_handle, komenda, NULL, 100);

		if(osOK == status)
		{
			// dekoodowanie komend np poprzez 1 litere

			if (komenda[0] == 'M' && komenda[1] == '1' &&
				komenda[2] == '1' && komenda[3] == '4' &&
				komenda[4] == '\n')
			{
				int x = 0;
			}


//			if (komenda to G0 XN YM)
//			{
//				wrzuc na kolejke komend do silnika komende zeby jechal na x=N i y=M
//			}
//			if komenda to G28
//			{
//				ustaw obecna pozycje jako home
//			}

		}

	}

}

