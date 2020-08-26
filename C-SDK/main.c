/*
 * main.c
 *
 *  Created on: Aug 26, 2020
 *      Author: Nathan
 */


#include <stdio.h>
#include "RadarIQ.h"

int main()
{
	printf("RIQ-m C-SDK Test Program");

	radariqHandle_t myRadar = RadarIQ_init();

	return 0;
}
