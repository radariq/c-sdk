/*
 * main.c
 *
 *  Created on: Aug 26, 2020
 *      Author: Nathan
 */


#include <stdio.h>
#include "RadarIQ.h"

static void callbackSendRadarData(uint8_t * const buffer, const uint16_t len);
static void callbackRadarLog(char * const buffer);

int main()
{
	printf("\n*** RIQ-m C-SDK Test Program *** \n\n");

	radariqHandle_t myRadar = RadarIQ_init(callbackSendRadarData, callbackRadarLog);

	printf("* Created RadarIQ instance, using %u bytes of memory\n", RadarIQ_getMemoryUsage(myRadar));

	return 0;
}

static void callbackSendRadarData(uint8_t * const buffer, const uint16_t len)
{


}

static void callbackRadarLog(char * const buffer)
{


}
