/*
 * main.c
 *
 *  Created on: Aug 26, 2020
 *      Author: Nathan
 */


#include <stdio.h>
#include "RadarIQ.h"

static void callbackSendRadarData(uint8_t * const buffer, const uint16_t len);
static uint8_t callbackReadSerialData();
static void callbackRadarLog(char * const buffer);

int main()
{
	printf("\n*** RIQ-m C-SDK Test Program *** \n\n");

	radariqHandle_t myRadar = RadarIQ_init(callbackSendRadarData, callbackReadSerialData, callbackRadarLog);

	printf("* Created RadarIQ instance, using %u bytes of memory\n", RadarIQ_getMemoryUsage());

	for (uint32_t i = 0u; i < 5u; i++)
	{
		RadarIQ_readSerial(myRadar);
	}

	return 0;
}

static void callbackSendRadarData(uint8_t * const buffer, const uint16_t len)
{


}

static uint8_t callbackReadSerialData()
{
	static uint8_t i;
	i++;

	return i;
}

static void callbackRadarLog(char * const buffer)
{


}
