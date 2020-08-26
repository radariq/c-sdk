/*
 * RadarIQ.c
 *
 *  Created on: Aug 26, 2020
 *      Author: Nathan
 */

#include "RadarIQ.h"
#include <assert.h>

struct radariq_t
{
	uint8_t * rxBuffer;
};

radariqHandle_t RadarIQ_init()
{
	radariqHandle_t handle = malloc(sizeof(radariq_t));
	assert(NULL != handle);

	handle->rxBuffer = malloc(RADARIQ_RX_BUFFER_SIZE);
	assert(NULL != handle->rxBuffer);

	return handle;
}
