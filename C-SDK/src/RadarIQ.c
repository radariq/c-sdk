/**
 * @file
 * RadarIQ C module
 *
 */

//===============================================================================================//
// INCLUDES
//===============================================================================================//

#include "RadarIQ.h"
#include <assert.h>

//===============================================================================================//
// DATA TYPES
//===============================================================================================//

struct radariq_t
{
	radariqCaptureMode_t captureMode;
	radariqResetCode_t resetCode;
	radariqPointDensity_t pointDensity;

	radariqData_t data;
	radariqStatistics_t stats;

	void(*sendSerialDataCallback)(uint8_t * const, const uint16_t);
	void(*logCallback)(char * const);
};

//===============================================================================================//
// CONSTANTS
//===============================================================================================//

//===============================================================================================//
// FILE-SCOPE VARIABLES
//===============================================================================================//

//===============================================================================================//
// FILE-SCOPE FUNCTION PROTOTYPES
//===============================================================================================//

//===============================================================================================//
// GLOBAL-SCOPE FUNCTIONS
//===============================================================================================//

radariqHandle_t RadarIQ_init(void(*sendSerialDataCallback)(uint8_t * const, const uint16_t), void(*logCallback)(char * const))
{
	assert(NULL != sendSerialDataCallback);
	assert(NULL != logCallback);

	radariqHandle_t handle = malloc(sizeof(radariq_t));
	assert(NULL != handle);

	handle->sendSerialDataCallback = sendSerialDataCallback;
	handle->logCallback = logCallback;

	return handle;
}

radariqData_t RadarIQ_getData(const radariqHandle_t obj)
{
	assert(NULL != obj);

	return obj->data;
}

uint32_t RadarIQ_getMemoryUsage(const radariqHandle_t obj)
{
	assert(NULL != obj);

	return sizeof(*obj);
}

//===============================================================================================//
// FILE-SCOPE FUNCTIONS
//===============================================================================================//


