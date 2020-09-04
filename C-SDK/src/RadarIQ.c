/**
 * @file
 * RadarIQ C module
 *
 */

//===============================================================================================//
// INCLUDES
//===============================================================================================//

#include "RadarIQ.h"

//===============================================================================================//
// DATA TYPES
//===============================================================================================//

typedef enum
{
	PARSING_STATE_WAITING_FOR_HEADER,
	PARSING_STATE_WAITING_FOR_FOOTER,
	PARSING_STATE_READING_PACKET,
} radariqParsingState_t;

//===============================================================================================//
// OBJECTS
//===============================================================================================//

struct radariq_t
{
	radariqCaptureMode_t captureMode;
	radariqResetCode_t resetCode;
	radariqPointDensity_t pointDensity;

	radariqData_t data;
	radariqStatistics_t stats;

	uint8_t rxBuffer[RADARIQ_RX_BUFFER_SIZE];
	radariqParsingState_t parsingState;

	void(*sendSerialDataCallback)(uint8_t * const, const uint16_t);
	uint8_t(*readSerialDataCallback)(void);
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

radariqHandle_t RadarIQ_init(void(*sendSerialDataCallback)(uint8_t * const, const uint16_t),
		uint8_t(*readSerialDataCallback)(void),
		void(*logCallback)(char * const))
{
	radariq_assert(NULL != sendSerialDataCallback);
	radariq_assert(NULL != readSerialDataCallback);
	radariq_assert(NULL != logCallback);

	radariqHandle_t handle = malloc(sizeof(radariq_t));
	radariq_assert(NULL != handle);

	handle->sendSerialDataCallback = sendSerialDataCallback;
	handle->readSerialDataCallback = readSerialDataCallback;
	handle->logCallback = logCallback;

	handle->captureMode = RADARIQ_MODE_POINT_CLOUD;
	handle->parsingState = PARSING_STATE_WAITING_FOR_HEADER;

	return handle;
}

bool RadarIQ_readSerial(const radariqHandle_t obj)
{
	radariq_assert(NULL != obj);

	bool isDataReady = false;

	uint8_t rxByte = obj->readSerialDataCallback();

	switch(obj->parsingState)
	{
	case PARSING_STATE_WAITING_FOR_HEADER:
	{
		if ()

		break;
	}
	case PARSING_STATE_WAITING_FOR_FOOTER:
	{
		break;
	}
	case PARSING_STATE_READING_PACKET:
	{
		break;
	}
	default:
	}


	return isDataReady;
}

 radariqReturnVal_t RadarIQ_getData(const radariqHandle_t obj, radariqData_t * const dest)
{
	radariq_assert(NULL != obj);
	radariq_assert(NULL != dest);

	*dest = obj->data;

	return RADARIQ_RETURN_VAL_OK;
}

uint32_t RadarIQ_getMemoryUsage()
{
	return sizeof(radariq_t);
}

//===============================================================================================//
// FILE-SCOPE FUNCTIONS
//===============================================================================================//


