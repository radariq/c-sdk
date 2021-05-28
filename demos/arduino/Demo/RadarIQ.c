/**
 * @file
 * RadarIQ C module
 *
 */

//TODO remove unused byte helpers and rename the rest

//===============================================================================================//
// INCLUDES
//===============================================================================================//

#include "RadarIQ.h"

//===============================================================================================//
// DATA TYPES
//===============================================================================================//

typedef enum
{
    RADARIQ_CMD_VAR_REQUEST     = 0,
    RADARIQ_CMD_VAR_RESPONSE    = 1,
    RADARIQ_CMD_VAR_SET         = 2
} RadarIQCommandVariant_t;

typedef enum
{
	RX_STATE_WAITING_FOR_HEADER,
	RX_STATE_WAITING_FOR_FOOTER,
} RadarIQRxState_t;

typedef enum
{
	RADARIQ_SUBFRAME_START = 0,
	RADARIQ_SUBFRAME_MIDDLE = 1,
	RADARIQ_SUBFRAME_END = 2
} RadarIQSubframe_t;

typedef struct
{
	uint8_t data[RADARIQ_RX_BUFFER_SIZE];
	uint16_t len;
} RadarIQRxBuffer_t;

typedef struct
{
	uint8_t data[RADARIQ_TX_BUFFER_SIZE];
	uint16_t len;
} RadarIQTxBuffer_t;

typedef enum
{
	RADARIQ_MSG_TYPE_TEMPORARY 	= 0,
	RADARIQ_MSG_TYPE_DEBUG 		= 1,
	RADARIQ_MSG_TYPE_INFO 		= 2,
	RADARIQ_MSG_TYPE_WARNING 	= 3,
	RADARIQ_MSG_TYPE_ERROR 		= 4,
	RADARIQ_MSG_TYPE_SUCCESS 	= 5
} RadarIQMsgType_t;

//===============================================================================================//
// OBJECTS
//===============================================================================================//

struct RadarIQ_t
{
	RadarIQCaptureMode_t captureMode;
	RadarIQResetCode_t resetCode;
	RadarIQPointDensity_t pointDensity;

	RadarIQData_t data;
	uint16_t numDataPoints;
	RadarIQStatistics_t stats;
	bool isPowerGood;

	RadarIQRxBuffer_t rxBuffer;
	RadarIQRxBuffer_t rxPacket;
	RadarIQTxBuffer_t txBuffer;
	RadarIQTxBuffer_t txPacket;
	RadarIQRxState_t rxState;

	void(*sendSerialDataCallback)(uint8_t * const, const uint16_t);
	RadarIQUartData_t(*readSerialDataCallback)(void);
	void(*logCallback)(char * const);
};

//===============================================================================================//
// CONSTANTS
//===============================================================================================//

#define RADARIQ_PACKET_ESC			(uint8_t)0xB2
#define RADARIQ_PACKET_XOR			(uint8_t)0x04
#define RADARIQ_PACKET_HEAD			(uint8_t)0xB0
#define RADARIQ_PACKET_FOOT			(uint8_t)0xB1

//===============================================================================================//
// FILE-SCOPE VARIABLES
//===============================================================================================//

//===============================================================================================//
// FILE-SCOPE FUNCTION PROTOTYPES
//===============================================================================================//

static void RadarIQ_sendPacket(const RadarIQHandle_t obj);
static RadarIQCommand_t RadarIQ_pollResponse(const RadarIQHandle_t obj);
static bool RadarIQ_decodePacket(const RadarIQHandle_t obj);
static RadarIQCommand_t RadarIQ_parsePacket(const RadarIQHandle_t obj);
static uint16_t getCrc16Ccitt(uint8_t const * array, uint8_t len);
static void encodeHelper(const RadarIQHandle_t obj, uint8_t const databyte);

// Packet parsing
static void RadarIQ_parsePointCloud(const RadarIQHandle_t obj);
static void RadarIQ_parseObjectTracking(const RadarIQHandle_t obj);
static void RadarIQ_parseMessage(const RadarIQHandle_t obj);
static void RadarIQ_parseProcessingStats(const RadarIQHandle_t obj);
static void RadarIQ_parsePointCloudStats(const RadarIQHandle_t obj);
static void RadarIQ_parsePowerStatus(const RadarIQHandle_t obj);

// Byte helpers
static uint32_t bytePack32(const uint8_t * const data);
static int32_t bytePack32Signed(const uint8_t * const data);
static uint16_t bytePack16(const uint8_t * const data);
static int16_t bytePack16Signed(const uint8_t * const data);
static void byteUnpack32(const uint32_t data, uint8_t * const dest);
static void byteUnpack32Signed(const int32_t data, uint8_t * const dest);
static void byteUnpack16(uint16_t const data, uint8_t * const dest);
static void byteUnpack16Signed(int16_t const data, uint8_t * const dest);

//===============================================================================================//
// GLOBAL-SCOPE FUNCTIONS
//===============================================================================================//

RadarIQHandle_t RadarIQ_init(void(*sendSerialDataCallback)(uint8_t * const, const uint16_t),
		RadarIQUartData_t(*readSerialDataCallback)(void),
		void(*logCallback)(char * const))
{
	radariq_assert(NULL != sendSerialDataCallback);
	radariq_assert(NULL != readSerialDataCallback);
	radariq_assert(NULL != logCallback);

	RadarIQHandle_t handle = malloc(sizeof(RadarIQ_t));
	radariq_assert(NULL != handle);
	memset((void*)handle, 0, sizeof(RadarIQ_t));

	handle->sendSerialDataCallback = sendSerialDataCallback;
	handle->readSerialDataCallback = readSerialDataCallback;
	handle->logCallback = logCallback;

	handle->captureMode = RADARIQ_MODE_POINT_CLOUD;
	handle->rxState = RX_STATE_WAITING_FOR_HEADER;

	return handle;
}

RadarIQCommand_t RadarIQ_readSerial(const RadarIQHandle_t obj)
{
	radariq_assert(NULL != obj);

	RadarIQCommand_t packet = RADARIQ_CMD_NONE;
	RadarIQUartData_t rxData;
	rxData = obj->readSerialDataCallback();
	
	if (!rxData.isReadable)
	{
		return packet;	
	}

	switch(obj->rxState)
	{
		case RX_STATE_WAITING_FOR_HEADER:
		{
			if (RADARIQ_PACKET_HEAD == rxData.data)
			{
				obj->rxBuffer.data[0] = rxData.data;
				obj->rxBuffer.len = 1u;

				obj->rxState = RX_STATE_WAITING_FOR_FOOTER;
			}

			break;
		}
		case RX_STATE_WAITING_FOR_FOOTER:
		{
			obj->rxBuffer.data[obj->rxBuffer.len] = rxData.data;
			obj->rxBuffer.len = (obj->rxBuffer.len + 1) % RADARIQ_RX_BUFFER_SIZE;

			if (RADARIQ_PACKET_FOOT == rxData.data)
			{
				bool success = RadarIQ_decodePacket(obj);

				if (success)
				{
					packet = RadarIQ_parsePacket(obj);
				}
				else
				{
					packet = RADARIQ_CMD_ERROR;
				}

				obj->rxState = RX_STATE_WAITING_FOR_HEADER;
			}

			break;
		}
		default:
		{
			break;
		}
	}

	return packet;
}

 void RadarIQ_getData(const RadarIQHandle_t obj, RadarIQData_t * const dest)
{
	radariq_assert(NULL != obj);
	radariq_assert(NULL != dest);

	*dest = obj->data;
}

void RadarIQ_getStatistics(const RadarIQHandle_t obj, RadarIQProcessingStats_t * const processing, 
	RadarIQPointcloudStats_t * const pointcloud, RadarIQChipTemperatures_t * const temperatures)
{
	radariq_assert(NULL != obj);	
	radariq_assert(NULL != processing);
	radariq_assert(NULL != pointcloud);
	radariq_assert(NULL != temperatures);
	
	*processing = obj->stats.processing;
	*pointcloud = obj->stats.pointcloud;
	*temperatures = obj->stats.temperature;	
}

bool RadarIQ_isPowerGood(const RadarIQHandle_t obj)
{
	radariq_assert(NULL != obj);
	
	return obj->isPowerGood;	
}

void RadarIQ_getProcessingStats(const RadarIQHandle_t obj, RadarIQProcessingStats_t * const dest)
{
	radariq_assert(NULL != obj);	
	radariq_assert(NULL != dest);
	
	*dest = obj->stats.processing;
}

void RadarIQ_getPointCloudStats(const RadarIQHandle_t obj, RadarIQPointcloudStats_t * const dest)
{
	radariq_assert(NULL != obj);	
	radariq_assert(NULL != dest);
	
	*dest = obj->stats.pointcloud;
}

void RadarIQ_getChipTemperatures(const RadarIQHandle_t obj, RadarIQChipTemperatures_t * const dest)
{
	radariq_assert(NULL != obj);	
	radariq_assert(NULL != dest);
	
	*dest = obj->stats.temperature;
}

//===============================================================================================//
// GLOBAL-SCOPE FUNCTIONS - Debug / Info
//===============================================================================================//

uint32_t RadarIQ_getMemoryUsage()
{
	return sizeof(RadarIQ_t);
}

uint8_t RadarIQ_getDataBuffer(const RadarIQHandle_t obj, uint8_t* dest)
{
	radariq_assert(NULL != obj);

	memcpy(dest, obj->rxPacket.data, obj->rxPacket.len);

	return obj->rxPacket.len;
}


//===============================================================================================//
// GLOBAL-SCOPE FUNCTIONS - UART Commands
//===============================================================================================//

void RadarIQ_start(const RadarIQHandle_t obj, const uint8_t numFrames)
{
	radariq_assert(NULL != obj);

	obj->txPacket.data[0] = RADARIQ_CMD_CAPTURE_START;
	obj->txPacket.data[1] = RADARIQ_CMD_VAR_REQUEST;
	obj->txPacket.data[2] = numFrames;
	obj->txPacket.len = 3u;

	RadarIQ_sendPacket(obj);
}

RadarIQReturnVal_t RadarIQ_reset(const RadarIQHandle_t obj, const RadarIQResetCode_t code)
{
	radariq_assert(NULL != obj);

	RadarIQReturnVal_t ret = RADARIQ_RETURN_VAL_OK;

	if ((code != RADARIQ_RESET_FACTORY_SETTINGS) && (code != RADARIQ_RESET_REBOOT))
	{
		ret = RADARIQ_RETURN_VAL_ERR;
	}

	else
	{
		obj->txPacket.data[0] = RADARIQ_CMD_RESET;
		obj->txPacket.data[1] = RADARIQ_CMD_VAR_SET;
		obj->txPacket.data[2] = (uint8_t)code;
		obj->txPacket.len = 3u;

		RadarIQ_sendPacket(obj);
		
		if (RADARIQ_CMD_RESET != RadarIQ_pollResponse(obj))
		{
			ret = RADARIQ_RETURN_VAL_ERR;	
		}
	}

	return ret;
}

RadarIQReturnVal_t RadarIQ_save(const RadarIQHandle_t obj)
{
	radariq_assert(NULL != obj);

	RadarIQReturnVal_t ret = RADARIQ_RETURN_VAL_OK;

	obj->txPacket.data[0] = RADARIQ_CMD_SAVE;
	obj->txPacket.data[1] = RADARIQ_CMD_VAR_REQUEST;
	obj->txPacket.len = 2u;

	RadarIQ_sendPacket(obj);

	if (RADARIQ_CMD_SAVE != RadarIQ_pollResponse(obj))
	{
		ret = RADARIQ_RETURN_VAL_ERR;	
	}

	return ret;
}

RadarIQReturnVal_t RadarIQ_getVersion(const RadarIQHandle_t obj, RadarIQVersion_t * const firmware, RadarIQVersion_t * const hardware)
{
	radariq_assert(NULL != obj);
	radariq_assert(NULL != firmware);
	radariq_assert(NULL != hardware);

	RadarIQReturnVal_t ret = RADARIQ_RETURN_VAL_OK;

	obj->txPacket.data[0] = RADARIQ_CMD_VERSION;
	obj->txPacket.data[1] = RADARIQ_CMD_VAR_REQUEST;
	obj->txPacket.len = 2u;

	RadarIQ_sendPacket(obj);

	if (RADARIQ_CMD_VERSION == RadarIQ_pollResponse(obj))
	{	
		memcpy((void*)firmware, (void*)&obj->rxPacket.data[2], 4);	
		memcpy((void*)hardware, (void*)&obj->rxPacket.data[6], 4);	
	}
	else
	{
		ret = RADARIQ_RETURN_VAL_ERR;
	}

	return ret;
}

RadarIQReturnVal_t RadarIQ_getRadarVersions(const RadarIQHandle_t obj, RadarIQVersionIWR_t * const sbl,
		RadarIQVersionIWR_t * const app1, RadarIQVersionIWR_t * const app2)
{
	radariq_assert(NULL != obj);

	RadarIQReturnVal_t ret = RADARIQ_RETURN_VAL_OK;


	return ret;
}

RadarIQReturnVal_t RadarIQ_getSerialNumber(const RadarIQHandle_t obj, RadarIQSerialNo_t * const serial)
{
	radariq_assert(NULL != obj);

	RadarIQReturnVal_t ret = RADARIQ_RETURN_VAL_OK;

	obj->txPacket.data[0] = RADARIQ_CMD_SERIAL;
	obj->txPacket.data[1] = RADARIQ_CMD_VAR_REQUEST;
	obj->txPacket.len = 2u;

	RadarIQ_sendPacket(obj);

	if (RADARIQ_CMD_SERIAL == RadarIQ_pollResponse(obj))
	{
		memcpy((void*)serial, (void*)&obj->rxPacket.data[2], 4);
	}
	else
	{
		ret = RADARIQ_RETURN_VAL_ERR;	
	}

	return ret;
}

RadarIQReturnVal_t RadarIQ_getFrameRate(const RadarIQHandle_t obj, uint8_t * const rate)
{
	radariq_assert(NULL != obj);

	RadarIQReturnVal_t ret = RADARIQ_RETURN_VAL_OK;

	obj->txPacket.data[0] = RADARIQ_CMD_FRAME_RATE;
	obj->txPacket.data[1] = RADARIQ_CMD_VAR_REQUEST;
	obj->txPacket.len = 2u;

	RadarIQ_sendPacket(obj);

	if (RADARIQ_CMD_FRAME_RATE == RadarIQ_pollResponse(obj))
	{
		*rate = obj->rxPacket.data[2];		
	}
	else
	{
		ret = RADARIQ_RETURN_VAL_ERR;	
	}

	return ret;
}

RadarIQReturnVal_t RadarIQ_setFrameRate(const RadarIQHandle_t obj, uint8_t rate)
{
	radariq_assert(NULL != obj);

	RadarIQReturnVal_t ret = RADARIQ_RETURN_VAL_OK;

	if (RADARIQ_MIN_FRAME_RATE > rate)
	{
		rate = RADARIQ_MIN_FRAME_RATE;
		ret = RADARIQ_RETURN_VAL_WARNING;	
	}
	if (RADARIQ_MAX_FRAME_RATE < rate)
	{
		rate = RADARIQ_MAX_FRAME_RATE;
		ret = RADARIQ_RETURN_VAL_WARNING;	
	}

	obj->txPacket.data[0] = RADARIQ_CMD_FRAME_RATE;
	obj->txPacket.data[1] = RADARIQ_CMD_VAR_SET;
	obj->txPacket.data[2] = rate;
	obj->txPacket.len = 3u;

	RadarIQ_sendPacket(obj);

	if (RADARIQ_CMD_FRAME_RATE != RadarIQ_pollResponse(obj))
	{
		ret = RADARIQ_RETURN_VAL_ERR;	
	}
	
	return ret;
}

RadarIQReturnVal_t RadarIQ_getMode(const RadarIQHandle_t obj, RadarIQCaptureMode_t * const mode)
{
	radariq_assert(NULL != obj);
	radariq_assert(NULL != mode);

	RadarIQReturnVal_t ret = RADARIQ_RETURN_VAL_OK;

	obj->txPacket.data[0] = RADARIQ_CMD_MODE;
	obj->txPacket.data[1] = RADARIQ_CMD_VAR_REQUEST;
	obj->txPacket.len = 2u;

	RadarIQ_sendPacket(obj);

	if (RADARIQ_CMD_MODE == RadarIQ_pollResponse(obj))
	{
		*mode = (RadarIQCaptureMode_t)obj->rxPacket.data[2];
	}
	else
	{
		ret = RADARIQ_RETURN_VAL_ERR;	
	}

	return ret;
}

RadarIQReturnVal_t RadarIQ_setMode(const RadarIQHandle_t obj, RadarIQCaptureMode_t mode)
{
	radariq_assert(NULL != obj);

	RadarIQReturnVal_t ret = RADARIQ_RETURN_VAL_OK;

	if ((RADARIQ_MODE_POINT_CLOUD > mode) || (RADARIQ_MODE_RAW_DATA < mode))
	{
		ret = RADARIQ_RETURN_VAL_ERR;
		return ret;	
	}

	obj->txPacket.data[0] = RADARIQ_CMD_MODE;
	obj->txPacket.data[1] = RADARIQ_CMD_VAR_SET;
	obj->txPacket.data[2] = (uint8_t)mode;
	obj->txPacket.len = 3u;

	RadarIQ_sendPacket(obj);

	if (RADARIQ_CMD_MODE != RadarIQ_pollResponse(obj))
	{
		ret = RADARIQ_RETURN_VAL_ERR;	
	}

	return ret;
}

RadarIQReturnVal_t RadarIQ_getDistanceFilter(const RadarIQHandle_t obj, uint16_t * const min, uint16_t * const max)
{
	radariq_assert(NULL != obj);
	radariq_assert(NULL != min);
	radariq_assert(NULL != max);	

	RadarIQReturnVal_t ret = RADARIQ_RETURN_VAL_OK;

	obj->txPacket.data[0] = RADARIQ_CMD_DIST_FILT;
	obj->txPacket.data[1] = RADARIQ_CMD_VAR_REQUEST;
	obj->txPacket.len = 2u;

	RadarIQ_sendPacket(obj);

	if (RADARIQ_CMD_DIST_FILT == RadarIQ_pollResponse(obj))
	{	
		*min = bytePack16(&obj->rxPacket.data[2]);
		*max = bytePack16(&obj->rxPacket.data[4]);
	}
	else
	{
		ret = RADARIQ_RETURN_VAL_ERR;
	}

	return ret;
}

RadarIQReturnVal_t RadarIQ_setDistanceFilter(const RadarIQHandle_t obj, uint16_t min, uint16_t max)
{
	radariq_assert(NULL != obj);

	RadarIQReturnVal_t ret = RADARIQ_RETURN_VAL_OK;
	
	if (min > max)
	{
		uint16_t temp = min;
		min = max;
		max = temp;	
	}
	
	if (RADARIQ_MAX_DIST_FILT < min)
	{
		min = RADARIQ_MAX_DIST_FILT;
		ret = RADARIQ_RETURN_VAL_WARNING;
	}
	
	if (RADARIQ_MAX_DIST_FILT < max)
	{
		max = RADARIQ_MAX_DIST_FILT;
		ret = RADARIQ_RETURN_VAL_WARNING;
	}
	
	obj->txPacket.data[0] = RADARIQ_CMD_DIST_FILT;
	obj->txPacket.data[1] = RADARIQ_CMD_VAR_SET;
	byteUnpack16(min, &obj->txPacket.data[2]);
	byteUnpack16(max, &obj->txPacket.data[4]);	
	obj->txPacket.len = 6u;
	
	RadarIQ_sendPacket(obj);

	if (RADARIQ_CMD_DIST_FILT != RadarIQ_pollResponse(obj))
	{
		ret = RADARIQ_RETURN_VAL_ERR;	
	}

	return ret;
}

RadarIQReturnVal_t RadarIQ_getAngleFilter(const RadarIQHandle_t obj, int8_t * const min, int8_t * const max)
{
	radariq_assert(NULL != obj);

	RadarIQReturnVal_t ret = RADARIQ_RETURN_VAL_OK;

	obj->txPacket.data[0] = RADARIQ_CMD_ANGLE_FILT;
	obj->txPacket.data[1] = RADARIQ_CMD_VAR_REQUEST;
	obj->txPacket.len = 2u;

	RadarIQ_sendPacket(obj);

	if (RADARIQ_CMD_ANGLE_FILT == RadarIQ_pollResponse(obj))
	{
		*min = (int8_t)(0u | obj->rxPacket.data[2]);
		*max = (int8_t)(0u | obj->rxPacket.data[3]);	
	}
	else
	{	
		ret = RADARIQ_RETURN_VAL_ERR;
	}

	return ret;
}

RadarIQReturnVal_t RadarIQ_setAngleFilter(const RadarIQHandle_t obj, int8_t min, int8_t max)
{
	radariq_assert(NULL != obj);

	RadarIQReturnVal_t ret = RADARIQ_RETURN_VAL_OK;

	if (min > max)
	{
		int8_t temp = min;
		min = max;
		max = temp;	
	}
	
	if (RADARIQ_MIN_ANGLE_FILT > min)
	{
		min = RADARIQ_MIN_ANGLE_FILT;
		ret = RADARIQ_RETURN_VAL_WARNING;
	}
	if (RADARIQ_MAX_ANGLE_FILT < min)
	{
		min = RADARIQ_MAX_ANGLE_FILT;
		ret = RADARIQ_RETURN_VAL_WARNING;
	}
	if (RADARIQ_MIN_ANGLE_FILT > max)
	{
		max = RADARIQ_MIN_ANGLE_FILT;
		ret = RADARIQ_RETURN_VAL_WARNING;
	}
	if (RADARIQ_MAX_ANGLE_FILT < max)
	{
		max = RADARIQ_MAX_ANGLE_FILT;
		ret = RADARIQ_RETURN_VAL_WARNING;
	}
	
	obj->txPacket.data[0] = RADARIQ_CMD_ANGLE_FILT;
	obj->txPacket.data[1] = RADARIQ_CMD_VAR_SET;
	obj->txPacket.data[2] = (uint8_t)(0 | min);
	obj->txPacket.data[3] = (uint8_t)(0 | max);	
	obj->txPacket.len = 4u;
	
	RadarIQ_sendPacket(obj);

	if (RADARIQ_CMD_ANGLE_FILT != RadarIQ_pollResponse(obj))
	{
		ret = RADARIQ_RETURN_VAL_ERR;	
	}
	
	return ret;
}

RadarIQReturnVal_t RadarIQ_getMovingFilter(const RadarIQHandle_t obj, RadarIQMovingFilterMode_t * const filter)
{
	radariq_assert(NULL != obj);

	RadarIQReturnVal_t ret = RADARIQ_RETURN_VAL_OK;

	obj->txPacket.data[0] = RADARIQ_CMD_MOVING_FILT;
	obj->txPacket.data[1] = RADARIQ_CMD_VAR_REQUEST;
	obj->txPacket.len = 2u;

	RadarIQ_sendPacket(obj);

	if (RADARIQ_CMD_MOVING_FILT == RadarIQ_pollResponse(obj))
	{
		*filter = (RadarIQMovingFilterMode_t)obj->rxPacket.data[2];	
	}
	else
	{
		ret = RADARIQ_RETURN_VAL_ERR;	
	}
	
	return ret;
}

RadarIQReturnVal_t RadarIQ_setMovingFilter(const RadarIQHandle_t obj, RadarIQMovingFilterMode_t filter)
{
	radariq_assert(NULL != obj);

	RadarIQReturnVal_t ret = RADARIQ_RETURN_VAL_OK;

	if ((RADARIQ_MOVING_BOTH != filter) && (RADARIQ_MOVING_OBJECTS_ONLY != filter))
	{
		ret = RADARIQ_RETURN_VAL_ERR;
		return ret;	
	}

	obj->txPacket.data[0] = RADARIQ_CMD_MOVING_FILT;
	obj->txPacket.data[1] = RADARIQ_CMD_VAR_SET;
	obj->txPacket.data[2] = (uint8_t)filter;
	obj->txPacket.len = 3u;

	RadarIQ_sendPacket(obj);

	if (RADARIQ_CMD_MOVING_FILT != RadarIQ_pollResponse(obj))
	{
		ret = RADARIQ_RETURN_VAL_ERR;	
	}

	return ret;
}

RadarIQReturnVal_t RadarIQ_getPointDensity(const RadarIQHandle_t obj, RadarIQPointDensity_t * const density)
{
	radariq_assert(NULL != obj);

	RadarIQReturnVal_t ret = RADARIQ_RETURN_VAL_OK;

	obj->txPacket.data[0] = RADARIQ_CMD_PNT_DENSITY;
	obj->txPacket.data[1] = RADARIQ_CMD_VAR_REQUEST;
	obj->txPacket.len = 2u;

	RadarIQ_sendPacket(obj);

	if (RADARIQ_CMD_PNT_DENSITY == RadarIQ_pollResponse(obj))
	{
		*density = (RadarIQPointDensity_t)obj->rxPacket.data[2];
	}
	else
	{
		ret = RADARIQ_RETURN_VAL_ERR;	
	}

	return ret;
}

RadarIQReturnVal_t RadarIQ_setPointDensity(const RadarIQHandle_t obj, RadarIQPointDensity_t density)
{
	radariq_assert(NULL != obj);

	RadarIQReturnVal_t ret = RADARIQ_RETURN_VAL_OK;

	if ((RADARIQ_DENSITY_NORMAL > density) || (RADARIQ_DENSITY_VERY_DENSE < density))
	{
		ret = RADARIQ_RETURN_VAL_ERR;
		return ret;	
	}

	obj->txPacket.data[0] = RADARIQ_CMD_PNT_DENSITY;
	obj->txPacket.data[1] = RADARIQ_CMD_VAR_SET;
	obj->txPacket.data[2] = (uint8_t)density;
	obj->txPacket.len = 3u;

	RadarIQ_sendPacket(obj);

	if (RADARIQ_CMD_PNT_DENSITY != RadarIQ_pollResponse(obj))
	{
		ret = RADARIQ_RETURN_VAL_ERR;	
	}

	return ret;
}

RadarIQReturnVal_t RadarIQ_getCertainty(const RadarIQHandle_t obj, uint8_t * const certainty)
{
	radariq_assert(NULL != obj);

	RadarIQReturnVal_t ret = RADARIQ_RETURN_VAL_OK;

	obj->txPacket.data[0] = RADARIQ_CMD_CERTAINTY;
	obj->txPacket.data[1] = RADARIQ_CMD_VAR_REQUEST;
	obj->txPacket.len = 2u;

	RadarIQ_sendPacket(obj);

	if (RADARIQ_CMD_CERTAINTY == RadarIQ_pollResponse(obj))
	{
		*certainty = obj->rxPacket.data[2];
	}
	else
	{
		ret = RADARIQ_RETURN_VAL_ERR;	
	}

	return ret;
}

RadarIQReturnVal_t RadarIQ_setCertainty(const RadarIQHandle_t obj, uint8_t certainty)
{
	radariq_assert(NULL != obj);

	RadarIQReturnVal_t ret = RADARIQ_RETURN_VAL_OK;

	if (RADARIQ_MAX_CERTAINTY < certainty)
	{
		certainty = RADARIQ_MAX_CERTAINTY;
		ret = RADARIQ_RETURN_VAL_WARNING;	
	}

	obj->txPacket.data[0] = RADARIQ_CMD_CERTAINTY;
	obj->txPacket.data[1] = RADARIQ_CMD_VAR_SET;
	obj->txPacket.data[2] = certainty;
	obj->txPacket.len = 3u;

	RadarIQ_sendPacket(obj);

	if (RADARIQ_CMD_CERTAINTY != RadarIQ_pollResponse(obj))
	{
		ret = RADARIQ_RETURN_VAL_ERR;	
	}

	return ret;
}

RadarIQReturnVal_t RadarIQ_getHeightFilter(const RadarIQHandle_t obj, int16_t * const min, int16_t * const max)
{
	radariq_assert(NULL != obj);

	RadarIQReturnVal_t ret = RADARIQ_RETURN_VAL_OK;
	
	obj->txPacket.data[0] = RADARIQ_CMD_HEIGHT_FILT;
	obj->txPacket.data[1] = RADARIQ_CMD_VAR_REQUEST;
	obj->txPacket.len = 2u;

	RadarIQ_sendPacket(obj);

	if (RADARIQ_CMD_HEIGHT_FILT == RadarIQ_pollResponse(obj))
	{	
		*min = bytePack16Signed(&obj->rxPacket.data[2]);
		*max = bytePack16Signed(&obj->rxPacket.data[4]);
	}
	else
	{
		ret = RADARIQ_RETURN_VAL_ERR;
	}

	return ret;
}

RadarIQReturnVal_t RadarIQ_setHeightFilter(const RadarIQHandle_t obj, int16_t min, int16_t max)
{
	radariq_assert(NULL != obj);

	RadarIQReturnVal_t ret = RADARIQ_RETURN_VAL_OK;

	if (min > max)
	{
		int16_t temp = min;
		min = max;
		max = temp;	
	}
	
	obj->txPacket.data[0] = RADARIQ_CMD_HEIGHT_FILT;
	obj->txPacket.data[1] = RADARIQ_CMD_VAR_SET;
	byteUnpack16Signed(min, &obj->txPacket.data[2]);
	byteUnpack16Signed(max, &obj->txPacket.data[4]);	
	obj->txPacket.len = 6u;
	
	RadarIQ_sendPacket(obj);

	if (RADARIQ_CMD_HEIGHT_FILT != RadarIQ_pollResponse(obj))
	{
		ret = RADARIQ_RETURN_VAL_ERR;	
	}

	return ret;
}

RadarIQReturnVal_t RadarIQ_sceneCalibrate(const RadarIQHandle_t obj)
{
	radariq_assert(NULL != obj);
	
	RadarIQReturnVal_t ret = RADARIQ_RETURN_VAL_ERR;
	
	obj->txPacket.data[0] = RADARIQ_CMD_SCENE_CALIB;
	obj->txPacket.data[1] = RADARIQ_CMD_VAR_SET;
	obj->txPacket.len = 2u;

	RadarIQ_sendPacket(obj);
	
	// Poll for acknowledgement message
	// Several other messages are expected to be recieved before the ack
	for (uint32_t poll = 0u; poll < 50u; poll++)
	{
		if (RADARIQ_CMD_SCENE_CALIB == RadarIQ_pollResponse(obj))
		{
			ret = RADARIQ_RETURN_VAL_OK;
			break;	
		}
	}
	
	return ret;
}

RadarIQReturnVal_t RadarIQ_getObjectSize(const RadarIQHandle_t obj, uint8_t * const size)
{
	radariq_assert(NULL != obj);

	RadarIQReturnVal_t ret = RADARIQ_RETURN_VAL_OK;

	obj->txPacket.data[0] = RADARIQ_CMD_OBJECT_SIZE;
	obj->txPacket.data[1] = RADARIQ_CMD_VAR_REQUEST;
	obj->txPacket.len = 2u;

	RadarIQ_sendPacket(obj);

	if (RADARIQ_CMD_OBJECT_SIZE == RadarIQ_pollResponse(obj))
	{
		*size = obj->rxPacket.data[2];
	}
	else
	{
		ret = RADARIQ_RETURN_VAL_ERR;	
	}

	return ret;
}

RadarIQReturnVal_t RadarIQ_setObjectSize(const RadarIQHandle_t obj, uint8_t size)
{
	radariq_assert(NULL != obj);

	RadarIQReturnVal_t ret = RADARIQ_RETURN_VAL_OK;

	if (RADARIQ_MAX_OBJ_SIZE < size)
	{
		size = RADARIQ_MAX_OBJ_SIZE;
		ret = RADARIQ_RETURN_VAL_WARNING;	
	}

	obj->txPacket.data[0] = RADARIQ_CMD_OBJECT_SIZE;
	obj->txPacket.data[1] = RADARIQ_CMD_VAR_SET;
	obj->txPacket.data[2] = size;
	obj->txPacket.len = 3u;

	RadarIQ_sendPacket(obj);

	if (RADARIQ_CMD_OBJECT_SIZE != RadarIQ_pollResponse(obj))
	{
		ret = RADARIQ_RETURN_VAL_ERR;	
	}

	return ret;	
}

//===============================================================================================//
// FILE-SCOPE FUNCTIONS - Packet Parsing
//===============================================================================================//

static RadarIQCommand_t RadarIQ_pollResponse(const RadarIQHandle_t obj)
{
	radariq_assert(NULL != obj);

	RadarIQCommand_t response;

    int64_t startTime = (int64_t)radariq_get_mseconds;
    char strMsg[32]; 
    //sprintf(strMsg, "pollResponse: start time = %i", startTime);
    //obj->logCallback(strMsg);
	
	do
	{		
		response = RadarIQ_readSerial(obj);
	
		if (RADARIQ_CMD_NONE < response)
		{
			//sprintf(strMsg, "pollResponse: response %i", response);
			//obj->logCallback(strMsg);
			break;	
		}
	}while (startTime >= ((int64_t)radariq_get_mseconds - 1000));

	if (RADARIQ_CMD_NONE >= response)
	{
		//sprintf(strMsg, "pollResponse: timeout at %i", radariq_get_seconds);
		//obj->logCallback(strMsg);
	}

	return response;
}

static RadarIQCommand_t RadarIQ_parsePacket(const RadarIQHandle_t obj)
{
	radariq_assert(NULL != obj);
	
	const RadarIQCommand_t command = (RadarIQCommand_t)obj->rxPacket.data[0];
	RadarIQCommand_t ret = command;

	//TODO: check command variant is response

	switch (command)
	{
	case RADARIQ_CMD_MESSAGE:
	{
		RadarIQ_parseMessage(obj);
		break;	
	}
    case RADARIQ_CMD_VERSION:       
    case RADARIQ_CMD_SERIAL:        
    case RADARIQ_CMD_RESET:          	
    case RADARIQ_CMD_FRAME_RATE:      	
    case RADARIQ_CMD_MODE:      	
    case RADARIQ_CMD_DIST_FILT:       	
    case RADARIQ_CMD_ANGLE_FILT:       	
    case RADARIQ_CMD_MOVING_FILT:      	
    case RADARIQ_CMD_SAVE:     	
    case RADARIQ_CMD_PNT_DENSITY:
    case RADARIQ_CMD_CERTAINTY:
    case RADARIQ_CMD_HEIGHT_FILT:
    case RADARIQ_CMD_IWR_VERSION:
    case RADARIQ_CMD_SCENE_CALIB:
	case RADARIQ_CMD_OBJECT_SIZE:
	{
    	break;	
	}
	case RADARIQ_CMD_PNT_CLOUD_FRAME:
	{
		//obj->logCallback("parsePacket: Point Cloud");
		RadarIQ_parsePointCloud(obj);
		break;
	}
	case RADARIQ_CMD_OBJ_TRACKING_FRAME:
	{
		//obj->logCallback("parsePacket: Object Tracking");
		RadarIQ_parseObjectTracking(obj);
		break;
	}
	case RADARIQ_CMD_PROC_STATS:
	{
		//obj->logCallback("parsePacket: Processing Stats");
		RadarIQ_parseProcessingStats(obj);
		break;
	}
	case RADARIQ_CMD_POINTCLOUD_STATS:
	{
		//obj->logCallback("parsePacket: Point Cloud Stats");
		RadarIQ_parsePointCloudStats(obj);
		break;	
	}
	case RADARIQ_CMD_POWER_STATUS:
	{
		//obj->logCallback("parsePacket: Power status");
		RadarIQ_parsePowerStatus(obj);
		break;	
	}
	default:
	{
		obj->logCallback("parsePacket: Unknown command");
		ret = RADARIQ_CMD_UNKNOWN;
		break;
	}
	}

	return ret;
}

static void RadarIQ_parsePointCloud(const RadarIQHandle_t obj)
{
	const RadarIQSubframe_t subFrameType = (RadarIQSubframe_t)obj->rxPacket.data[2];
	uint8_t pointCount = obj->rxPacket.data[3];
	obj->data.pointCloud.isFrameComplete = false;
	obj->data.pointCloud.numPoints = 0u;
	uint8_t packetIdx = 4u;

	// Loop through points in packet
	uint8_t pointNum;
	for (pointNum = 0u; pointNum < pointCount; pointNum++)
	{
        obj->data.pointCloud.points[obj->numDataPoints].x = bytePack16Signed(&obj->rxPacket.data[packetIdx]);
		packetIdx += 2u;
		obj->data.pointCloud.points[obj->numDataPoints].y = bytePack16Signed(&obj->rxPacket.data[packetIdx]);
        packetIdx += 2u;
        obj->data.pointCloud.points[obj->numDataPoints].z = bytePack16Signed(&obj->rxPacket.data[packetIdx]);
        packetIdx += 2u;
        obj->data.pointCloud.points[obj->numDataPoints].intensity = obj->rxPacket.data[packetIdx];
        packetIdx++;
        obj->data.pointCloud.points[obj->numDataPoints].velocity = bytePack16Signed(&obj->rxPacket.data[packetIdx]);
        packetIdx += 2u;

		obj->numDataPoints++;
		obj->data.pointCloud.numPoints = obj->numDataPoints;
		if (RADARIQ_MAX_POINTCLOUD < (obj->numDataPoints + 1u))
		{
			break;
		}
	}

	// Check if sub-frame type is end of frame
	if (RADARIQ_SUBFRAME_END == subFrameType)
	{
		// Reset point counter for frame
		obj->numDataPoints = 0u;

		// If loop completed then frame is complete
		if (pointNum == (pointCount - 1u))
		{
			obj->data.pointCloud.isFrameComplete = true;
		}
	}
}

static void RadarIQ_parseObjectTracking(const RadarIQHandle_t obj)
{
	const RadarIQSubframe_t subFrameType = (RadarIQSubframe_t)obj->rxPacket.data[2];
	uint8_t objectCount = obj->rxPacket.data[3];
	obj->data.objectTracking.isFrameComplete = false;
	obj->data.objectTracking.numObjects = 0u;
	uint8_t packetIdx = 4u;

	// Loop through points in packet
	uint8_t objectNum;
	for (objectNum = 0u; objectNum < objectCount; objectNum++)
	{
		obj->data.objectTracking.objects[obj->numDataPoints].targetId = obj->rxPacket.data[packetIdx];
		packetIdx++;
		obj->data.objectTracking.objects[obj->numDataPoints].xPos = bytePack16Signed(&obj->rxPacket.data[packetIdx]);
		packetIdx += 2u;
		obj->data.objectTracking.objects[obj->numDataPoints].yPos = bytePack16Signed(&obj->rxPacket.data[packetIdx]);
		packetIdx += 2u;
		obj->data.objectTracking.objects[obj->numDataPoints].zPos = bytePack16Signed(&obj->rxPacket.data[packetIdx]);
		packetIdx += 2u;
		obj->data.objectTracking.objects[obj->numDataPoints].xVel = bytePack16Signed(&obj->rxPacket.data[packetIdx]);
		packetIdx += 2u;
		obj->data.objectTracking.objects[obj->numDataPoints].yVel = bytePack16Signed(&obj->rxPacket.data[packetIdx]);
		packetIdx += 2u;
		obj->data.objectTracking.objects[obj->numDataPoints].zVel = bytePack16Signed(&obj->rxPacket.data[packetIdx]);
		packetIdx += 2u;
		obj->data.objectTracking.objects[obj->numDataPoints].xAcc = bytePack16Signed(&obj->rxPacket.data[packetIdx]);
		packetIdx += 2u;
		obj->data.objectTracking.objects[obj->numDataPoints].yAcc = bytePack16Signed(&obj->rxPacket.data[packetIdx]);
		packetIdx += 2u;
		obj->data.objectTracking.objects[obj->numDataPoints].zAcc = bytePack16Signed(&obj->rxPacket.data[packetIdx]);
		packetIdx += 2u;

		obj->numDataPoints++;
		obj->data.objectTracking.numObjects = obj->numDataPoints;
		if (RADARIQ_MAX_OBJECTS < (obj->numDataPoints + 1u))
		{
			break;
		}
	}

	// Check if sub-frame type is end of frame
	if (RADARIQ_SUBFRAME_END == subFrameType)
	{
		// Reset point counter for frame
		obj->numDataPoints = 0u;

		// If loop completed then frame is complete
		if (objectNum == (objectCount - 1u))
		{
			obj->data.objectTracking.isFrameComplete = true;
		}
	}
}

static void RadarIQ_parseMessage(const RadarIQHandle_t obj)
{
	radariq_assert(NULL != obj);
	
	char strLog[256];
	char strMsgType[16];
	
	RadarIQMsgType_t msgType = (RadarIQMsgType_t)obj->rxPacket.data[2]; 
	
	switch (msgType)
	{
		case RADARIQ_MSG_TYPE_TEMPORARY:
			sprintf(strMsgType, "%s", "TEMPORARY");
			break;	
		case RADARIQ_MSG_TYPE_DEBUG:
			sprintf(strMsgType, "%s", "DEBUG");
			break;	
		case RADARIQ_MSG_TYPE_INFO:
			sprintf(strMsgType, "%s", "INFO");
			break;	
		case RADARIQ_MSG_TYPE_WARNING:
			sprintf(strMsgType, "%s", "WARNING");
			break;	
		case RADARIQ_MSG_TYPE_ERROR:
			sprintf(strMsgType, "%s", "ERROR");
			break;	
		case RADARIQ_MSG_TYPE_SUCCESS:
			sprintf(strMsgType, "%s", "SUCCESS");
			break;	
	}
	
	snprintf(strLog, sizeof(strLog), "Radar Message - %s (%u):, %s", strMsgType, obj->rxPacket.data[3], (char*)&obj->rxPacket.data[4]);
	obj->logCallback(strLog);
}

static void RadarIQ_parseProcessingStats(const RadarIQHandle_t obj)
{
	radariq_assert(NULL != obj);
	
	memcpy((void*)&obj->stats.processing, (void*)&obj->rxPacket.data[2], 28);
	memcpy((void*)&obj->stats.temperature, (void*)&obj->rxPacket.data[30], 20);
}

static void RadarIQ_parsePointCloudStats(const RadarIQHandle_t obj)
{
	radariq_assert(NULL != obj);
	
	memcpy((void*)&obj->stats.pointcloud, (void*)&obj->rxPacket.data[2], 26);	
}

static void RadarIQ_parsePowerStatus(const RadarIQHandle_t obj)
{
	radariq_assert(NULL != obj);
	
	obj->isPowerGood = !obj->rxPacket.data[2];
}

static void RadarIQ_sendPacket(const RadarIQHandle_t obj)
{
	radariq_assert(NULL != obj);

	// Calculate CRC
	uint16_t const crc = getCrc16Ccitt(obj->txPacket.data, obj->txPacket.len);

	// Send packet header
	obj->txBuffer.data[0] = RADARIQ_PACKET_HEAD;
	obj->txBuffer.len = 1u;

	// Loop through data, check for footer bytes in data and escape them
	for (uint8_t idx = 0u; idx < obj->txPacket.len; idx++)
	{
		encodeHelper(obj, obj->txPacket.data[idx]);
	}

	encodeHelper(obj, (uint8_t)((crc & (uint16_t)0xFF00) >> 8u));
	encodeHelper(obj, (uint8_t)(crc & (uint16_t)0x00FF));

	// Insert footer
	obj->txBuffer.data[obj->txBuffer.len] = RADARIQ_PACKET_FOOT;
	obj->txBuffer.len++;

	obj->sendSerialDataCallback(obj->txBuffer.data, obj->txBuffer.len);
}

static bool RadarIQ_decodePacket(const RadarIQHandle_t obj)
{
	radariq_assert(NULL != obj);

	bool retval;
	obj->rxPacket.len = 0u;
	uint8_t srcIdx = 0u;

	// Loop through all bytes except footer
	while(srcIdx < (obj->rxBuffer.len - 1u)) //
	{
		if(obj->rxPacket.len >= (RADARIQ_RX_BUFFER_SIZE - 1u))
		{
			return false;
		}

		switch (*(obj->rxBuffer.data + srcIdx))
		{
			case RADARIQ_PACKET_HEAD:
				srcIdx++;
				break;

			case RADARIQ_PACKET_ESC:
				srcIdx++;
				*(obj->rxPacket.data + obj->rxPacket.len) = (*(obj->rxBuffer.data + srcIdx) ^ RADARIQ_PACKET_XOR);
				obj->rxPacket.len++;
				srcIdx++;
				break;

			default:
				*(obj->rxPacket.data + obj->rxPacket.len) = *(obj->rxBuffer.data + srcIdx);
				obj->rxPacket.len++;
				srcIdx++;
		}
	}

	// Calculates crc from decoded packet (crc calc does not include header, footer, or crc bytes)
	uint16_t const crc = getCrc16Ccitt(obj->rxPacket.data, obj->rxPacket.len - 2u);

	// Gets crc from the packet after it has been decoded
	uint16_t const rxCrc = (uint16_t) ( (*(obj->rxPacket.data + obj->rxPacket.len - 2u) & (uint16_t)0xFF) << 8u) |
			(*(obj->rxPacket.data + obj->rxPacket.len - 1u) & (uint16_t)0xFF);

	return (crc == rxCrc);
}

static void encodeHelper(const RadarIQHandle_t obj, uint8_t const databyte)
{
	radariq_assert(obj->txBuffer.len < (RADARIQ_TX_BUFFER_SIZE - 2u));

	switch (databyte)
	{
		case RADARIQ_PACKET_HEAD:
		{
			// Escape character
			obj->txBuffer.data[obj->txBuffer.len] = RADARIQ_PACKET_ESC;
			obj->txBuffer.len++;
			obj->txBuffer.data[obj->txBuffer.len] = RADARIQ_PACKET_HEAD ^ RADARIQ_PACKET_XOR;
			obj->txBuffer.len++;

			break;
		}
		case RADARIQ_PACKET_FOOT:
		{
			// Escape character
			obj->txBuffer.data[obj->txBuffer.len] = RADARIQ_PACKET_ESC;
			obj->txBuffer.len++;
			obj->txBuffer.data[obj->txBuffer.len] = RADARIQ_PACKET_FOOT ^ RADARIQ_PACKET_XOR;
			obj->txBuffer.len++;

			break;
		}
		case RADARIQ_PACKET_ESC:
		{
			// Escape character
			obj->txBuffer.data[obj->txBuffer.len] = RADARIQ_PACKET_ESC;
			obj->txBuffer.len++;
			obj->txBuffer.data[obj->txBuffer.len] = RADARIQ_PACKET_ESC ^ RADARIQ_PACKET_XOR;
			obj->txBuffer.len++;

			break;
		}
		default:
		{
			obj->txBuffer.data[obj->txBuffer.len] = databyte;
			obj->txBuffer.len++;
		}
	}
}

static uint16_t getCrc16Ccitt(uint8_t const * array, uint8_t len)
{
    uint8_t x;
    uint16_t crc = (uint16_t)0xFFFFu;

    for (uint8_t idx = 0u; idx < len; idx++)
    {
    	x = crc >> 8u ^ *array++;
		x ^= x>>4u;
		crc = (crc << 8u) ^ ((uint16_t)(x << 12u)) ^ ((uint16_t)(x << 5u)) ^ ((uint16_t)x); //lint !e734)
	}

    return crc;
}

//===============================================================================================//
// FILE-SCOPE FUNCTIONS - Byte Helpers
//===============================================================================================//

static uint32_t bytePack32(const uint8_t * const data)
{
    uint32_t dest = 0u;
    dest |= (((uint32_t)data[3] << 24u) & 0xFF000000u);
    dest |= (((uint32_t)data[2] << 16u) & 0x00FF0000u);
    dest |= (((uint32_t)data[1] << 8u) & 0x0000FF00u);
    dest |= (((uint32_t)data[0]) & 0x000000FFu);

    return dest;
}

static uint16_t bytePack16(const uint8_t * const data)
{
    uint16_t dest = 0u;
    dest |= (((uint32_t)data[1] << 8u) & 0xFF00u);
    dest |= (((uint32_t)data[0]) & 0x00FFu);

    return dest;
}

static int16_t bytePack16Signed(const uint8_t * const data)
{
    uint16_t temp = 0u;
    int16_t returnValue = 0;

    temp |= (((uint32_t)data[1] << 8u) & 0xFF00u);
    temp |= (((uint32_t)data[0]) & 0x00FFu);

    memcpy(&returnValue, &temp, sizeof(uint16_t));

    return returnValue;
}

static void byteUnpack32(const uint32_t data, uint8_t * const dest)
{
	dest[3] = (data >> 24u) & 0xFFu;
	dest[2] = (data >> 16u) & 0xFFu;
	dest[1] = (data >> 8u) & 0xFFu;
	dest[0] = data & 0xFFu;
}

static void byteUnpack32Signed(const int32_t data, uint8_t * const dest)
{
    uint32_t temp = 0u;
    memcpy(&temp, &data, sizeof(int32_t));

	dest[3] = (temp >> 24u) & 0xFFu;
	dest[2] = (temp >> 16u) & 0xFFu;
	dest[1] = (temp >> 8u) & 0xFFu;
	dest[0] = temp & 0xFFu;
}

static void byteUnpack16(uint16_t const data, uint8_t * const dest)
{
	dest[1] = (data >> 8u) & 0xFFu;
	dest[0] = data & 0xFFu;
}

static void byteUnpack16Signed(int16_t const data, uint8_t * const dest)
{
    uint16_t temp = 0u;
    memcpy(&temp, &data, sizeof(uint16_t));

	dest[1] = (temp >> 8u) & 0xFFu;
	dest[0] = temp & 0xFFu;
}

static int32_t bytePack32Signed(const uint8_t * const data)
{
    uint32_t temp = 0u;
    int32_t returnValue = 0;

    temp |= (((uint32_t)data[3] << 24u) & 0xFF000000u);
    temp |= (((uint32_t)data[2] << 16u) & 0x00FF0000u);
    temp |= (((uint32_t)data[1] << 8u) & 0x0000FF00u);
    temp |= (((uint32_t)data[0]) & 0x000000FFu);

    memcpy(&returnValue, &temp, sizeof(uint16_t));

    return returnValue;
}
