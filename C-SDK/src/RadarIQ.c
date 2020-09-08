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
    RADARIQ_CMD_MESSAGE          = 0x00,
    RADARIQ_CMD_VERSION          = 0x01,
    RADARIQ_CMD_SERIAL           = 0x02,
    RADARIQ_CMD_RESET            = 0x03,
    RADARIQ_CMD_FRAME_RATE       = 0x04,
    RADARIQ_CMD_MODE             = 0x05,
    RADARIQ_CMD_DIST_FILT        = 0x06,
    RADARIQ_CMD_ANGLE_FILT       = 0x07,
    RADARIQ_CMD_MOVING_FILT      = 0x08,
    RADARIQ_CMD_SAVE             = 0x09,
    RADARIQ_CMD_PNT_DENSITY      = 0x10,
    RADARIQ_CMD_CERTAINTY        = 0x11,
    RADARIQ_CMD_HEIGHT_FILT      = 0x12,
    RADARIQ_CMD_RAW_DP_ENABLE    = 0x13,
    RADARIQ_CMD_IWR_VERSION      = 0x14,
    RADARIQ_CMD_CAPTURE_START    = 0x64,
    RADARIQ_CMD_CAPTURE_STOP     = 0x65,
    RADARIQ_CMD_PNT_CLOUD_FRAME  = 0x66
} RadarIQCommands_t;

typedef enum
{
    RADARIQ_CMD_VAR_REQUEST     = 0,
    RADARIQ_CMD_VAR_RESPONSE    = 1,
    RADARIQ_CMD_VAR_SET         = 2
} RadarIQommandVariants_t;

typedef enum
{
	RADARIQ_PACKET_OK = 0x00,		//<! Everything OK
	RADARIQ_PACKET_CRC_FAIL,		//<! CRC does not match (during decoding)
	RADARIQ_PACKET_DEST_OVERFLOW,	//<! Destination buffer not big enough
    RADARIQ_PACKET_NO_HEAD,
    RADARIQ_PACKET_NO_FOOT
} RadarIQPacketStatus_t;

typedef struct
{
	RadarIQPacketStatus_t 	status;		//<! Status of encoding function
	uint8_t					len;		//<! Length of encoded packet
} RadarIQPacketReturn_t;

typedef enum
{
	PARSING_STATE_WAITING_FOR_HEADER,
	PARSING_STATE_WAITING_FOR_FOOTER,
	PARSING_STATE_READING_PACKET,
} RadarIQParsingState_t;

typedef struct
{
	uint8_t data[RADARIQ_PACKET_BUFFER_SIZE];
	uint16_t len;
} RadarIQPacketBuffer_t;

//===============================================================================================//
// OBJECTS
//===============================================================================================//

struct RadarIQ_t
{
	RadarIQCaptureMode_t captureMode;
	RadarIQResetCode_t resetCode;
	RadarIQPointDensity_t pointDensity;

	RadarIQData_t data;
	RadarIQStatistics_t stats;

	RadarIQPacketBuffer_t rxBuffer;
	RadarIQPacketBuffer_t dataBuffer;
	RadarIQPacketBuffer_t txBuffer;
	RadarIQParsingState_t parsingState;

	void(*sendSerialDataCallback)(uint8_t * const, const uint16_t);
	uint8_t(*readSerialDataCallback)(void);
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

RadarIQPacketReturn_t RadarIQ_sendPacket(const RadarIQHandle_t obj);
RadarIQPacketReturn_t RadarIQ_decodePacket(const RadarIQHandle_t obj);
static uint16_t RadarIQ_GetCrc16Ccitt(uint8_t const * array, uint8_t len);
static void encodeHelper(const RadarIQHandle_t obj, uint8_t const byte);

//===============================================================================================//
// GLOBAL-SCOPE FUNCTIONS
//===============================================================================================//

RadarIQHandle_t RadarIQ_init(void(*sendSerialDataCallback)(uint8_t * const, const uint16_t),
		uint8_t(*readSerialDataCallback)(void),
		void(*logCallback)(char * const))
{
	radariq_assert(NULL != sendSerialDataCallback);
	radariq_assert(NULL != readSerialDataCallback);
	radariq_assert(NULL != logCallback);

	RadarIQHandle_t handle = malloc(sizeof(RadarIQ_t));
	radariq_assert(NULL != handle);

	handle->sendSerialDataCallback = sendSerialDataCallback;
	handle->readSerialDataCallback = readSerialDataCallback;
	handle->logCallback = logCallback;

	handle->captureMode = RADARIQ_MODE_POINT_CLOUD;
	handle->parsingState = PARSING_STATE_WAITING_FOR_HEADER;

	handle->rxBuffer.len = 0u;
	handle->txBuffer.len = 0u;
	handle->dataBuffer.len = 0u;

	return handle;
}

bool RadarIQ_readSerial(const RadarIQHandle_t obj)
{
	radariq_assert(NULL != obj);

	bool isDataReady = false;
	uint8_t rxByte = obj->readSerialDataCallback();
	static uint16_t idx;

	switch(obj->parsingState)
	{
	case PARSING_STATE_WAITING_FOR_HEADER:
	{
		if (RADARIQ_PACKET_HEAD == rxByte)
		{
			obj->rxBuffer.data[0] = rxByte;
			idx = 1u;

			obj->parsingState = PARSING_STATE_WAITING_FOR_FOOTER;
		}

		break;
	}
	case PARSING_STATE_WAITING_FOR_FOOTER:
	{
		obj->rxBuffer.data[idx] = rxByte;
		idx = (idx + 1) % RADARIQ_PACKET_BUFFER_SIZE;
		obj->rxBuffer.len = idx;

		if (RADARIQ_PACKET_FOOT == rxByte)
		{
			obj->parsingState = PARSING_STATE_WAITING_FOR_HEADER;

			idx = 0;

			RadarIQPacketReturn_t ret = RadarIQ_decodePacket(obj);
			if (ret.status == RADARIQ_PACKET_OK) isDataReady = true;
		}

		break;
	}
	case PARSING_STATE_READING_PACKET:
	{
		break;
	}
	default:
	{
		break;
	}
	}


	return isDataReady;
}

 RadarIQReturnVal_t RadarIQ_getData(const RadarIQHandle_t obj, RadarIQData_t * const dest)
{
	radariq_assert(NULL != obj);
	radariq_assert(NULL != dest);

	*dest = obj->data;

	return RADARIQ_RETURN_VAL_OK;
}

uint32_t RadarIQ_getMemoryUsage()
{
	return sizeof(RadarIQ_t);
}

RadarIQReturnVal_t RadarIQ_start(const RadarIQHandle_t obj, const uint8_t numFrames)
{
	radariq_assert(NULL != obj);

	obj->txBuffer.data[0] = RADARIQ_CMD_CAPTURE_START;
	obj->txBuffer.data[1] = RADARIQ_CMD_VAR_REQUEST;
	obj->txBuffer.data[2] = numFrames;
	obj->txBuffer.len = 3u;

	if (RadarIQ_sendPacket(obj).status == RADARIQ_PACKET_OK)
	{
		return RADARIQ_RETURN_VAL_OK;
	}
	else
	{
		return RADARIQ_RETURN_VAL_ERR;
	}
}

uint8_t RadarIQ_getDataBuffer(const RadarIQHandle_t obj, uint8_t* dest)
{
	radariq_assert(NULL != obj);

	memcpy(dest, obj->rxBuffer.data, obj->rxBuffer.len);

	return obj->dataBuffer.len;
}

//===============================================================================================//
// FILE-SCOPE FUNCTIONS
//===============================================================================================//

RadarIQPacketReturn_t RadarIQ_sendPacket(const RadarIQHandle_t obj)
{
	radariq_assert(NULL != obj);

	RadarIQPacketReturn_t retval;
	retval.status = RADARIQ_PACKET_OK;

	// Calculate CRC
	uint16_t const crc = RadarIQ_GetCrc16Ccitt(obj->txBuffer.data, obj->txBuffer.len);

	// Send packet header
	uint8_t txByte = RADARIQ_PACKET_HEAD;
	obj->sendSerialDataCallback(&txByte, 1u);

	// Loop through data, check for footer bytes in data and escape them
	for (uint8_t idx = 0u; idx < obj->txBuffer.len; idx++)
	{
		encodeHelper(obj, obj->txBuffer.data[idx]);
	}

	encodeHelper(obj, (uint8_t)((crc & (uint16_t)0xFF00) >> 8u));
	encodeHelper(obj, (uint8_t)(crc & (uint16_t)0x00FF));

	// Send footer
	txByte = RADARIQ_PACKET_FOOT;
	obj->sendSerialDataCallback(&txByte, 1u);

	return retval;
}

RadarIQPacketReturn_t RadarIQ_decodePacket(const RadarIQHandle_t obj)
{
	// Check parameters are valid
	assert(NULL != obj);

	RadarIQPacketReturn_t retval;
	uint8_t destIdx = 0u;
	uint8_t srcIdx = 0u;

	// Loop through all bytes except footer
	while(srcIdx < (obj->rxBuffer.len - 1u)) //
	{
		if(destIdx >= (RADARIQ_PACKET_BUFFER_SIZE-1u))
		{
			retval.status = RADARIQ_PACKET_DEST_OVERFLOW;
			retval.len = destIdx;
			return retval;
		}

		switch (*(obj->rxBuffer.data + srcIdx))
		{
			case RADARIQ_PACKET_HEAD:
				srcIdx++;
				break;

			case RADARIQ_PACKET_ESC:
				srcIdx++;
				*(obj->dataBuffer.data + destIdx) = (*(obj->rxBuffer.data + srcIdx) ^ RADARIQ_PACKET_XOR);
				destIdx++;
				srcIdx++;
				break;

			default:
				*(obj->dataBuffer.data + destIdx) = *(obj->rxBuffer.data + srcIdx);
				destIdx++;
				srcIdx++;
		}
	}

	// Calculates crc from decoded packet (crc calc does not include header, footer, or crc bytes)
	uint16_t const crc = RadarIQ_GetCrc16Ccitt(obj->dataBuffer.data, destIdx - 2u);

	// Gets crc from the packet after it has been decoded
	uint16_t const rxCrc = (uint16_t) ( (*(obj->dataBuffer.data + destIdx - 2u) & (uint16_t)0xFF) << 8u) |
			(*(obj->dataBuffer.data + destIdx - 1u) & (uint16_t)0xFF);

	if(crc != rxCrc)
	{
		retval.status = RADARIQ_PACKET_CRC_FAIL;
		retval.len = 0u;
	}
	else
	{
		retval.status = RADARIQ_PACKET_OK;
		retval.len = destIdx - 2u;
		obj->dataBuffer.len = destIdx - 2u;
	}

	return retval;
}

static void encodeHelper(const RadarIQHandle_t obj, uint8_t const byte)
{
	uint8_t txBuff[2];

	switch (byte)
	{
		case RADARIQ_PACKET_HEAD:
		{
			// Escape character
			//*(dest + destIdx) = RADARIQ_PACKET_ESC;
			//destIdx++;
			txBuff[0] = RADARIQ_PACKET_ESC;
			txBuff[1] = RADARIQ_PACKET_HEAD ^ RADARIQ_PACKET_XOR;

			obj->sendSerialDataCallback(txBuff, 2u);

			//*(dest + destIdx) = RADARIQ_PACKET_HEAD ^ RADARIQ_PACKET_XOR;
			//destIdx++;

			break;
		}
		case RADARIQ_PACKET_FOOT:
		{
			// Escape character
			//*(dest + destIdx) = RADARIQ_PACKET_ESC;
			//destIdx++;

			txBuff[0] = RADARIQ_PACKET_ESC;
			txBuff[1] = RADARIQ_PACKET_FOOT ^ RADARIQ_PACKET_XOR;

			obj->sendSerialDataCallback(txBuff, 2u);

			//*(dest + destIdx) = RADARIQ_PACKET_FOOT ^ RADARIQ_PACKET_XOR;
			//destIdx++;

			break;
		}
		case RADARIQ_PACKET_ESC:
		{
			// Escape character
			//*(dest + destIdx) = RADARIQ_PACKET_ESC;
			//destIdx++;

			txBuff[0] = RADARIQ_PACKET_ESC;
			txBuff[1] = RADARIQ_PACKET_ESC ^ RADARIQ_PACKET_XOR;

			obj->sendSerialDataCallback(txBuff, 2u);

			//*(dest + destIdx) = RADARIQ_PACKET_ESC ^ RADARIQ_PACKET_XOR;
			//destIdx++;

			break;
		}
		default:
		{
			//*(dest + destIdx) = byte;
			//destIdx++;

			txBuff[0] = byte;

			obj->sendSerialDataCallback(txBuff, 1u);
		}
	}
}

static uint16_t RadarIQ_GetCrc16Ccitt(uint8_t const * array, uint8_t len)
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

