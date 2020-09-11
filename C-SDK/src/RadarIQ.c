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
    RADARIQ_CMD_VAR_REQUEST     = 0,
    RADARIQ_CMD_VAR_RESPONSE    = 1,
    RADARIQ_CMD_VAR_SET         = 2
} RadarIQommandVariant_t;

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

	RadarIQRxBuffer_t rxBuffer;
	RadarIQRxBuffer_t rxPacket;
	RadarIQTxBuffer_t txBuffer;
	RadarIQTxBuffer_t txPacket;
	RadarIQRxState_t rxState;

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

static void RadarIQ_sendPacket(const RadarIQHandle_t obj);
static RadarIQReturnVal_t RadarIQ_decodePacket(const RadarIQHandle_t obj);
static RadarIQCommand_t RadarIQ_parsePacket(const RadarIQHandle_t obj);
static void RadarIQ_parsePointCloud(const RadarIQHandle_t obj);
static uint16_t getCrc16Ccitt(uint8_t const * array, uint8_t len);
static void encodeHelper(const RadarIQHandle_t obj, uint8_t const byte);
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
		uint8_t(*readSerialDataCallback)(void),
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
	uint8_t rxByte = obj->readSerialDataCallback();

	switch(obj->rxState)
	{
		case RX_STATE_WAITING_FOR_HEADER:
		{
			if (RADARIQ_PACKET_HEAD == rxByte)
			{
				obj->rxBuffer.data[0] = rxByte;
				obj->rxBuffer.len = 1u;

				obj->rxState = RX_STATE_WAITING_FOR_FOOTER;
			}

			break;
		}
		case RX_STATE_WAITING_FOR_FOOTER:
		{
			obj->rxBuffer.data[obj->rxBuffer.len] = rxByte;
			obj->rxBuffer.len = (obj->rxBuffer.len + 1) % RADARIQ_RX_BUFFER_SIZE;

			if (RADARIQ_PACKET_FOOT == rxByte)
			{
				RadarIQReturnVal_t ret = RadarIQ_decodePacket(obj);

				if (RADARIQ_RETURN_VAL_OK == ret)
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

void RadarIQ_start(const RadarIQHandle_t obj, const uint8_t numFrames)
{
	radariq_assert(NULL != obj);

	obj->txPacket.data[0] = RADARIQ_CMD_CAPTURE_START;
	obj->txPacket.data[1] = RADARIQ_CMD_VAR_REQUEST;
	obj->txPacket.data[2] = numFrames;
	obj->txPacket.len = 3u;

	RadarIQ_sendPacket(obj);
}

uint8_t RadarIQ_getDataBuffer(const RadarIQHandle_t obj, uint8_t* dest)
{
	radariq_assert(NULL != obj);

	memcpy(dest, obj->rxPacket.data, obj->rxPacket.len);

	return obj->rxPacket.len;
}

//===============================================================================================//
// FILE-SCOPE FUNCTIONS - Packet Parsing
//===============================================================================================//

static RadarIQCommand_t RadarIQ_parsePacket(const RadarIQHandle_t obj)
{
	radariq_assert(NULL != obj);

	RadarIQCommand_t ret = RADARIQ_CMD_NONE;															//TODO: change how return values will work
	const RadarIQCommand_t command = (RadarIQCommand_t)obj->rxPacket.data[0];

	switch (command)
	{
	case RADARIQ_CMD_PNT_CLOUD_FRAME:
	{
		obj->logCallback("parsePacket: Pointcloud");

		RadarIQ_parsePointCloud(obj);
		ret = RADARIQ_CMD_PNT_CLOUD_FRAME;

		break;
	}
	case RADARIQ_CMD_OBJ

	default:
	{
		obj->logCallback("parsePacket: Unknown command");
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

RadarIQReturnVal_t RadarIQ_decodePacket(const RadarIQHandle_t obj)
{
	radariq_assert(NULL != obj);

	RadarIQReturnVal_t retval = RADARIQ_RETURN_VAL_OK;
	obj->rxPacket.len = 0u;
	uint8_t srcIdx = 0u;

	// Loop through all bytes except footer
	while(srcIdx < (obj->rxBuffer.len - 1u)) //
	{
		if(obj->rxPacket.len >= (RADARIQ_RX_BUFFER_SIZE - 1u))
		{
			retval = RADARIQ_RETURN_VAL_ERR;
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

	if(crc != rxCrc)
	{
		retval = RADARIQ_RETURN_VAL_ERR;
	}

	return retval;
}

static void encodeHelper(const RadarIQHandle_t obj, uint8_t const byte)
{
	radariq_assert(obj->txBuffer.len < (RADARIQ_TX_BUFFER_SIZE - 2u));

	switch (byte)
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
			obj->txBuffer.data[obj->txBuffer.len] = byte;
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
    dest |= ((data[3] << 24u) & 0xFF000000u);
    dest |= ((data[2] << 16u) & 0x00FF0000u);
    dest |= ((data[1] << 8u) & 0x0000FF00u);
    dest |= ((data[0]) & 0x000000FFu);

    return dest;
}

static uint16_t bytePack16(const uint8_t * const data)
{
    uint16_t dest = 0u;
    dest |= ((data[1] << 8u) & 0xFF00u);
    dest |= ((data[0]) & 0x00FFu);

    return dest;
}

static int16_t bytePack16Signed(const uint8_t * const data)
{
    uint16_t temp = 0u;
    int16_t returnValue = 0;

    temp |= ((data[1] << 8u) & 0xFF00u);
    temp |= ((data[0]) & 0x00FFu);

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

    temp |= ((data[3] << 24u) & 0xFF000000u);
    temp |= ((data[2] << 16u) & 0x00FF0000u);
    temp |= ((data[1] << 8u) & 0x0000FF00u);
    temp |= ((data[0]) & 0x000000FFu);

    memcpy(&returnValue, &temp, sizeof(uint16_t));

    return returnValue;
}
