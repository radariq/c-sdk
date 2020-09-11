/**
 * @file
 * RadarIQ C module
 *
 */

#ifndef SRC_RADARIQ_H_
#define SRC_RADARIQ_H_

#ifdef __cplusplus
extern "C" {
#endif

//===============================================================================================//
// INCLUDES
//===============================================================================================//

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <stdio.h>
#include <string.h>

//===============================================================================================//
// GENERAL-PURPOSE MACROS
//===============================================================================================//

#define RADARIQ_TX_BUFFER_SIZE			32u
#define RADARIQ_RX_BUFFER_SIZE 			256u
#define RADARIQ_MAX_POINTCLOUD			64u
#define RADARIQ_MAX_OBJECTS				16u

#define radariq_assert(expr) assert(expr)

//===============================================================================================//
// DATA TYPES
//===============================================================================================//

typedef enum
{
    RADARIQ_CMD_ERROR			 = -2,
    RADARIQ_CMD_NONE			 = -1,											//TODO: rename type to "packet" or something more fitting?
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
} RadarIQCommand_t;

typedef enum
{
	RADARIQ_RETURN_VAL_OK = 0,
	RADARIQ_RETURN_VAL_WARNING = 1,
	RADARIQ_RETURN_VAL_ERR = 2
} RadarIQReturnVal_t;

typedef enum
{
	RADARIQ_MODE_POINT_CLOUD = 0,
	RADARIQ_MODE_OBJECT_TRACKING = 1
}RadarIQCaptureMode_t;

typedef enum
{
	RADARIQ_MOVING_BOTH = 0,
	RADARIQ_MOVING_OBJECTS_ONLY = 1
}RadarIQMovingObjectMode_t;

typedef enum
{
	RADARIQ_RESET_REBOOT = 0,
	RADARIQ_FACTORY_SETTINGS = 1
}RadarIQResetCode_t;

typedef enum
{
	RADARIQ_DENSITY_NORMAL = 0,
	RADARIQ_DENSITY_DENSE = 1,
	RADARIQ_DENSITY_VERY_DENSE = 2
}RadarIQPointDensity_t;

typedef struct
{
	int16_t x;
	int16_t y;
	int16_t z;
	uint8_t intensity;
} RadarIQDataPoint_t;

typedef struct
{
	bool isFrameComplete;
	uint16_t numPoints;
	RadarIQDataPoint_t points[RADARIQ_MAX_POINTCLOUD];
} RadarIQDataPointCloud_t;

typedef struct
{
	uint8_t targetId;
	int16_t xPos;
	int16_t yPos;
	int16_t zPos;
	int16_t xVel;
	int16_t yVel;
	int16_t zVel;
	int16_t xAcc;
	int16_t yAcc;
	int16_t zAcc;
} RadarIQDataObject_t;

typedef struct
{
	uint8_t numObjects;
	RadarIQDataObject_t objects[RADARIQ_MAX_OBJECTS];
} RadarIQDataObjectTracking_t;

typedef union
{
	RadarIQDataPointCloud_t pointCloud;
	RadarIQDataObjectTracking_t objectTracking;
} RadarIQData_t;

typedef struct
{
	int16_t sensor0;
	int16_t sensor1;
	int16_t powerManagement;
	int16_t rx0;
	int16_t rx1;
	int16_t rx2;
	int16_t rx3;
	int16_t tx0;
	int16_t tx1;
	int16_t tx2;
} RadarIQChipTemperatures_t;

typedef struct
{
	uint32_t activeFrameCPULoad;
	uint32_t interFrameCPULoad;
	uint32_t interFrameProcTime;
	uint32_t transmitOutputTime;
	uint32_t interFrameProcMargin;
	uint32_t interChirpProcMargin;
} RadarIQTIStats_t;

typedef struct
{
	uint32_t frameAggregatingTime;
	uint32_t intensitySortTime;
	uint32_t nearestNeighboursTime;
	uint32_t uartTransmitTime;
	uint32_t numFilteredPoints;
	uint32_t numPointsTransmitted;
	bool inputPointsTruncated;
	bool outputPointsTruncated;
} RadarIQPointcloudStats_t;

typedef struct
{
	RadarIQTIStats_t processing;
	RadarIQPointcloudStats_t pointcloud;
	RadarIQChipTemperatures_t temperature;
} RadarIQStatistics_t;


//===============================================================================================//
// OBJECTS
//===============================================================================================//

typedef struct RadarIQ_t RadarIQ_t;
typedef RadarIQ_t* RadarIQHandle_t;

//===============================================================================================//
// FUNCTIONS
//===============================================================================================//

RadarIQHandle_t RadarIQ_init(void(*sendSerialDataCallback)(uint8_t * const, const uint16_t),
		uint8_t(*readSerialDataCallback)(void),
		void(*logCallback)(char * const));
RadarIQCommand_t RadarIQ_readSerial(const RadarIQHandle_t obj);
RadarIQReturnVal_t RadarIQ_getData(const RadarIQHandle_t obj, RadarIQData_t * const dest);
uint32_t RadarIQ_getMemoryUsage(void);

RadarIQReturnVal_t RadarIQ_getVersion(const RadarIQHandle_t obj);
RadarIQReturnVal_t RadarIQ_getRadarVersions(const RadarIQHandle_t obj);
RadarIQReturnVal_t RadarIQ_getSerialNumber(const RadarIQHandle_t obj);
void RadarIQ_start(const RadarIQHandle_t obj, const uint8_t numFrames);
uint8_t RadarIQ_getDataBuffer(const RadarIQHandle_t obj, uint8_t* dest);

#ifdef __cplusplus
}
#endif

#endif /* SRC_RADARIQ_H_ */
