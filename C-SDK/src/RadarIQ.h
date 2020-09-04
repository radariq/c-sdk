/**
 * @file
 * RadarIQ C module
 *
 */

#ifndef SRC_RADARIQ_H_
#define SRC_RADARIQ_H_

//===============================================================================================//
// INCLUDES
//===============================================================================================//

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <stdio.h>

//===============================================================================================//
// GENERAL-PURPOSE MACROS
//===============================================================================================//

#define RADARIQ_RX_BUFFER_SIZE 		256u
#define RADARIQ_MAX_POINTCLOUD		64u
#define RADARIQ_MAX_OBJECTS			16u

#define radariq_assert(expr) assert(expr)

//===============================================================================================//
// DATA TYPES
//===============================================================================================//

typedef enum
{
	RADARIQ_RETURN_VAL_OK = 0,
	RADARIQ_RETURN_VAL_WARNING = 1,
	RADARIQ_RETURN_VAL_ERR = 2
} radariqReturnVal_t;

typedef enum
{
	RADARIQ_MODE_POINT_CLOUD = 0,
	RADARIQ_MODE_OBJECT_TRACKING = 1
}radariqCaptureMode_t;

typedef enum
{
	RADARIQ_MOVING_BOTH = 0,
	RADARIQ_MOVING_OBJECTS_ONLY = 1
}radariqMovingObjectMode_t;

typedef enum
{
	RADARIQ_RESET_REBOOT = 0,
	RADARIQ_FACTORY_SETTINGS = 1
}radariqResetCode_t;

typedef enum
{
	RADARIQ_DENSITY_NORMAL = 0,
	RADARIQ_DENSITY_DENSE = 1,
	RADARIQ_DENSITY_VERY_DENSE = 2
}radariqPointDensity_t;

typedef struct
{
	int16_t x;
	int16_t y;
	int16_t z;
	uint8_t intensity;
} radariqDataPoint_t;

typedef struct
{
	uint16_t numPoints;
	radariqDataPoint_t points[RADARIQ_MAX_POINTCLOUD];
} radariqDataPointCloud_t;

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
} radariqDataObject_t;

typedef struct
{
	uint8_t numObjects;
	radariqDataObject_t objects[RADARIQ_MAX_OBJECTS];
} radariqDataObjectTracking_t;

typedef union
{
	radariqDataPointCloud_t pointCloud;
	radariqDataObjectTracking_t objectTracking;
} radariqData_t;

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
} radariqChipTemperatures_t;

typedef struct
{
	uint32_t activeFrameCPULoad;
	uint32_t interFrameCPULoad;
	uint32_t interFrameProcTime;
	uint32_t transmitOutputTime;
	uint32_t interFrameProcMargin;
	uint32_t interChirpProcMargin;
} radariqTIStats_t;

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
} radariqPointcloudStats_t;

typedef struct
{
	radariqTIStats_t processing;
	radariqPointcloudStats_t pointcloud;
	radariqChipTemperatures_t temperature;
} radariqStatistics_t;


//===============================================================================================//
// OBJECTS
//===============================================================================================//

typedef struct radariq_t radariq_t;
typedef radariq_t* radariqHandle_t;

//===============================================================================================//
// FUNCTIONS
//===============================================================================================//

radariqHandle_t RadarIQ_init(void(*sendSerialDataCallback)(uint8_t * const, const uint16_t),
		uint8_t(*readSerialDataCallback)(void),
		void(*logCallback)(char * const));
bool RadarIQ_readSerial(const radariqHandle_t obj);
radariqReturnVal_t RadarIQ_getData(const radariqHandle_t obj, radariqData_t * const dest);
uint32_t RadarIQ_getMemoryUsage();

#endif /* SRC_RADARIQ_H_ */
