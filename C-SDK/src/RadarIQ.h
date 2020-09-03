/**
 * @file
 * RadarIQ C module
 *
 */

#ifndef SRC_RADARIQ_H_
#define SRC_RADARIQ_H_

//===============================================================================================//
// GENERAL-PURPOSE MACROS
//===============================================================================================//

#define RADARIQ_RX_BUFFER_SIZE 		256u
#define RADARIQ_MAX_POINTCLOUD		64u
#define RADARIQ_MAX_OBJECTS			16u

//===============================================================================================//
// INCLUDES
//===============================================================================================//

#include <stdlib.h>
#include <stdint.h>

//===============================================================================================//
// DATA TYPES
//===============================================================================================//

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

} radariqStatistics_t;

//===============================================================================================//
// OBJECTS
//===============================================================================================//

typedef struct radariq_t radariq_t;
typedef radariq_t* radariqHandle_t;

//===============================================================================================//
// FUNCTIONS
//===============================================================================================//

radariqHandle_t RadarIQ_init(void(*sendSerialDataCallback)(uint8_t * const, const uint16_t), void(*logCallback)(char * const));
radariqData_t RadarIQ_getData(const radariqHandle_t obj);
uint32_t RadarIQ_getMemoryUsage(const radariqHandle_t obj);

#endif /* SRC_RADARIQ_H_ */
