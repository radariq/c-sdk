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
#include <time.h>

//===============================================================================================//
// DEFINITIONS
//===============================================================================================//

// UART buffer sizes
#define RADARIQ_TX_BUFFER_SIZE			32u
#define RADARIQ_RX_BUFFER_SIZE 			256u

// Data storage sizes
#define RADARIQ_MAX_POINTCLOUD			5u
#define RADARIQ_MAX_OBJECTS				16u

// Limits
#define RADARIQ_MAX_MESSAGE_STRING		200u
#define RADARIQ_MIN_FRAME_RATE			1u
#define RADARIQ_MAX_FRAME_RATE			30u
#define RADARIQ_MIN_DIST_FILT			0u
#define RADARIQ_MAX_DIST_FILT			10000u
#define RADARIQ_MIN_ANGLE_FILT			-55
#define RADARIQ_MAX_ANGLE_FILT			55
#define RADARIQ_MAX_CERTAINTY			9u
#define RADARIQ_MAX_OBJ_SIZE			4u

// Macros
#define radariq_assert(expr)			assert(expr)

#include <Arduino.h>
#define radariq_get_mseconds millis()

//===============================================================================================//
// DATA TYPES
//===============================================================================================//

typedef enum
{
  RADARIQ_CMD_NONE			 	= -99,			//TODO: rename type to "packet" or something more fitting?
  RADARIQ_CMD_ERROR			 	= -2,
  RADARIQ_CMD_UNKNOWN				= -1,
  RADARIQ_CMD_MESSAGE          	= 0x00,
  RADARIQ_CMD_VERSION          	= 0x01,
  RADARIQ_CMD_SERIAL           	= 0x02,
  RADARIQ_CMD_RESET            	= 0x03,
  RADARIQ_CMD_FRAME_RATE       	= 0x04,
  RADARIQ_CMD_MODE             	= 0x05,
  RADARIQ_CMD_DIST_FILT        	= 0x06,
  RADARIQ_CMD_ANGLE_FILT       	= 0x07,
  RADARIQ_CMD_MOVING_FILT      	= 0x08,
  RADARIQ_CMD_SAVE             	= 0x09,
  RADARIQ_CMD_PNT_DENSITY      	= 0x10,
  RADARIQ_CMD_CERTAINTY        	= 0x11,
  RADARIQ_CMD_HEIGHT_FILT      	= 0x12,
  RADARIQ_CMD_IWR_VERSION      	= 0x14,
  RADARIQ_CMD_SCENE_CALIB			= 0x15,		
  RADARIQ_CMD_OBJECT_SIZE      	= 0x16,		
  RADARIQ_CMD_CAPTURE_START    	= 0x64,
  RADARIQ_CMD_CAPTURE_STOP     	= 0x65,
  RADARIQ_CMD_PNT_CLOUD_FRAME  	= 0x66,
  RADARIQ_CMD_OBJ_TRACKING_FRAME	= 0x67,
  RADARIQ_CMD_PROC_STATS			= 0x68,
  RADARIQ_CMD_RAW_DATA			= 0x69,
  RADARIQ_CMD_POINTCLOUD_STATS    = 0x70,
  RADARIQ_CMD_POWER_STATUS     	= 0x71
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
	RADARIQ_MODE_OBJECT_TRACKING = 1,
	RADARIQ_MODE_RAW_DATA = 2,
}RadarIQCaptureMode_t;

typedef enum
{
	RADARIQ_MOVING_BOTH = 0,
	RADARIQ_MOVING_OBJECTS_ONLY = 1
}RadarIQMovingFilterMode_t;

typedef enum
{
	RADARIQ_RESET_REBOOT = 0,
	RADARIQ_RESET_FACTORY_SETTINGS = 1
}RadarIQResetCode_t;

typedef enum
{
	RADARIQ_DENSITY_NORMAL = 0,
	RADARIQ_DENSITY_DENSE = 1,
	RADARIQ_DENSITY_VERY_DENSE = 2
}RadarIQPointDensity_t;

typedef struct __attribute__((__packed__))
{
	int16_t x;
	int16_t y;
	int16_t z;
	uint8_t intensity;
	int16_t velocity;
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
	bool isFrameComplete;
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
	uint32_t uartTransmitTime;
} RadarIQProcessingStats_t;

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
	RadarIQProcessingStats_t processing;
	RadarIQPointcloudStats_t pointcloud;
	RadarIQChipTemperatures_t temperature;
} RadarIQStatistics_t;

typedef struct
{
	uint8_t major;
	uint8_t minor;
	uint16_t build;
} RadarIQVersion_t;

typedef struct
{
	uint8_t major;
	uint8_t minor;
	uint16_t build;
	char name[20];
} RadarIQVersionIWR_t;

typedef struct
{
	uint32_t a;
	uint32_t b;
} RadarIQSerialNo_t;

typedef struct
{
	uint8_t data;
	bool isReadable;
} RadarIQUartData_t;


//===============================================================================================//
// OBJECTS
//===============================================================================================//

typedef struct RadarIQ_t RadarIQ_t;
typedef RadarIQ_t* RadarIQHandle_t;

//===============================================================================================//
// FUNCTIONS
//===============================================================================================//

RadarIQHandle_t RadarIQ_init(void(*sendSerialDataCallback)(uint8_t * const, const uint16_t),
		RadarIQUartData_t(*readSerialDataCallback)(void),
		void(*logCallback)(char * const));
RadarIQCommand_t RadarIQ_readSerial(const RadarIQHandle_t obj);

// Debug & info
uint32_t RadarIQ_getMemoryUsage(void);
uint8_t RadarIQ_getDataBuffer(const RadarIQHandle_t obj, uint8_t* dest);

// Data & stats getters
void RadarIQ_getData(const RadarIQHandle_t obj, RadarIQData_t * dest);
void RadarIQ_getProcessingStats(const RadarIQHandle_t obj, RadarIQProcessingStats_t * const dest);
void RadarIQ_getPointCloudStats(const RadarIQHandle_t obj, RadarIQPointcloudStats_t * const dest);
void RadarIQ_getChipTemperatures(const RadarIQHandle_t obj, RadarIQChipTemperatures_t * const dest);
bool RadarIQ_isPowerGood(const RadarIQHandle_t obj);

// UART commands
void RadarIQ_start(const RadarIQHandle_t obj, const uint8_t numFrames);
RadarIQReturnVal_t RadarIQ_reset(const RadarIQHandle_t obj, const RadarIQResetCode_t code);
RadarIQReturnVal_t RadarIQ_save(const RadarIQHandle_t obj);
RadarIQReturnVal_t RadarIQ_getVersion(const RadarIQHandle_t obj, RadarIQVersion_t * const firmware, RadarIQVersion_t * const hardware);
RadarIQReturnVal_t RadarIQ_getRadarVersions(const RadarIQHandle_t obj, RadarIQVersionIWR_t * const sbl, RadarIQVersionIWR_t * const app1, RadarIQVersionIWR_t * const app2);
RadarIQReturnVal_t RadarIQ_getSerialNumber(const RadarIQHandle_t obj, RadarIQSerialNo_t * const serial);
RadarIQReturnVal_t RadarIQ_getFrameRate(const RadarIQHandle_t obj, uint8_t * const rate);
RadarIQReturnVal_t RadarIQ_setFrameRate(const RadarIQHandle_t obj, uint8_t rate);
RadarIQReturnVal_t RadarIQ_getMode(const RadarIQHandle_t obj, RadarIQCaptureMode_t * const mode);
RadarIQReturnVal_t RadarIQ_setMode(const RadarIQHandle_t obj, RadarIQCaptureMode_t mode);
RadarIQReturnVal_t RadarIQ_getDistanceFilter(const RadarIQHandle_t obj, uint16_t * const min, uint16_t * const max);
RadarIQReturnVal_t RadarIQ_setDistanceFilter(const RadarIQHandle_t obj, uint16_t min, uint16_t max);
RadarIQReturnVal_t RadarIQ_getAngleFilter(const RadarIQHandle_t obj, int8_t * const min, int8_t * const max);
RadarIQReturnVal_t RadarIQ_setAngleFilter(const RadarIQHandle_t obj, int8_t min, int8_t max);
RadarIQReturnVal_t RadarIQ_getMovingFilter(const RadarIQHandle_t obj, RadarIQMovingFilterMode_t * const filter);
RadarIQReturnVal_t RadarIQ_setMovingFilter(const RadarIQHandle_t obj, RadarIQMovingFilterMode_t filter);
RadarIQReturnVal_t RadarIQ_getPointDensity(const RadarIQHandle_t obj, RadarIQPointDensity_t * const density);
RadarIQReturnVal_t RadarIQ_setPointDensity(const RadarIQHandle_t obj, RadarIQPointDensity_t density);
RadarIQReturnVal_t RadarIQ_getCertainty(const RadarIQHandle_t obj, uint8_t * const certainty);
RadarIQReturnVal_t RadarIQ_setCertainty(const RadarIQHandle_t obj, uint8_t certainty);
RadarIQReturnVal_t RadarIQ_getHeightFilter(const RadarIQHandle_t obj, int16_t * const min, int16_t * const max);
RadarIQReturnVal_t RadarIQ_setHeightFilter(const RadarIQHandle_t obj, int16_t min, int16_t max);
RadarIQReturnVal_t RadarIQ_sceneCalibrate(const RadarIQHandle_t obj);
RadarIQReturnVal_t RadarIQ_getObjectSize(const RadarIQHandle_t obj, uint8_t * const size);
RadarIQReturnVal_t RadarIQ_setObjectSize(const RadarIQHandle_t obj, uint8_t size);

#ifdef __cplusplus
}
#endif

#endif /* SRC_RADARIQ_H_ */
