//-------------------------------------------------------------------------------------------------
//                                                                            				     --
//                             			RadarIQ C-SDK                                  			 --
//                                                                            				     --
//                   		(C) 2021 RadarIQ <support@radariq.io>                    			 --
//                                                                            					 --
//                            			License: MIT                                    	     --
//                                                                            					 --
//------------------------------------------------------------------------------------------------- 

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

/* UART buffer sizes */
#define RADARIQ_TX_BUFFER_SIZE			32u		///< Tx buffer size in bytes
#define RADARIQ_RX_BUFFER_SIZE 			256u	///< Rx buffer size in bytes

/* Frame data storage sizes */
#define RADARIQ_MAX_POINTCLOUD			64u		///< Maximum number of point-cloud points to store in one frame
#define RADARIQ_MAX_OBJECTS				16u		///< Maximum number of detected objects to store in one frame

/* Limits */
#define RADARIQ_MAX_MESSAGE_STRING		200u	///< Maximum string length of the message in message packets
#define RADARIQ_MIN_FRAME_RATE			1u		///< Minimum capture frame rate in frames/second
#define RADARIQ_MAX_FRAME_RATE			30u		///< Maximum capture frame rate in frames/second
#define RADARIQ_MIN_DIST_FILT			0u		///< Minimum bound length of distance in millimeters
#define RADARIQ_MAX_DIST_FILT			10000u	///< Maximum bound length of distance in millimeters
#define RADARIQ_MIN_ANGLE_FILT			-55		///< Minimum bound angle of angle filter in degrees
#define RADARIQ_MAX_ANGLE_FILT			55		///< Maximum bound angle of angle filter in degrees
#define RADARIQ_MAX_CERTAINTY			9u		///< Maximum certainty level in point-cloud mode
#define RADARIQ_MAX_OBJ_SIZE			4u		///< Maximum target object size in object-tracking mode

/**
 * Assertion macro - redefine if necessary or remove at your own risk
 */
#define radariq_assert(expr)			assert(expr)

/**
 * Assertion macro - redefine if necessary or remove at your own risk
 */	
#define radariq_get_seconds				(time(NULL) * 1000)

//===============================================================================================//
// DATA TYPES
//===============================================================================================//

/**
 * UART packet commands sent to and from the RadarIQ device
 */
typedef enum
{
    RADARIQ_CMD_NONE			 	= -99,		///< Value indicates no packet has been recieved
    RADARIQ_CMD_ERROR			 	= -2,		///< Value indicates an error occured during packet parsing
    RADARIQ_CMD_UNKNOWN				= -1,		///< Value indicates an unknown packet type was recieved
    RADARIQ_CMD_MESSAGE          	= 0x00,		///< Messages sent from device
    RADARIQ_CMD_VERSION          	= 0x01,		///< Firmware and hardware version of device
    RADARIQ_CMD_SERIAL           	= 0x02,		///< Serial number of device
    RADARIQ_CMD_RESET            	= 0x03,		///< Resets the device
    RADARIQ_CMD_FRAME_RATE       	= 0x04,		///< Sets the capture frame-rate of device
    RADARIQ_CMD_MODE             	= 0x05,		///< Sets the capture mode of the device
    RADARIQ_CMD_DIST_FILT        	= 0x06,		///< Sets the distance filter of the device in millimeters	
    RADARIQ_CMD_ANGLE_FILT       	= 0x07,		///< Sets the angle filter of the device in degrees	
    RADARIQ_CMD_MOVING_FILT      	= 0x08,		///< Sets the moving filter setting of the device	
    RADARIQ_CMD_SAVE             	= 0x09,		///< Saves the device settings to its EEPROM
    RADARIQ_CMD_PNT_DENSITY      	= 0x10,		///< Sets the point-cloud point density
    RADARIQ_CMD_CERTAINTY        	= 0x11,		///< Sets the point-cloud certainty level
    RADARIQ_CMD_HEIGHT_FILT      	= 0x12,		///< Sets the height filter of the device in millimeters	
    RADARIQ_CMD_IWR_VERSION      	= 0x14,		///< Returns the firmware versions running on the IWR6843
    RADARIQ_CMD_SCENE_CALIB			= 0x15,		///< Runs a scene calibration on the device
    RADARIQ_CMD_OBJECT_SIZE      	= 0x16,		///< Sets the target object size for object-tracking mode
    RADARIQ_CMD_CAPTURE_START    	= 0x64,		///< Starts a radar data capture on the device
    RADARIQ_CMD_CAPTURE_STOP     	= 0x65,		///< Stops a radar data capture
    RADARIQ_CMD_PNT_CLOUD_FRAME  	= 0x66,		///< Point-cloud frame data returned from device
	RADARIQ_CMD_OBJ_TRACKING_FRAME	= 0x67,		///< Object-tracking frame data returned from device
	RADARIQ_CMD_PROC_STATS			= 0x68,		///< Processing statistics returned from device
	RADARIQ_CMD_RAW_DATA			= 0x69,		///< Raw data frame data returned from device
	RADARIQ_CMD_POINTCLOUD_STATS    = 0x70,		///< Point-cloud statistics returned from device
	RADARIQ_CMD_POWER_STATUS     	= 0x71		///< Power ok flag returned from device
} RadarIQCommand_t;

/**
 * Return values for functions
 */
typedef enum
{
	RADARIQ_RETURN_VAL_OK = 0,
	RADARIQ_RETURN_VAL_WARNING = 1,
	RADARIQ_RETURN_VAL_ERR = 2
} RadarIQReturnVal_t;

/**
 * Radar data capture modes
 */
typedef enum
{
	RADARIQ_MODE_POINT_CLOUD = 0,
	RADARIQ_MODE_OBJECT_TRACKING = 1,
	RADARIQ_MODE_RAW_DATA = 2,
}RadarIQCaptureMode_t;

/**
 * Moving filter modes
 */
typedef enum
{
	RADARIQ_MOVING_BOTH = 0,
	RADARIQ_MOVING_OBJECTS_ONLY = 1
}RadarIQMovingFilterMode_t;

/**
 * Reset command codes
 */
typedef enum
{
	RADARIQ_RESET_REBOOT = 0,
	RADARIQ_RESET_FACTORY_SETTINGS = 1
}RadarIQResetCode_t;

/**
 * Point-cloud point density settings
 */
typedef enum
{
	RADARIQ_DENSITY_NORMAL = 0,
	RADARIQ_DENSITY_DENSE = 1,
	RADARIQ_DENSITY_VERY_DENSE = 2
}RadarIQPointDensity_t;

/**
 * Point-cloud data point
 */
typedef struct
{
	int16_t x;
	int16_t y;
	int16_t z;
	uint8_t intensity;
	int16_t velocity;
} RadarIQDataPoint_t;

/**
 * Point-cloud data frame
 */
typedef struct
{
	bool isFrameComplete;
	uint16_t numPoints;
	RadarIQDataPoint_t points[RADARIQ_MAX_POINTCLOUD];
} RadarIQDataPointCloud_t;

/**
 * Object-tracking object data
 */
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

/**
 * Object-tracking frame data
 */
typedef struct
{
	bool isFrameComplete;
	uint8_t numObjects;
	RadarIQDataObject_t objects[RADARIQ_MAX_OBJECTS];
} RadarIQDataObjectTracking_t;

/**
 * Frame data - only one type can be accessed at one time
 */
typedef union
{
	RadarIQDataPointCloud_t pointCloud;
	RadarIQDataObjectTracking_t objectTracking;
} RadarIQData_t;

/**
 * Radar chip temperature measurements sent from RadarIQ device
 */
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

/**
 * Radar data processing statistics sent from RadarIQ device
 */
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

/**
 * Radar point-cloud processing statistics sent from RadarIQ device
 */
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
void RadarIQ_getData(const RadarIQHandle_t obj, RadarIQData_t * const dest);
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
