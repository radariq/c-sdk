/**
 * @file
 * RadarIQ SDK.
 * Software Development Kit to integrate the RadarIQ M1 sensor with C and C++ applications
 *
 * @copyright Copyright (C) 2021 RadarIQ
 *            Licensed under the MIT license
 *
 * @author RadarIQ Ltd
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
// DEFINITIONS
//===============================================================================================//

/* UART buffer sizes */
#define RADARIQ_TX_BUFFER_SIZE             32u       ///< Tx buffer size in bytes
#define RADARIQ_RX_BUFFER_SIZE             256u      ///< Rx buffer size in bytes

/* Frame data storage sizes */
#define RADARIQ_MAX_POINTCLOUD             64u       ///< Maximum number of point-cloud points to store in one frame
#define RADARIQ_MAX_OBJECTS                16u       ///< Maximum number of detected objects to store in one frame

/* Limits */
#define RADARIQ_MAX_MESSAGE_STRING         200u      ///< Maximum string length of the message in message packets
#define RADARIQ_MIN_FRAME_RATE             1u        ///< Minimum capture frame rate in frames/second
#define RADARIQ_MAX_FRAME_RATE             30u       ///< Maximum capture frame rate in frames/second
#define RADARIQ_MIN_DIST_FILT              0u        ///< Minimum bound length of distance in millimeters
#define RADARIQ_MAX_DIST_FILT              10000u    ///< Maximum bound length of distance in millimeters
#define RADARIQ_MIN_ANGLE_FILT             -55       ///< Minimum bound angle of angle filter in degrees
#define RADARIQ_MAX_ANGLE_FILT             55        ///< Maximum bound angle of angle filter in degrees
#define RADARIQ_MAX_SENSITIVITY            9u        ///< Maximum sensitivity level in point-cloud mode
#define RADARIQ_MAX_OBJ_SIZE               4u        ///< Maximum target object size in object-tracking mode

/* Debug */
#define RADARIQ_DEBUG_ENABLE               0         ///< Enables any debug messages printed from RadarIQ.c if set to 1  

/**
 * Assertion macro - redefine if necessary or remove at your own risk
 */
#define RADARIQ_ASSERT(expr)    assert(expr)


//===============================================================================================//
// DATA TYPES
//===============================================================================================//

/**
 * UART packet commands sent to and from the RadarIQ device
 */
typedef enum
{
    RADARIQ_CMD_NONE                 = -99,         ///< Value indicates no packet has been recieved
    RADARIQ_CMD_ERROR                = -2,          ///< Value indicates an error occured during packet parsing
    RADARIQ_CMD_UNKNOWN              = -1,          ///< Value indicates an unknown packet type was recieved
    RADARIQ_CMD_MESSAGE              = 0x00,        ///< Messages sent from device
    RADARIQ_CMD_VERSION              = 0x01,        ///< Firmware and hardware version of device
    RADARIQ_CMD_SERIAL               = 0x02,        ///< Serial number of device
    RADARIQ_CMD_RESET                = 0x03,        ///< Resets the device
    RADARIQ_CMD_FRAME_RATE           = 0x04,        ///< Sets the capture frame-rate of device
    RADARIQ_CMD_MODE                 = 0x05,        ///< Sets the capture mode of the device
    RADARIQ_CMD_DIST_FILT            = 0x06,        ///< Sets the distance filter of the device in millimeters    
    RADARIQ_CMD_ANGLE_FILT           = 0x07,        ///< Sets the angle filter of the device in degrees    
    RADARIQ_CMD_MOVING_FILT          = 0x08,        ///< Sets the moving filter setting of the device    
    RADARIQ_CMD_SAVE                 = 0x09,        ///< Saves the device settings to its EEPROM
    RADARIQ_CMD_PNT_DENSITY          = 0x10,        ///< Sets the point-cloud point density
    RADARIQ_CMD_SENSITIVITY          = 0x11,        ///< Sets the point-cloud sensitivity level
    RADARIQ_CMD_HEIGHT_FILT          = 0x12,        ///< Sets the height filter of the device in millimeters    
    RADARIQ_CMD_IWR_VERSION          = 0x14,        ///< Returns the firmware versions running on the IWR
    RADARIQ_CMD_SCENE_CALIB          = 0x15,        ///< Runs a scene calibration on the device
    RADARIQ_CMD_OBJECT_SIZE          = 0x16,        ///< Sets the target object size for object-tracking mode
    RADARIQ_CMD_CAPTURE_START        = 0x64,        ///< Starts a radar data capture on the device
    RADARIQ_CMD_CAPTURE_STOP         = 0x65,        ///< Stops a radar data capture
    RADARIQ_CMD_PNT_CLOUD_FRAME      = 0x66,        ///< Point-cloud frame data returned from device
    RADARIQ_CMD_OBJ_TRACKING_FRAME   = 0x67,        ///< Object-tracking frame data returned from device
    RADARIQ_CMD_PROC_STATS           = 0x68,        ///< Processing statistics returned from device
    RADARIQ_CMD_RAW_DATA             = 0x69,        ///< Raw data frame data returned from device
    RADARIQ_CMD_POINTCLOUD_STATS     = 0x70,        ///< Point-cloud statistics returned from device
    RADARIQ_CMD_POWER_STATUS         = 0x71         ///< Power ok flag returned from device
} RadarIQCommand_t;

/**
 * UART packet command variants
 */
typedef enum
{
    RADARIQ_CMD_VAR_REQUEST     = 0,    ///< Requests a response from the device
    RADARIQ_CMD_VAR_RESPONSE    = 1,    ///< A response sent from the device
    RADARIQ_CMD_VAR_SET         = 2     ///< Sets a parameter of the device
} RadarIQCommandVariant_t;

/**
 * Return values for functions
 */
typedef enum
{
    RADARIQ_RETURN_VAL_OK = 0,             ///< Function excecuted ok
    RADARIQ_RETURN_VAL_WARNING = 1,        ///< Value(s) passed to function were out of range and limited
    RADARIQ_RETURN_VAL_ERR = 2             ///< Function returned an error
} RadarIQReturnVal_t;

/**
 * Radar data capture modes
 */
typedef enum
{
    RADARIQ_MODE_POINT_CLOUD = 0,         ///< Point-cloud capture mode
    RADARIQ_MODE_OBJECT_TRACKING = 1,     ///< Object-tracking capture mode
}RadarIQCaptureMode_t;

/**
 * Moving filter modes
 */
typedef enum
{
    RADARIQ_MOVING_BOTH = 0,               ///< Filter will keep static and moving objects
    RADARIQ_MOVING_OBJECTS_ONLY = 1        ///< Filter will remove static objects and keep moving objects
}RadarIQMovingFilterMode_t;

/**
 * Reset command codes
 */
typedef enum
{
    RADARIQ_RESET_REBOOT = 0,             ///< Command will reboot the device
    RADARIQ_RESET_FACTORY_SETTINGS = 1    ///< Command will restore the factory default settings
}RadarIQResetCode_t;

/**
 * Point-cloud point density settings
 */
typedef enum
{
    RADARIQ_DENSITY_NORMAL = 0,            ///< Single frame will be aggregated into final point-cloud frame
    RADARIQ_DENSITY_DENSE = 1,             ///< Multiple frames will be aggregated into final point-cloud frame
    RADARIQ_DENSITY_VERY_DENSE = 2         ///< More multiple frames will be aggregated into final point-cloud frame
}RadarIQPointDensity_t;

/**
 * Point-cloud data point
 */
typedef struct
{
    int16_t x;                        ///< The point's x-coordinate in millimeters
    int16_t y;                        ///< The point's y-coordinate in millimeters
    int16_t z;                        ///< The point's z-coordinate in millimeters
    uint8_t intensity;                ///< The point's intensity in the range 0-255
    int16_t velocity;                 ///< The point's velocity magnitude in millimeters/second
} RadarIQDataPoint_t;

/**
 * Point-cloud data frame
 */
typedef struct
{
    bool isFrameComplete;            ///< Indicates whether a frame is complete or has points truncated to fit max storage    
    uint16_t numPoints;              ///< Indicates number of points in the frame
    RadarIQDataPoint_t points[RADARIQ_MAX_POINTCLOUD];    ///< Array to store the points data
} RadarIQDataPointCloud_t;

/**
 * Object-tracking object data
 */
typedef struct
{
    uint8_t targetId;                ///< The target object's ID (<250) or point not associated (>250)
    int16_t xPos;                    ///< The target object's x-coordinate in millimeters
    int16_t yPos;                    ///< The target object's y-coordinate in millimeters
    int16_t zPos;                    ///< The target object's z-coordinate in millimeters
    int16_t xVel;                    ///< The target object's x-velocity in millimeters/second                
    int16_t yVel;                    ///< The target object's y-velocity in millimeters/second
    int16_t zVel;                    ///< The target object's z-velocity in millimeters/second
    int16_t xAcc;                    ///< The target object's x-acceleration in millimeters/second/second
    int16_t yAcc;                    ///< The target object's y-acceleration in millimeters/second/second
    int16_t zAcc;                    ///< The target object's z-acceleration in millimeters/second/second
} RadarIQDataObject_t;

/**
 * Object-tracking frame data
 */
typedef struct
{
    bool isFrameComplete;            ///< Indicates whether a frame is complete or has objects truncated to fit max storage    
    uint8_t numObjects;              ///< Indicates number of detected objects in the frame
    RadarIQDataObject_t objects[RADARIQ_MAX_OBJECTS];    ///< Array to store the detected objects data
} RadarIQDataObjectTracking_t;

/**
 * Types of messages sent from the device in a message packet
 */
typedef enum
{
    RADARIQ_MSG_TYPE_TEMPORARY     = 0,        ///< Temporary messages not intended to be used permanently
    RADARIQ_MSG_TYPE_DEBUG         = 1,        ///< Debug information about an operation
    RADARIQ_MSG_TYPE_INFO          = 2,        ///< General information about an operation
    RADARIQ_MSG_TYPE_WARNING       = 3,        ///< An operation was completed but not as expected
    RADARIQ_MSG_TYPE_ERROR         = 4,        ///< An operation was not completed due to an error
    RADARIQ_MSG_TYPE_SUCCESS       = 5         ///< An operation completed successfully
} RadarIQMsgType_t;

/**
 * Types of message codes sent from the device in a message packet
 */
typedef enum
{
    RADARIQ_MSG_CODE_GENERAL            = 0,    ///< General debug messages with no specific code
    RADARIQ_MSG_CODE_FRAMERATE_TOO_HIGH = 1,    ///< Requested frame rate was too high and limited
    RADARIQ_MSG_CODE_CALIB_FAILED       = 2,    ///< Device could not be calibrated
    RADARIQ_MSG_CODE_IWR_COMMS_TIMEOUT  = 3,    ///<
    RADARIQ_MSG_CODE_INVALID_COMMAND    = 100,  ///<
    RADARIQ_MSG_CODE_INALIVD_VALUE      = 101,  ///< 
    RADARIQ_MSG_CODE_PACKET_OVERFLOW    = 102,  ///<  
} RadarIQMsgCode_t;

typedef struct
{
    RadarIQMsgType_t type;
    uint8_t code;                               ///< Warning/error code or 0 for general debug messages     
    char message[RADARIQ_MAX_MESSAGE_STRING];   ///< Accesses the string from message packets
} RadarIQMsg_t;

/**
 * Data returned from device - only one type can be accessed at one time
 */
typedef union
{
    RadarIQDataPointCloud_t pointCloud;                ///< Accesses the point-cloud data struct
    RadarIQDataObjectTracking_t objectTracking;        ///< Accesses the object-tracking data struct

} RadarIQData_t;

/**
 * Radar chip temperature measurements sent from RadarIQ device
 */
typedef struct
{
    int16_t sensor0;                ///< Digital temperature sensor reading from the IWR in degrees C
    int16_t sensor1;                ///< Digital temperature sensor reading from the IWR in degrees C
    int16_t powerManagement;        ///< Power management temperature sensor reading from the IWR in degrees C
    int16_t rx0;                    ///< RX0 temperature sensor reading from the IWR in degrees C
    int16_t rx1;                    ///< RX1 temperature sensor reading from the IWR in degrees C
    int16_t rx2;                    ///< RX2 temperature sensor reading from the IWR in degrees C
    int16_t rx3;                    ///< RX3 temperature sensor reading from the IWR in degrees C
    int16_t tx0;                    ///< TX0 temperature sensor reading from the IWR in degrees C
    int16_t tx1;                    ///< TX1 temperature sensor reading from the IWR in degrees C
    int16_t tx2;                    ///< TX2 temperature sensor reading from the IWR in degrees C
} RadarIQChipTemperatures_t;

/**
 * Radar data processing statistics sent from RadarIQ device
 */
typedef struct
{
    uint32_t activeFrameCPULoad;    ///< CPU load (%) of the IWR during active frame
    uint32_t interFrameCPULoad;     ///< CPU load (%) of the IWR between frames
    uint32_t interFrameProcTime;    ///< Processing time of the IWR  between frames in microseconds
    uint32_t transmitOutputTime;    ///< Time taken for the IWR to process and transmit one frame in microseconds
    uint32_t interFrameProcMargin;  ///< Free processing time of the IWR between frames in microseconds
    uint32_t interChirpProcMargin;  ///< Free processing time of the IWR between chirps in microseconds
    uint32_t uartTransmitTime;      ///< Time taken for the IWR to send one single frame over UART in microseconds
} RadarIQProcessingStats_t;

/**
 * Radar point-cloud processing statistics sent from RadarIQ device
 */
typedef struct
{
    uint32_t frameAggregatingTime;    ///< Time spent aggregating points from frame(s) in microseconds    
    uint32_t intensitySortTime;       ///< Time spent ordering points by intensity in microseconds
    uint32_t nearestNeighboursTime;   ///< Time spent filtering points using nearest neighbors in microseconds
    uint32_t uartTransmitTime;        ///< Time taken for the IWR to send one single frame over UART in microseconds
    uint32_t numFilteredPoints;       ///< Number of points removed using nearest neighbors filter
    uint32_t numPointsTransmitted;    ///< Number of points transmitted frame over UART
    bool inputPointsTruncated;        ///< Number of points to aggregate were truncated to max frame size
    bool outputPointsTruncated;       ///< Number of points to transmit were truncated to max frame size
} RadarIQPointcloudStats_t;

/**
 * All radar statistics sent from RadarIQ device
 */
typedef struct
{
    RadarIQProcessingStats_t processing;       ///< Statistics relating to general data processing
    RadarIQPointcloudStats_t pointcloud;       ///< Statistics relating to the point cloud specific processing
    RadarIQChipTemperatures_t temperature;     ///< Statistics relating to the sensors temperature
} RadarIQStatistics_t;

/**
 * Firmware / hardware version number format
 */
typedef struct
{
    uint8_t major;                  ///< Major version
    uint8_t minor;                  ///< Minor version
    uint16_t build;                 ///< Build version
} RadarIQVersion_t;

/**
 * IWR application firmware version number format
 */
typedef struct
{
    uint8_t major;                  ///< Major version
    uint8_t minor;                  ///< Minor version
    uint16_t build;                 ///< Build version
    char name[20];                  ///< Application name
} RadarIQVersionIWR_t;

/**
 * RadarIQ device serial number format
 */
typedef struct
{
    uint32_t a;                 ///< First part of the sensor serial number
    uint32_t b;                 ///< Second part of the device serial number
} RadarIQSerialNo_t;

/**
 * UART data byte struct
 */
typedef struct
{
    uint8_t data;            ///< Stores the data byte received over UART
    bool isReadable;         ///< Indicates whether data is available for reading from UART
} RadarIQUartData_t;

//===============================================================================================//
// OBJECTS
//===============================================================================================//

typedef struct RadarIQ_t RadarIQ_t;
typedef RadarIQ_t* RadarIQHandle_t;

//===============================================================================================//
// FUNCTIONS
//===============================================================================================//

/* Object initialization */
RadarIQHandle_t RadarIQ_init(void(*sendSerialDataCallback)(uint8_t * const, const uint16_t),
        RadarIQUartData_t(*readSerialDataCallback)(void),
        void(*logCallback)(char * const),
        uint32_t(*millisCallback)(void));

/* UART read function */
RadarIQCommand_t RadarIQ_readSerial(const RadarIQHandle_t obj);

/* Debug & info */
uint32_t RadarIQ_getMemoryUsage(void);
uint16_t RadarIQ_getDataBuffer(const RadarIQHandle_t obj, uint8_t* dest);

/* Data & stats getters */
void RadarIQ_getData(const RadarIQHandle_t obj, RadarIQData_t * dest);
void RadarIQ_getProcessingStats(const RadarIQHandle_t obj, RadarIQProcessingStats_t * const dest);
void RadarIQ_getPointCloudStats(const RadarIQHandle_t obj, RadarIQPointcloudStats_t * const dest);
void RadarIQ_getChipTemperatures(const RadarIQHandle_t obj, RadarIQChipTemperatures_t * const dest);
bool RadarIQ_isPowerGood(const RadarIQHandle_t obj);

/* UART commands */
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
RadarIQReturnVal_t RadarIQ_getSensitivity(const RadarIQHandle_t obj, uint8_t * const sensitivity);
RadarIQReturnVal_t RadarIQ_setSensitivity(const RadarIQHandle_t obj, uint8_t sensitivity);
RadarIQReturnVal_t RadarIQ_getHeightFilter(const RadarIQHandle_t obj, int16_t * const min, int16_t * const max);
RadarIQReturnVal_t RadarIQ_setHeightFilter(const RadarIQHandle_t obj, int16_t min, int16_t max);
RadarIQReturnVal_t RadarIQ_sceneCalibrate(const RadarIQHandle_t obj);
RadarIQReturnVal_t RadarIQ_getObjectSize(const RadarIQHandle_t obj, uint8_t * const size);
RadarIQReturnVal_t RadarIQ_setObjectSize(const RadarIQHandle_t obj, uint8_t size);

#ifdef __cplusplus
}
#endif

#endif /* SRC_RADARIQ_H_ */