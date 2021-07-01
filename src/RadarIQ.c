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

//===============================================================================================//
// INCLUDES
//===============================================================================================//

#include "RadarIQ.h"

//===============================================================================================//
// DATA TYPES
//===============================================================================================//

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
 * UART packet receiving states
 */
typedef enum
{
    RX_STATE_WAITING_FOR_HEADER,        ///< Waiting for header byte to be received (RADARIQ_PACKET_HEAD)    
    RX_STATE_WAITING_FOR_FOOTER,        ///< Waiting for footer byte to be received (RADARIQ_PACKET_FOOT)    
} RadarIQRxState_t;

/**
 * UART receive buffer from device
 */
typedef struct
{
    uint8_t data[RADARIQ_RX_BUFFER_SIZE];    ///< Buffer to store a single packet
    uint16_t len;                            ///< Length of packet in bytes
} RadarIQRxBuffer_t;

/**
 * UART transmit buffer to device
 */
typedef struct
{
    uint8_t data[RADARIQ_TX_BUFFER_SIZE];    ///< Buffer to store a single packet
    uint16_t len;                            ///< Length of packet in bytes
} RadarIQTxBuffer_t;

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
 * Sub-frame types for object tracking and point cloud frames sent over multiple packets
 */
typedef enum
{
    RADARIQ_SUBFRAME_START = 0,                ///< Sub-frame is the first of multiple sub-frames
    RADARIQ_SUBFRAME_MIDDLE = 1,               ///< Sub-frame is 1 or more of the middle sub-frames
    RADARIQ_SUBFRAME_END = 2                   ///< Sub-frame is the last (or only) sub-frame
} RadarIQSubframe_t;

//===============================================================================================//
// OBJECTS
//===============================================================================================//

/**
 * The RadarIQ object definition
 */
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
    uint32_t(*millisCallback)(void);
};

//===============================================================================================//
// CONSTANTS
//===============================================================================================//

/**
 * UART packet control bytes
 */
#define RADARIQ_PACKET_ESC            (uint8_t)0xB2    ///< Escape byte used for any control bytes found in data
#define RADARIQ_PACKET_XOR            (uint8_t)0x04    ///< Exclusive OR byte used for any control bytes found in data
#define RADARIQ_PACKET_HEAD           (uint8_t)0xB0    ///< Packet header byte
#define RADARIQ_PACKET_FOOT           (uint8_t)0xB1    ///< Packet footer byte

//===============================================================================================//
// FILE-SCOPE VARIABLES
//===============================================================================================//

//===============================================================================================//
// FILE-SCOPE FUNCTION PROTOTYPES
//===============================================================================================//

// Packet processing
static void RadarIQ_sendPacket(const RadarIQHandle_t obj);
static RadarIQCommand_t RadarIQ_pollResponse(const RadarIQHandle_t obj);
static RadarIQReturnVal_t RadarIQ_decodePacket(const RadarIQHandle_t obj);
static RadarIQCommand_t RadarIQ_parsePacket(const RadarIQHandle_t obj);
static uint16_t RadarIQ_getCrc16Ccitt(uint8_t const * array, uint8_t len);
static void RadarIQ_encodeHelper(const RadarIQHandle_t obj, uint8_t const databyte);

// Packet parsing
static void RadarIQ_parsePointCloud(const RadarIQHandle_t obj);
static void RadarIQ_parseObjectTracking(const RadarIQHandle_t obj);
static void RadarIQ_parseMessage(const RadarIQHandle_t obj);
static void RadarIQ_parseProcessingStats(const RadarIQHandle_t obj);
static void RadarIQ_parsePointCloudStats(const RadarIQHandle_t obj);
static void RadarIQ_parsePowerStatus(const RadarIQHandle_t obj);

// Byte helpers
static uint16_t RadarIQ_pack16Unsigned(const uint8_t * const data);
static int16_t RadarIQ_pack16Signed(const uint8_t * const data);
static void RadarIQ_unpack16Unsigned(uint16_t const data, uint8_t * const dest);
static void RadarIQ_unpack16Signed(int16_t const data, uint8_t * const dest);

//===============================================================================================//
// GLOBAL-SCOPE FUNCTIONS
//===============================================================================================//

/**
 * Allocates and initializes a RadarIQ object instance using heap allocation.
 * @warning If memory fails to allocate, try reducing the ::RADARIQ_MAX_POINTCLOUD or ::RADARIQ_MAX_OBJECTS values
 *
 * @param sendSerialDataCallback Callback function for sending data over UART to the device
 * @param readSerialDataCallback Callback function for reading UART data from the device
 * @param logCallback Callback function for printing out debug messages e.g. over USB serial
 * @param millisCallback Callback function for reading the microcontroller's uptime in milliseconds
 * 
 * @return A handle for an instance of the RadarIQ_t object
 */ 
RadarIQHandle_t RadarIQ_init(void(*sendSerialDataCallback)(uint8_t * const, const uint16_t),
        RadarIQUartData_t(*readSerialDataCallback)(void),
        void(*logCallback)(char * const),
        uint32_t(*millisCallback)(void))
{
    radariq_assert(NULL != sendSerialDataCallback);
    radariq_assert(NULL != readSerialDataCallback);
    radariq_assert(NULL != logCallback);
    radariq_assert(NULL != millisCallback);

    RadarIQHandle_t handle = malloc(sizeof(RadarIQ_t));
    radariq_assert(NULL != handle);
    memset((void*)handle, 0, sizeof(RadarIQ_t));

    handle->sendSerialDataCallback = sendSerialDataCallback;
    handle->readSerialDataCallback = readSerialDataCallback;
    handle->logCallback = logCallback;
    handle->millisCallback = millisCallback;

    handle->captureMode = RADARIQ_MODE_POINT_CLOUD;
    handle->rxState = RX_STATE_WAITING_FOR_HEADER;

    return handle;
}

/**
 * Reads data from the device UART using the provided callback and checks for a complete packet.
 *
 * @param obj The RadarIQ object handle returned from RadarIQ_init()
 * 
 * @return A packet command value from RadarIQCommand_t, or negative value if none received or an error occurred
 */ 
RadarIQCommand_t RadarIQ_readSerial(const RadarIQHandle_t obj)
{
    radariq_assert(NULL != obj);

    RadarIQCommand_t packet = RADARIQ_CMD_NONE;
    RadarIQUartData_t rxData;
    rxData = obj->readSerialDataCallback();
    
    if (rxData.isReadable)
    {
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
                    if (RadarIQ_decodePacket(obj) == RADARIQ_RETURN_VAL_OK)
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
    }

    return packet;
}

/**
 * Gets a copy of the most recent data received from device.
 * Should be called immediately after a ::RADARIQ_CMD_PNT_CLOUD_FRAME or ::RADARIQ_CMD_OBJ_TRACKING_FRAME packet is returned from ::RadarIQ_readSerial()
 *
 * @param obj The RadarIQ object handle returned from RadarIQ_init()
 * @param dest Pointer to a RadarIQData_t struct to copy the data into
 */ 
void RadarIQ_getData(const RadarIQHandle_t obj, RadarIQData_t * dest)
{
    radariq_assert(NULL != obj);
    radariq_assert(NULL != dest);

    memcpy((void*)dest, (void*)&obj->data, sizeof(RadarIQData_t));
}

/**
 * Gets a copy of the most recent statistics received from device.
 * Should be called immediately after a ::RADARIQ_CMD_PROC_STATS or ::RADARIQ_CMD_POINTCLOUD_STATS packet is returned from RadarIQ_readSerial()
 *
 * @param obj The RadarIQ object handle returned from RadarIQ_init()
 * @param processing Pointer to a RadarIQProcessingStats_t struct to copy the processing statistics into
 * @param pointcloud Pointer to a RadarIQPointcloudStats_t struct to copy the point-cloud statistics into
 * @param temperatures Pointer to a RadarIQChipTemperatures_t struct to copy the radar chip temperatures into
 */ 
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

/**
 * Reads power-good flag from radar power supply which indicates whether it is regulating correctly.
 * Should be called immediately after a ::RADARIQ_CMD_POWER_STATUS packet is returned from RadarIQ_readSerial()
 *
 * @param obj The RadarIQ object handle returned from RadarIQ_init()
 *
 * @return True if radar power supply output is regulated ok, false otherwise
 */ 
bool RadarIQ_isPowerGood(const RadarIQHandle_t obj)
{
    radariq_assert(NULL != obj);
    
    return obj->isPowerGood;    
}

/**
 * Gets a copy of the most recent processing statistics received from device.
 * Should be called immediately after a ::RADARIQ_CMD_PROC_STATS packet is returned from RadarIQ_readSerial()
 *
 * @param obj The RadarIQ object handle returned from RadarIQ_init()
 * @param dest Pointer to a RadarIQProcessingStats_t struct to copy the processing statistics into
 */ 
void RadarIQ_getProcessingStats(const RadarIQHandle_t obj, RadarIQProcessingStats_t * const dest)
{
    radariq_assert(NULL != obj);    
    radariq_assert(NULL != dest);
    
    *dest = obj->stats.processing;
}

/**
 * Gets a copy of the most recent point-cloud statistics received from device.
 * Should be called immediately after a ::RADARIQ_CMD_POINTCLOUD_STATS packet is returned from RadarIQ_readSerial()
 *
 * @param obj The RadarIQ object handle returned from RadarIQ_init()
 * @param dest Pointer to a RadarIQPointcloudStats_t struct to copy the point-cloud statistics into
 */ 
void RadarIQ_getPointCloudStats(const RadarIQHandle_t obj, RadarIQPointcloudStats_t * const dest)
{
    radariq_assert(NULL != obj);    
    radariq_assert(NULL != dest);
    
    *dest = obj->stats.pointcloud;
}

/**
 * Gets a copy of the most recent radar chip temperatures from device.
 * Should be called immediately after a ::RADARIQ_CMD_PROC_STATS packet is returned from RadarIQ_readSerial()
 *
 * @param obj The RadarIQ object handle returned from RadarIQ_init()
 * @param dest Pointer to a RadarIQChipTemperatures_t struct to copy the radar chip temperatures into
 */ 
void RadarIQ_getChipTemperatures(const RadarIQHandle_t obj, RadarIQChipTemperatures_t * const dest)
{
    radariq_assert(NULL != obj);    
    radariq_assert(NULL != dest);
    
    *dest = obj->stats.temperature;
}

//===============================================================================================//
// GLOBAL-SCOPE FUNCTIONS - Debug / Info
//===============================================================================================//

/**
 * Gets the total memory size requirements for one instance of a RadarIQ_t object in bytes.
 *
 * @return The size of a single RadarIQ_t instance in bytes
 */ 
uint32_t RadarIQ_getMemoryUsage()
{
    return sizeof(RadarIQ_t);
}

/**
 * Copies the current contents of the receive packet buffer into a destination buffer.
 * @warning Buffer must be of length ::RADARIQ_RX_BUFFER_SIZE or greater
 *
 * @param obj The RadarIQ object handle returned from RadarIQ_init()
 * @param dest Pointer to the destination buffer
 * 
 * @return The current length of the packet in the buffer, or 0 if none is present
 */ 
uint8_t RadarIQ_getDataBuffer(const RadarIQHandle_t obj, uint8_t* dest)
{
    radariq_assert(NULL != obj);

    memcpy(dest, obj->rxPacket.data, obj->rxPacket.len);

    return obj->rxPacket.len;
}

//===============================================================================================//
// GLOBAL-SCOPE FUNCTIONS - UART Commands
//===============================================================================================//

/**
 * Sends a ::RADARIQ_CMD_CAPTURE_START packet to the device to start radar data capture.
 *
 * @param obj The RadarIQ object handle returned from RadarIQ_init()
 * @param numFrames The number of frames (1-255) to capture, or 0 for continuous capture
 */
void RadarIQ_start(const RadarIQHandle_t obj, const uint8_t numFrames)
{
    radariq_assert(NULL != obj);

    obj->txPacket.data[0] = RADARIQ_CMD_CAPTURE_START;
    obj->txPacket.data[1] = RADARIQ_CMD_VAR_REQUEST;
    obj->txPacket.data[2] = numFrames;
    obj->txPacket.len = 3u;

    RadarIQ_sendPacket(obj);
}

/**
 * Sends a ::RADARIQ_CMD_RESET packet to the device to reset the device.
 *
 * @param obj The RadarIQ object handle returned from RadarIQ_init()
 * @param code The type of reset - device reboot (::RADARIQ_RESET_REBOOT) or factory settings reset (::RADARIQ_RESET_FACTORY_SETTINGS)
 * 
 * @return ::RADARIQ_RETURN_VAL_OK on success, ::RADARIQ_RETURN_VAL_ERR if no valid response was received
 */
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

/**
 * Sends a ::RADARIQ_CMD_SAVE packet to the device to save its current settings to EEPROM.
 *
 * @param obj The RadarIQ object handle returned from RadarIQ_init()
 * 
 * @return ::RADARIQ_RETURN_VAL_OK on success, ::RADARIQ_RETURN_VAL_ERR if no valid response was received
 */
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

/**
 * Sends a ::RADARIQ_CMD_VERSION packet to the device to get its hardware and firmware version numbers.
 *
 * @param obj The RadarIQ object handle returned from RadarIQ_init()
 * @param firmware Pointer to a RadarIQVersion_t struct to copy the firmware version number into
 * @param hardware Pointer to a RadarIQVersion_t struct to copy the hardware version number into
 * 
 * @return ::RADARIQ_RETURN_VAL_OK on success, ::RADARIQ_RETURN_VAL_ERR if no valid response was received
 */
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

/**
 * Sends a ::RADARIQ_CMD_IWR_VERSION packet to the device to get the firmware versions running on the IWR.
 *
 * @param obj The RadarIQ object handle returned from RadarIQ_init()
 * @param sbl Pointer to a RadarIQVersionIWR_t struct to copy the secondary bootloader firmware version information into
 * @param app1 Pointer to a RadarIQVersionIWR_t struct to copy the first application firmware version information into
 * @param app2 Pointer to a RadarIQVersionIWR_t struct to copy the second application firmware version information into
 * 
 * @return ::RADARIQ_RETURN_VAL_OK on success, ::RADARIQ_RETURN_VAL_ERR if no valid response was received
 */
RadarIQReturnVal_t RadarIQ_getRadarVersions(const RadarIQHandle_t obj, RadarIQVersionIWR_t * const sbl,
        RadarIQVersionIWR_t * const app1, RadarIQVersionIWR_t * const app2)
{
    radariq_assert(NULL != obj);
    radariq_assert(NULL != sbl);
    radariq_assert(NULL != app1);
    radariq_assert(NULL != app2);

    RadarIQReturnVal_t ret = RADARIQ_RETURN_VAL_OK;

    obj->txPacket.data[0] = RADARIQ_CMD_IWR_VERSION;
    obj->txPacket.data[1] = RADARIQ_CMD_VAR_REQUEST;
    obj->txPacket.len = 2u;

    RadarIQ_sendPacket(obj);

    if (RADARIQ_CMD_IWR_VERSION == RadarIQ_pollResponse(obj))
    {    
        memcpy((void*)sbl, (void*)&obj->rxPacket.data[2], 4);
        memset(sbl->name, 0, 20);    
        memcpy((void*)app1, (void*)&obj->rxPacket.data[6], sizeof(RadarIQVersionIWR_t));
        memcpy((void*)app2, (void*)&obj->rxPacket.data[30], sizeof(RadarIQVersionIWR_t));
    }
    else
    {
        ret = RADARIQ_RETURN_VAL_ERR;
    }

    return ret;
}

/**
 * Sends a ::RADARIQ_CMD_SERIAL packet to the device to read its serial number
 *
 * @param obj The RadarIQ object handle returned from RadarIQ_init()
 * @param serial Pointer to a RadarIQSerialNo_t struct to copy the serial number into
 * 
 * @return ::RADARIQ_RETURN_VAL_OK on success, ::RADARIQ_RETURN_VAL_ERR if no valid response was received
 */
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

/**
 * Sends a ::RADARIQ_CMD_FRAME_RATE packet to the device to read the currently set capture frame rate.
 *
 * @param obj The RadarIQ object handle returned from RadarIQ_init()
 * @param rate Pointer to a variable to copy the frame rate into
 * 
 * @return ::RADARIQ_RETURN_VAL_OK on success, ::RADARIQ_RETURN_VAL_ERR if no valid response was received
 */
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

/**
 * Sends a ::RADARIQ_CMD_FRAME_RATE packet to the device to set the capture frame rate.
 *
 * @param obj The RadarIQ object handle returned from RadarIQ_init()
 * @param rate Frame rate to set in frames/second
 * 
 * @return ::RADARIQ_RETURN_VAL_OK on success, ::RADARIQ_RETURN_VAL_ERR if no valid response was received,
 * ::RADARIQ_RETURN_VAL_WARNING if provided frame rate was out of valid range and limited
 */
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

/**
 * Sends a ::RADARIQ_CMD_MODE packet to the device to read the currently set capture mode.
 *
 * @param obj The RadarIQ object handle returned from RadarIQ_init()
 * @param mode Pointer to a RadarIQCaptureMode_t variable to copy the capture mode into
 * 
 * @return ::RADARIQ_RETURN_VAL_OK on success, ::RADARIQ_RETURN_VAL_ERR if no valid response was received
 */
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

/**
 * Sends a ::RADARIQ_CMD_MODE packet to the device to set the capture mode.
 *
 * @param obj The RadarIQ object handle returned from RadarIQ_init()
 * @param mode The capture mode (::RADARIQ_MODE_POINT_CLOUD or ::RADARIQ_MODE_OBJECT_TRACKING) to set
 * 
 * @return ::RADARIQ_RETURN_VAL_OK on success, ::RADARIQ_RETURN_VAL_ERR if no valid response was received or supplied mode is invalid
 */
RadarIQReturnVal_t RadarIQ_setMode(const RadarIQHandle_t obj, RadarIQCaptureMode_t mode)
{
    radariq_assert(NULL != obj);

    RadarIQReturnVal_t ret = RADARIQ_RETURN_VAL_OK;

    if ((RADARIQ_MODE_POINT_CLOUD > mode) || (RADARIQ_MODE_RAW_DATA < mode))    //TODO remove raw
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

/**
 * Sends a ::RADARIQ_CMD_DIST_FILT packet to the device to read the current distance filter settings.
 *
 * @param obj The RadarIQ object handle returned from RadarIQ_init()
 * @param min Pointer to a variable to copy the minimum distance setting into
 * @param max Pointer to a variable to copy the maximum distance setting into
 * 
 * @return ::RADARIQ_RETURN_VAL_OK on success, ::RADARIQ_RETURN_VAL_ERR if no valid response was received
 */
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
        *min = RadarIQ_pack16Unsigned(&obj->rxPacket.data[2]);
        *max = RadarIQ_pack16Unsigned(&obj->rxPacket.data[4]);
    }
    else
    {
        ret = RADARIQ_RETURN_VAL_ERR;
    }

    return ret;
}

/**
 * Sends a ::RADARIQ_CMD_DIST_FILT packet to the device to set the distance filter settings.
 *
 * @param obj The RadarIQ object handle returned from RadarIQ_init()
 * @param min The minimum distance setting into apply
 * @param max The maximum distance setting into apply
 * 
 * @return ::RADARIQ_RETURN_VAL_OK on success, ::RADARIQ_RETURN_VAL_ERR if no valid response was received,
 * ::RADARIQ_RETURN_VAL_WARNING if provided min or max values were out of valid range and limited
 */
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
    RadarIQ_unpack16Unsigned(min, &obj->txPacket.data[2]);
    RadarIQ_unpack16Unsigned(max, &obj->txPacket.data[4]);    
    obj->txPacket.len = 6u;
    
    RadarIQ_sendPacket(obj);

    if (RADARIQ_CMD_DIST_FILT != RadarIQ_pollResponse(obj))
    {
        ret = RADARIQ_RETURN_VAL_ERR;    
    }

    return ret;
}

/**
 * Sends a ::RADARIQ_CMD_ANGLE_FILT packet to the device to read the current angle filter settings.
 *
 * @param obj The RadarIQ object handle returned from RadarIQ_init()
 * @param min Pointer to a variable to copy the minimum angle setting into
 * @param max Pointer to a variable to copy the maximum angle setting into
 * 
 * @return ::RADARIQ_RETURN_VAL_OK on success, ::RADARIQ_RETURN_VAL_ERR if no valid response was received
 */
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

/**
 * Sends a ::RADARIQ_CMD_ANGLE_FILT packet to the device to set the angle filter settings.
 *
 * @param obj The RadarIQ object handle returned from RadarIQ_init()
 * @param min The minimum angle setting into apply
 * @param max The maximum angle setting into apply
 * 
 * @return ::RADARIQ_RETURN_VAL_OK on success, ::RADARIQ_RETURN_VAL_ERR if no valid response was received,
 * ::RADARIQ_RETURN_VAL_WARNING if provided min or max values were out of valid range and limited
 */
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

/**
 * Sends a ::RADARIQ_CMD_MOVING_FILT packet to the device to read the current moving filter setting.
 *
 * @param obj The RadarIQ object handle returned from RadarIQ_init()
 * @param filter Pointer to a RadarIQMovingFilterMode_t variable to copy the filter setting into
 * 
 * @return ::RADARIQ_RETURN_VAL_OK on success, ::RADARIQ_RETURN_VAL_ERR if no valid response was received
 */
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

/**
 * Sends a ::RADARIQ_CMD_MOVING_FILT packet to the device to set the moving filter setting.
 *
 * @param obj The RadarIQ object handle returned from RadarIQ_init()
 * @param filter The moving filter setting to apply
 * 
 * @return ::RADARIQ_RETURN_VAL_OK on success, ::RADARIQ_RETURN_VAL_ERR if no valid response was received 
 * or supplied filter setting provided is invalid
 */
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

/**
 * Sends a ::RADARIQ_CMD_PNT_DENSITY packet to the device to read the current point-cloud point density setting.
 *
 * @param obj The RadarIQ object handle returned from RadarIQ_init()
 * @param density Pointer to a RadarIQPointDensity_t variable to copy the point density setting into
 * 
 * @return ::RADARIQ_RETURN_VAL_OK on success, ::RADARIQ_RETURN_VAL_ERR if no valid response was received
 */
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

/**
 * Sends a ::RADARIQ_CMD_PNT_DENSITY packet to the device to set the point-cloud point density.
 *
 * @param obj The RadarIQ object handle returned from RadarIQ_init()
 * @param density The point density setting to apply
 * 
 * @return ::RADARIQ_RETURN_VAL_OK on success, ::RADARIQ_RETURN_VAL_ERR if no valid response was received
 */
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

/**
 * Sends a ::RADARIQ_CMD_SENSITIVITY packet to the device to read the current point-cloud sensitivity level.
 *
 * @param obj The RadarIQ object handle returned from RadarIQ_init()
 * @param sensitivity Pointer to a variable to copy the sensitivity level into
 * 
 * @return ::RADARIQ_RETURN_VAL_OK on success, ::RADARIQ_RETURN_VAL_ERR if no valid response was received
 */
RadarIQReturnVal_t RadarIQ_getSensitivity(const RadarIQHandle_t obj, uint8_t * const sensitivity)
{
    radariq_assert(NULL != obj);

    RadarIQReturnVal_t ret = RADARIQ_RETURN_VAL_OK;

    obj->txPacket.data[0] = RADARIQ_CMD_SENSITIVITY;
    obj->txPacket.data[1] = RADARIQ_CMD_VAR_REQUEST;
    obj->txPacket.len = 2u;

    RadarIQ_sendPacket(obj);

    if (RADARIQ_CMD_SENSITIVITY == RadarIQ_pollResponse(obj))
    {
        *sensitivity = obj->rxPacket.data[2];
    }
    else
    {
        ret = RADARIQ_RETURN_VAL_ERR;    
    }

    return ret;
}

/**
 * Sends a ::RADARIQ_CMD_SENSITIVITY packet to the device to set the point-cloud sensitivity level.
 *
 * @param obj The RadarIQ object handle returned from RadarIQ_init()
 * @param sensitivity The sensitivity level (0-9) to set
 * 
 * @return ::RADARIQ_RETURN_VAL_OK on success, ::RADARIQ_RETURN_VAL_ERR if no valid response was received
 */
RadarIQReturnVal_t RadarIQ_setSensitivity(const RadarIQHandle_t obj, uint8_t sensitivity)
{
    radariq_assert(NULL != obj);

    RadarIQReturnVal_t ret = RADARIQ_RETURN_VAL_OK;

    if (RADARIQ_MAX_SENSITIVITY < sensitivity)
    {
        sensitivity = RADARIQ_MAX_SENSITIVITY;
        ret = RADARIQ_RETURN_VAL_WARNING;    
    }

    obj->txPacket.data[0] = RADARIQ_CMD_SENSITIVITY;
    obj->txPacket.data[1] = RADARIQ_CMD_VAR_SET;
    obj->txPacket.data[2] = sensitivity;
    obj->txPacket.len = 3u;

    RadarIQ_sendPacket(obj);

    if (RADARIQ_CMD_SENSITIVITY != RadarIQ_pollResponse(obj))
    {
        ret = RADARIQ_RETURN_VAL_ERR;    
    }

    return ret;
}

/**
 * Sends a ::RADARIQ_CMD_HEIGHT_FILT packet to the device to read the current height filter settings.
 *
 * @param obj The RadarIQ object handle returned from RadarIQ_init()
 * @param min Pointer to a variable to copy the minimum height setting into
 * @param max Pointer to a variable to copy the maximum height setting into
 * 
 * @return ::RADARIQ_RETURN_VAL_OK on success, ::RADARIQ_RETURN_VAL_ERR if no valid response was received
 */
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
        *min = RadarIQ_pack16Signed(&obj->rxPacket.data[2]);
        *max = RadarIQ_pack16Signed(&obj->rxPacket.data[4]);
    }
    else
    {
        ret = RADARIQ_RETURN_VAL_ERR;
    }

    return ret;
}

/**
 * Sends a ::RADARIQ_CMD_HEIGHT_FILT packet to the device to set the height filter settings.
 *
 * @param obj The RadarIQ object handle returned from RadarIQ_init()
 * @param min The minimum height setting into apply
 * @param max The minimum height setting into apply
 * 
 * @return ::RADARIQ_RETURN_VAL_OK on success, ::RADARIQ_RETURN_VAL_ERR if no valid response was received
 */
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
    RadarIQ_unpack16Signed(min, &obj->txPacket.data[2]);
    RadarIQ_unpack16Signed(max, &obj->txPacket.data[4]);    
    obj->txPacket.len = 6u;
    
    RadarIQ_sendPacket(obj);

    if (RADARIQ_CMD_HEIGHT_FILT != RadarIQ_pollResponse(obj))
    {
        ret = RADARIQ_RETURN_VAL_ERR;    
    }

    return ret;
}

/**
 * Sends a ::RADARIQ_CMD_SCENE_CALIB packet to the device to perform a scene calibration.
 *
 * @param obj The RadarIQ object handle returned from RadarIQ_init()
 * 
 * @return ::RADARIQ_RETURN_VAL_OK on success, ::RADARIQ_RETURN_VAL_ERR if no valid response was received
 */
RadarIQReturnVal_t RadarIQ_sceneCalibrate(const RadarIQHandle_t obj)
{
    radariq_assert(NULL != obj);
    
    RadarIQReturnVal_t ret = RADARIQ_RETURN_VAL_ERR;
    
    obj->txPacket.data[0] = RADARIQ_CMD_SCENE_CALIB;
    obj->txPacket.data[1] = RADARIQ_CMD_VAR_SET;
    obj->txPacket.len = 2u;

    RadarIQ_sendPacket(obj);
    
    // Poll for acknowledgment message
    // Several other messages are expected to be received before the ack
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

/**
 * Sends a ::RADARIQ_CMD_OBJECT_SIZE packet to the device to read the current target object size for tracking.
 *
 * @param obj The RadarIQ object handle returned from RadarIQ_init()
 * @param size Pointer to a variable to copy the target object size into
 * 
 * @return ::RADARIQ_RETURN_VAL_OK on success, ::RADARIQ_RETURN_VAL_ERR if no valid response was received
 */
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

/**
 * Sends a ::RADARIQ_CMD_OBJECT_SIZE packet to the device to set the target object size for tracking.
 *
 * @param obj The RadarIQ object handle returned from RadarIQ_init()
 * @param size The target object size (0-4) to set
 * 
 * @return ::RADARIQ_RETURN_VAL_OK on success, ::RADARIQ_RETURN_VAL_ERR if no valid response was received,
 * ::RADARIQ_RETURN_VAL_WARNING if provided value was out of valid range and limited
 */
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

/**
 * Polls for a response from the device UART.
 * Function will block until a valid packet is received or a timeout of 1 second is reached
 *
 * @param obj The RadarIQ object handle returned from RadarIQ_init()
 * 
 * @return A packet command value from RadarIQCommand_t, or negative value if none received or an error occurred
 */
static RadarIQCommand_t RadarIQ_pollResponse(const RadarIQHandle_t obj)
{
    radariq_assert(NULL != obj);

    RadarIQCommand_t response;

    int32_t startTime = (int32_t)obj->millisCallback();
    char strMsg[32]; 
    
    do
    {        
        response = RadarIQ_readSerial(obj);
    
        if (RADARIQ_CMD_NONE < response)
        {
            //sprintf(strMsg, "pollResponse: response %i", response);
            //obj->logCallback(strMsg);
            break;    
        }
    }while (startTime >= (((int32_t)obj->millisCallback()) - 1000));

    if (RADARIQ_CMD_NONE >= response)
    {
        //sprintf(strMsg, "pollResponse: timeout at %i", radariq_get_seconds);
        //obj->logCallback(strMsg);
    }

    return response;
}

/**
 * Parses a packet received from the device UART.
 *
 * @param obj The RadarIQ object handle returned from RadarIQ_init()
 * 
 * @return A packet command value from RadarIQCommand_t, or negative value if an error occurred
 */
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
    case RADARIQ_CMD_SENSITIVITY:
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

/**
 * Parses a point-cloud packet received from the device UART.
 *
 * @param obj The RadarIQ object handle returned from RadarIQ_init()
 */
static void RadarIQ_parsePointCloud(const RadarIQHandle_t obj)
{
    RadarIQCommand_t ret = RADARIQ_CMD_NONE;
  
    const RadarIQSubframe_t subFrameType = (RadarIQSubframe_t)obj->rxPacket.data[2];
    uint8_t pointCount = obj->rxPacket.data[3];
    obj->data.pointCloud.isFrameComplete = false;
    obj->data.pointCloud.numPoints = pointCount;
    uint8_t packetIdx = 4u;

    // Loop through points in packet
    uint8_t pointNum;
    for (pointNum = 0u; pointNum < pointCount; pointNum++)
    {
        obj->data.pointCloud.points[obj->numDataPoints].x = RadarIQ_pack16Signed(&obj->rxPacket.data[packetIdx]);
        packetIdx += 2u;
        obj->data.pointCloud.points[obj->numDataPoints].y = RadarIQ_pack16Signed(&obj->rxPacket.data[packetIdx]);
        packetIdx += 2u;
        obj->data.pointCloud.points[obj->numDataPoints].z = RadarIQ_pack16Signed(&obj->rxPacket.data[packetIdx]);
        packetIdx += 2u;
        obj->data.pointCloud.points[obj->numDataPoints].intensity = obj->rxPacket.data[packetIdx];
        packetIdx++;
        obj->data.pointCloud.points[obj->numDataPoints].velocity = RadarIQ_pack16Signed(&obj->rxPacket.data[packetIdx]);
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

/**
 * Parses a object-tracking packet received from the device UART.
 *
 * @param obj The RadarIQ object handle returned from RadarIQ_init()
 */
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
        obj->data.objectTracking.objects[obj->numDataPoints].xPos = RadarIQ_pack16Signed(&obj->rxPacket.data[packetIdx]);
        packetIdx += 2u;
        obj->data.objectTracking.objects[obj->numDataPoints].yPos = RadarIQ_pack16Signed(&obj->rxPacket.data[packetIdx]);
        packetIdx += 2u;
        obj->data.objectTracking.objects[obj->numDataPoints].zPos = RadarIQ_pack16Signed(&obj->rxPacket.data[packetIdx]);
        packetIdx += 2u;
        obj->data.objectTracking.objects[obj->numDataPoints].xVel = RadarIQ_pack16Signed(&obj->rxPacket.data[packetIdx]);
        packetIdx += 2u;
        obj->data.objectTracking.objects[obj->numDataPoints].yVel = RadarIQ_pack16Signed(&obj->rxPacket.data[packetIdx]);
        packetIdx += 2u;
        obj->data.objectTracking.objects[obj->numDataPoints].zVel = RadarIQ_pack16Signed(&obj->rxPacket.data[packetIdx]);
        packetIdx += 2u;
        obj->data.objectTracking.objects[obj->numDataPoints].xAcc = RadarIQ_pack16Signed(&obj->rxPacket.data[packetIdx]);
        packetIdx += 2u;
        obj->data.objectTracking.objects[obj->numDataPoints].yAcc = RadarIQ_pack16Signed(&obj->rxPacket.data[packetIdx]);
        packetIdx += 2u;
        obj->data.objectTracking.objects[obj->numDataPoints].zAcc = RadarIQ_pack16Signed(&obj->rxPacket.data[packetIdx]);
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

/**
 * Parses a message packet received from the device UART.
 *
 * @param obj The RadarIQ object handle returned from RadarIQ_init()
 */
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
    
    snprintf(strLog, sizeof(strLog), "Radar Message - %s (%u):, %s", strMsgType, (int)msgType, (char*)&obj->rxPacket.data[4]);
    strLog[255] = 0;
    obj->logCallback(strLog);
}

/**
 * Parses a processing statistics packet received from the device UART.
 *
 * @param obj The RadarIQ object handle returned from RadarIQ_init()
 */
static void RadarIQ_parseProcessingStats(const RadarIQHandle_t obj)
{
    radariq_assert(NULL != obj);
    
    memcpy((void*)&obj->stats.processing, (void*)&obj->rxPacket.data[2], 28);
    memcpy((void*)&obj->stats.temperature, (void*)&obj->rxPacket.data[30], 20);
}

/**
 * Parses a point-cloud statistics packet received from the device UART.
 *
 * @param obj The RadarIQ object handle returned from RadarIQ_init()
 */
static void RadarIQ_parsePointCloudStats(const RadarIQHandle_t obj)
{
    radariq_assert(NULL != obj);
    
    memcpy((void*)&obj->stats.pointcloud, (void*)&obj->rxPacket.data[2], 26);    
}

/**
 * Parses a power status packet received from the device UART.
 *
 * @param obj The RadarIQ object handle returned from RadarIQ_init()
 */
static void RadarIQ_parsePowerStatus(const RadarIQHandle_t obj)
{
    radariq_assert(NULL != obj);
    
    obj->isPowerGood = !obj->rxPacket.data[2];
}

/**
 * Sends a packet to the device over UART.
 *
 * @param obj The RadarIQ object handle returned from RadarIQ_init()
 */
static void RadarIQ_sendPacket(const RadarIQHandle_t obj)
{
    radariq_assert(NULL != obj);

    // Calculate CRC
    uint16_t const crc = RadarIQ_getCrc16Ccitt(obj->txPacket.data, obj->txPacket.len);

    // Send packet header
    obj->txBuffer.data[0] = RADARIQ_PACKET_HEAD;
    obj->txBuffer.len = 1u;

    // Loop through data, check for footer bytes in data and escape them
    for (uint8_t idx = 0u; idx < obj->txPacket.len; idx++)
    {
        RadarIQ_encodeHelper(obj, obj->txPacket.data[idx]);
    }

    RadarIQ_encodeHelper(obj, (uint8_t)((crc & (uint16_t)0xFF00) >> 8u));
    RadarIQ_encodeHelper(obj, (uint8_t)(crc & (uint16_t)0x00FF));

    // Insert footer
    obj->txBuffer.data[obj->txBuffer.len] = RADARIQ_PACKET_FOOT;
    obj->txBuffer.len++;

    obj->sendSerialDataCallback(obj->txBuffer.data, obj->txBuffer.len);
}

/**
 * Decodes a packet received from the device over UART.
 *
 * @param obj The RadarIQ object handle returned from RadarIQ_init()
 * 
 * @return ::RADARIQ_RETURN_VAL_OK if packet CRC check is ok, ::RADARIQ_RETURN_VAL_ERR otherwise
 */
static RadarIQReturnVal_t RadarIQ_decodePacket(const RadarIQHandle_t obj)
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

    // Calculate crc from decoded packet (crc calculation does not include header, footer, or crc bytes)
    uint16_t const crc = RadarIQ_getCrc16Ccitt(obj->rxPacket.data, obj->rxPacket.len - 2u);

    // Get crc from the packet after it has been decoded
    uint16_t const rxCrc = (uint16_t) ( (*(obj->rxPacket.data + obj->rxPacket.len - 2u) & (uint16_t)0xFF) << 8u) |
            (*(obj->rxPacket.data + obj->rxPacket.len - 1u) & (uint16_t)0xFF);

    RadarIQReturnVal_t ret = RADARIQ_RETURN_VAL_ERR;

    if (crc == rxCrc)
    {
        ret = RADARIQ_RETURN_VAL_OK;
    }

    return ret;
}

/**
 * Encodes a byte into a packet to send to the device over UART
 *
 * @param obj The RadarIQ object handle returned from RadarIQ_init()
 * @param databyte The data byte to encode
 */
static void RadarIQ_encodeHelper(const RadarIQHandle_t obj, uint8_t const databyte)
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

/**
 * Calculates the 16-bit CRC of an array.
 *
 * @param obj The RadarIQ object handle returned from RadarIQ_init()
 * @param array The array to calculate the CRC for
 * @param len The length of the array in bytes
 * 
 * @return The calculated CRC value
 */
static uint16_t RadarIQ_getCrc16Ccitt(uint8_t const * array, uint8_t len)
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

/**
 * Packs 2-bytes of a buffer into an unsigned 16-bit integer.
 *
 * @param data The buffer data bytes to pack
 * 
 * @return The packed integer value
 */
static uint16_t RadarIQ_pack16Unsigned(const uint8_t * const data)
{
    uint16_t dest = 0u;
    dest |= (((uint32_t)data[1] << 8u) & 0xFF00u);
    dest |= (((uint32_t)data[0]) & 0x00FFu);

    return dest;
}

/**
 * Packs 2-bytes of a buffer into a signed 16-bit integer
 *
 * @param data The buffer data bytes to pack
 * 
 * @return The packed integer value
 */
static int16_t RadarIQ_pack16Signed(const uint8_t * const data)
{
    uint16_t temp = 0u;
    int16_t returnValue = 0;

    temp |= ((data[1] << 8u) & 0xFF00u);
    temp |= ((data[0]) & 0x00FFu);

    memcpy(&returnValue, &temp, sizeof(uint16_t));

    return returnValue;
}

/**
 * Unpacks an unsigned 16-bit integer into a 2-byte buffer.
 *
 * @param data The integer value to unpack
 * @param dest The buffer to unpack the data byte into
 */
static void RadarIQ_unpack16Unsigned(uint16_t const data, uint8_t * const dest)
{
    dest[1] = (data >> 8u) & 0xFFu;
    dest[0] = data & 0xFFu;
}

/**
 * Unpacks a signed 16-bit integer into a 2-byte buffer.
 *
 * @param data The integer value to unpack
 * @param dest The buffer to unpack the data byte into
 */
static void RadarIQ_unpack16Signed(int16_t const data, uint8_t * const dest)
{
    uint16_t temp = 0u;
    memcpy(&temp, &data, sizeof(uint16_t));

    dest[1] = (temp >> 8u) & 0xFFu;
    dest[0] = temp & 0xFFu;
}