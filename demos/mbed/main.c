//------------------------------------------------------------------------------
//                                                                            --
//                                RadarIQ                                     --
//                         C-SDK Demo Application                             --
//                                                                            --
//                   (C) 2021 RadarIQ <support@radariq.io>                    --
//                                                                            --
//                            License: MIT                                    --
//                                                                            --
//------------------------------------------------------------------------------ 


//------------------------------------------------------------------------------
// Includes
//----------

#include "mbed.h"
#include "RadarIQ.h"

//------------------------------------------------------------------------------
// Objects
//---------

static BufferedSerial radarUart(PA_9, PA_10);
static BufferedSerial debugUart(USBTX, USBRX);
static DigitalOut debugLed(LED1);
static RadarIQHandle_t myRadar;

//------------------------------------------------------------------------------
// Function Prototypes
//------------------------------------------------------------------------------

static void callbackSendRadarData(uint8_t * const buffer, const uint16_t len);
static RadarIQUartData_t callbackReadSerialData(void);
static void callbackRadarLog(char * const buffer);


//------------------------------------------------------------------------------
// Program Entry Point
//------------------------------------------------------------------------------

int main()
{
    // Setup serial ports
    debugUart.set_baud(115200);
    radarUart.set_baud(115200);
    radarUart.sync();
    
    // Create the RadarIQ object
    myRadar = RadarIQ_init(callbackSendRadarData, callbackReadSerialData, callbackRadarLog);
    printf("* Created RadarIQ instance, using %u bytes of memory\n\r", RadarIQ_getMemoryUsage());
    
    // Get RadarIQ version
    RadarIQVersion_t firmware;
    RadarIQVersion_t hardware;
    if (RadarIQ_getVersion(myRadar, &firmware, &hardware) == RADARIQ_RETURN_VAL_OK)
    {
        printf("* Firmware version = %u.%u.%u\n\r", firmware.major, firmware.minor, firmware.build);
    }
    else
    {
        printf("* Error reading version number from RadarIQ module\n\r");    
    }
    
    // Get serial number
    RadarIQSerialNo_t serialNo;
    if (RadarIQ_getSerialNumber(myRadar, &serialNo) == RADARIQ_RETURN_VAL_OK)
    {
        printf("* Serial number: a = 0x%8x, b = 0x%8x\n\r", serialNo.a, serialNo.b);
    }
    else
    {
        printf("* Error reading serial number from RadarIQ module\n\r");  
    }
    
    // Set frame rate
    if (RadarIQ_setFrameRate(myRadar, 2u) == RADARIQ_RETURN_VAL_OK)
    {
        uint8_t frameRate;
        if (RadarIQ_getFrameRate(myRadar, &frameRate) == RADARIQ_RETURN_VAL_OK)
        {
            printf("* Frame rate is set to %u\n\r", frameRate);
        }
        else
        {
            printf("* Error reading frame rate from RadarIQ module\n");  
        }
    }
    else
    {
        printf("* Error setting frame rate of RadarIQ module\n\r");  
    }

    // Set capture mode to point-cloud
    if (RadarIQ_setMode(myRadar, RADARIQ_MODE_POINT_CLOUD) == RADARIQ_RETURN_VAL_OK)
    {
        RadarIQCaptureMode_t mode;
        if (RadarIQ_getMode(myRadar, &mode) == RADARIQ_RETURN_VAL_OK)
        {
            printf("* Capture mode is set to: %i\n\r", mode);
        }
        else
        {
            printf("* Error reading capture mode from RadarIQ module\n\r");  
        }
    }
    else
    {
        printf("* Error setting capture mode of RadarIQ module\n\r");  
    }
    
    // Set distance filter
    if (RadarIQ_setDistanceFilter(myRadar, 0u, 10000u) == RADARIQ_RETURN_VAL_OK)
    {
        uint16_t dMin, dMax;
        if (RadarIQ_getDistanceFilter(myRadar, &dMin, &dMax) == RADARIQ_RETURN_VAL_OK)
        {
            printf("* Distance filter: min = %u, max = %u\n\r", dMin, dMax);
        }
        else
        {
            printf("* Error reading distance filter setting from RadarIQ module\n\r");   
        }
    }
    else
    {
        printf("* Error setting distance filter of RadarIQ module\n\r");  
    }
    
    // Set angle filter
    if (RadarIQ_setAngleFilter(myRadar, -55, 55) == RADARIQ_RETURN_VAL_OK)
    { 
        int8_t angleMin, angleMax;
        if (RadarIQ_getAngleFilter(myRadar, &angleMin, &angleMax) == RADARIQ_RETURN_VAL_OK)
        {
            printf("* Angle filter: min = %i, max = %i\n\r", angleMin, angleMax);   
        }
        else
        {
            printf("* Error reading angle filter setting from RadarIQ module\n\r");   
        }
    }
    else
    {
        printf("* Error setting angle filter of RadarIQ module\n\r");  
    }
    
    // Set height filter
    if (RadarIQ_setHeightFilter(myRadar, -1000, 1000) == RADARIQ_RETURN_VAL_OK)
    {
        int16_t heightMin, heightMax;
        if (RadarIQ_getHeightFilter(myRadar, &heightMin, &heightMax) == RADARIQ_RETURN_VAL_OK)
        {
            printf("* Height filter: min = %i, max = %i\n\r", heightMin, heightMax);   
        }
        else
        {
            printf("* Error reading height filter setting from RadarIQ module\n\r");   
        }
    }
    else
    {
        printf("* Error setting height filter of RadarIQ module\n\r");  
    }
    
    // Set certainty level
    if (RadarIQ_setCertainty(myRadar, 6u) == RADARIQ_RETURN_VAL_OK)
    {
        uint8_t certainty;
        if (RadarIQ_getCertainty(myRadar, &certainty) == RADARIQ_RETURN_VAL_OK)
        {
            printf("* Certainty level is set to: %u\n\r", certainty);   
        }
        else
        {
            printf("* Error reading certainty level from RadarIQ module\n\r");  
        }
    }
    else
    {
        printf("* Error setting certainty level of RadarIQ module\n\r");  
    }
    
    // Set moving filter option
    if (RadarIQ_setMovingFilter(myRadar, RADARIQ_MOVING_BOTH) == RADARIQ_RETURN_VAL_OK)
    {
        RadarIQMovingFilterMode_t movingFilter;
        if (RadarIQ_getMovingFilter(myRadar, &movingFilter) == RADARIQ_RETURN_VAL_OK)
        {
            printf("* Moving filter set to: %i\n\r", movingFilter);   
        }
        else
        {
            printf("* Error reading moving filter setting from RadarIQ module\n\r");  
        }
    }
    else
    {
        printf("* Error setting moving filter of RadarIQ module\n\r");  
    }
    
    // Save settings to memory on RadarIQ module
    if (RadarIQ_save(myRadar) == RADARIQ_RETURN_VAL_OK)
    {
        printf("* Saved radar settings successfully\n\r");
    }
    else
    {
        printf("* Failed to save radar settings\n\r");    
    }
    
    // Run a scene calibration
    if (RadarIQ_sceneCalibrate(myRadar) == RADARIQ_RETURN_VAL_OK)
    {
        printf("* Ran scene calibration successfully\n\r");  
    }
    else
    {
        printf("* Failed to run scene calibration\n\r");    
    }

    // Send start capture command
    RadarIQ_start(myRadar, 10);

    // Program loop
    while(1)
    {   
        // Check if  serial data is available to read
        if (radarUart.readable())
        {
            // Process the serial data
            RadarIQCommand_t packet = RadarIQ_readSerial(myRadar);
              
            // Check received packet type
            switch (packet)
            {
                // Point cloud frame
                case RADARIQ_CMD_PNT_CLOUD_FRAME:
                {
                    RadarIQData_t radarData;
                    RadarIQ_getData(myRadar, &radarData);
                                    
                    printf("\n\r** Data num points = %u\n\r", radarData.pointCloud.numPoints);
                    
                    for (uint32_t i = 0u; i < radarData.pointCloud.numPoints; i++)
                    {
                        printf("*  %u: x = %i, y = %i, z = %i, i = %u, v = %i\n\r", i, 
                            radarData.pointCloud.points[i].x, radarData.pointCloud.points[i].y,
                            radarData.pointCloud.points[i].z, radarData.pointCloud.points[i].intensity,
                            radarData.pointCloud.points[i].velocity);
                    }
                    
                    break;    
                } 
                
                // Processing statistics
                case RADARIQ_CMD_PROC_STATS:
                {
                    RadarIQProcessingStats_t processing;
                    RadarIQChipTemperatures_t temperatures;
                    
                    RadarIQ_getProcessingStats(myRadar, &processing);
                    RadarIQ_getChipTemperatures(myRadar, &temperatures);
                    
                    printf("* Processing: %u, %u, %u, %u, %u, %u, %u\n\r", processing.activeFrameCPULoad,
                        processing.interFrameCPULoad, processing.interFrameProcTime,
                        processing.transmitOutputTime, processing.interFrameProcMargin,
                        processing.interChirpProcMargin, processing.uartTransmitTime);  
                    
                    printf("* Temperature: %i, %i, %i, %i, %i, %i, %i, %i, %i, %i\n\r", temperatures.sensor0,
                        temperatures.sensor1, temperatures.powerManagement, temperatures.rx0,
                        temperatures.rx1, temperatures.rx2, temperatures.rx3, temperatures.tx0,
                        temperatures.tx1, temperatures.tx2); 
    
                    break;    
                }
                
                case RADARIQ_CMD_POINTCLOUD_STATS:
                {
                    RadarIQPointcloudStats_t pointcloud;
                    
                    RadarIQ_getPointCloudStats(myRadar, &pointcloud);
                    
                    printf("* Point cloud: %u, %u, %u, %u, %u, %u, %u, %u\n\r", pointcloud.frameAggregatingTime,
                        pointcloud.intensitySortTime, pointcloud.nearestNeighboursTime,
                        pointcloud.uartTransmitTime, pointcloud.numFilteredPoints,
                        pointcloud.numPointsTransmitted, pointcloud.inputPointsTruncated,
                        pointcloud.outputPointsTruncated);     
                    
                }
                
                default:
                {
                }    
            }   
        }
    }
}

//------------------------------------------------------------------------------
// Radar Callback Functions
//--------------------------

static void callbackSendRadarData(uint8_t * const buffer, const uint16_t len)
{
    radarUart.write(buffer, len);
}

static RadarIQUartData_t callbackReadSerialData()
{ 
    RadarIQUartData_t ret;
    
    ret.isReadable = radarUart.readable();
    if (ret.isReadable)
    {
        radarUart.read(&ret.data, 1);
    }
    
    return ret;
}

static void callbackRadarLog(char * const buffer)
{
    printf("* Log: %s\n\r", buffer);    
}