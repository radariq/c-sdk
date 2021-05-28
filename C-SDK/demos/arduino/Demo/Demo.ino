//------------------------------------------------------------------------------
//                                                                            --
//                                RadarIQ                                     --                                         
//                       C-SDK Demo Application 2021                          --
//                                                                            --
//------------------------------------------------------------------------------

#define __ASSERT_USE_STDERR

//------------------------------------------------------------------------------
// Includes
//----------

#include <assert.h>
#include <Arduino.h>
#include "RadarIQ.h"

//------------------------------------------------------------------------------
// Objects
//---------

static RadarIQHandle_t myRadar;

//------------------------------------------------------------------------------
// Function Prototypes
//---------------------

static void callbackSendRadarData(uint8_t * const buffer, const uint16_t len);
static RadarIQUartData_t callbackReadSerialData(void);
static void callbackRadarLog(char * const buffer);

//------------------------------------------------------------------------------
// Program Entry Point
//------------------------------------------------------------------------------

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  
  Serial.begin(115200);   // PC debug serial port
  Serial1.begin(115200);  // RadarIQ device serial port

  char buffer[128];

  // Create the RadarIQ object
  myRadar = RadarIQ_init(callbackSendRadarData, callbackReadSerialData, callbackRadarLog);
  sprintf(buffer, "* Created RadarIQ instance, using %u bytes of memory\n\r", RadarIQ_getMemoryUsage());
  Serial.print(buffer);

  // Get RadarIQ version
  RadarIQVersion_t firmware;
  RadarIQVersion_t hardware;
  if (RadarIQ_getVersion(myRadar, &firmware, &hardware) == RADARIQ_RETURN_VAL_OK)
  {
    sprintf(buffer, "* Firmware version = %u.%u.%u\n\r", firmware.major, firmware.minor, firmware.build);
    Serial.print(buffer);
  }
  else
  {
    Serial.print("* Error reading version number from RadarIQ module\n\r");
  }
    
  // Get serial number
  RadarIQSerialNo_t serialNo;
  if (RadarIQ_getSerialNumber(myRadar, &serialNo) == RADARIQ_RETURN_VAL_OK)
  {
    sprintf(buffer, "* Serial number: a = 0x%8x, b = 0x%8x\n\r", serialNo.a, serialNo.b);
    Serial.print(buffer);
  }
  else
  {
    Serial.print("* Error reading serial number from RadarIQ module\n\r");
  }
  
  // Set frame rate
  if (RadarIQ_setFrameRate(myRadar, 2u) == RADARIQ_RETURN_VAL_OK)
  {
    uint8_t frameRate;
    if (RadarIQ_getFrameRate(myRadar, &frameRate) == RADARIQ_RETURN_VAL_OK)
    {
      sprintf(buffer, "* Frame rate is set to %u\n\r", frameRate);
      Serial.print(buffer);
    }
    else
    {
      Serial.print("* Error reading frame rate from RadarIQ module\n");
    }
  }
  else
  {
    Serial.print("* Error setting frame rate of RadarIQ module\n\r");  
  }

  // Set capture mode to point-cloud
  if (RadarIQ_setMode(myRadar, RADARIQ_MODE_POINT_CLOUD) == RADARIQ_RETURN_VAL_OK)
  {
    RadarIQCaptureMode_t mode;
    if (RadarIQ_getMode(myRadar, &mode) == RADARIQ_RETURN_VAL_OK)
    {
      sprintf(buffer, "* Capture mode is set to: %i\n\r", mode);
      Serial.print(buffer);
    }
    else
    {
      Serial.print("* Error reading capture mode from RadarIQ module\n\r");  
    }
  }
  else
  {
    Serial.print("* Error setting capture mode of RadarIQ module\n\r");  
  }
  
  // Set distance filter
  if (RadarIQ_setDistanceFilter(myRadar, 50u, 500u) == RADARIQ_RETURN_VAL_OK)
  {
    uint16_t dMin, dMax;
    if (RadarIQ_getDistanceFilter(myRadar, &dMin, &dMax) == RADARIQ_RETURN_VAL_OK)
    {
      sprintf(buffer, "* Distance filter: min = %u, max = %u\n\r", dMin, dMax);
      Serial.print(buffer);
    }
    else
    {
      sprintf(buffer, "* Error reading distance filter setting from RadarIQ module\n\r");
      Serial.print(buffer); 
    }
  }
  else
  {
    Serial.print("* Error setting distance filter of RadarIQ module\n\r");  
  }
  
  // Set angle filter
  if (RadarIQ_setAngleFilter(myRadar, -45, 45) == RADARIQ_RETURN_VAL_OK)
  { 
    int8_t angleMin, angleMax;
    if (RadarIQ_getAngleFilter(myRadar, &angleMin, &angleMax) == RADARIQ_RETURN_VAL_OK)
    {
      sprintf(buffer, "* Angle filter: min = %i, max = %i\n\r", angleMin, angleMax);
      Serial.print(buffer);  
    }
    else
    {
      Serial.print("* Error reading angle filter setting from RadarIQ module\n\r");   
    }
  }
  else
  {
    Serial.print("* Error setting angle filter of RadarIQ module\n\r");  
  }
  
  // Set height filter
  if (RadarIQ_setHeightFilter(myRadar, -100, 100) == RADARIQ_RETURN_VAL_OK)
  {
    int16_t heightMin, heightMax;
    if (RadarIQ_getHeightFilter(myRadar, &heightMin, &heightMax) == RADARIQ_RETURN_VAL_OK)
    {
      sprintf(buffer, "* Height filter: min = %i, max = %i\n\r", heightMin, heightMax);
      Serial.print(buffer);   
    }
    else
    {
      Serial.print("* Error reading height filter setting from RadarIQ module\n\r");   
    }
  }
  else
  {
    Serial.print("* Error setting height filter of RadarIQ module\n\r");  
  }
  
  // Set certainty level
  if (RadarIQ_setCertainty(myRadar, 6u) == RADARIQ_RETURN_VAL_OK)
  {
    uint8_t certainty;
    if (RadarIQ_getCertainty(myRadar, &certainty) == RADARIQ_RETURN_VAL_OK)
    {
      sprintf(buffer, "* Certainty level is set to: %u\n\r", certainty);
      Serial.print(buffer);  
    }
    else
    {
      Serial.print("* Error reading certainty level from RadarIQ module\n\r");  
    }
  }
  else
  {
    Serial.print("* Error setting certainty level of RadarIQ module\n\r");  
  }
  
  // Set moving filter option
  if (RadarIQ_setMovingFilter(myRadar, RADARIQ_MOVING_BOTH) == RADARIQ_RETURN_VAL_OK)
  {
    RadarIQMovingFilterMode_t movingFilter;
    if (RadarIQ_getMovingFilter(myRadar, &movingFilter) == RADARIQ_RETURN_VAL_OK)
    {
      sprintf(buffer, "* Moving filter set to: %i\n\r", movingFilter);
      Serial.print(buffer); 
    }
    else
    {
      Serial.print("* Error reading moving filter setting from RadarIQ module\n\r");  
    }
  }
  else
  {
    Serial.print("* Error setting moving filter of RadarIQ module\n\r");  
  }
  
  // Save settings to memory on RadarIQ module
  if (RadarIQ_save(myRadar) == RADARIQ_RETURN_VAL_OK)
  {
    Serial.print("* Saved radar settings successfully\n\r");
  }
  else
  {
    Serial.print("* Failed to save radar settings\n\r");    
  }
  
  // Run a scene calibration
  if (RadarIQ_sceneCalibrate(myRadar) == RADARIQ_RETURN_VAL_OK)
  {
    sprintf(buffer, "* Ran scene calibration successfully\n\r");
    Serial.print(buffer);  
  }
  else
  {
    Serial.print("* Failed to run scene calibration\n\r");    
  }

  // Send start capture command
  RadarIQ_start(myRadar, 10);
}

char buffer[512];

void loop()
{
  // Process the serial data
  RadarIQCommand_t packet = RadarIQ_readSerial(myRadar);
  
  // Check recieved packet type
  switch (packet)
  {
    // Pointcloud frame
    case RADARIQ_CMD_PNT_CLOUD_FRAME:
    {
      RadarIQData_t radarData;
      RadarIQ_getData(myRadar, &radarData);
      
      sprintf(buffer, "\n\r** Data num points = %u\n\r", radarData.pointCloud.numPoints);
      
      for (uint32_t i = 0u; i < radarData.pointCloud.numPoints; i++)
      {
        sprintf(buffer, "*  %u: x = %i, y = %i, z = %i, i = %u, v = %i\n\r", i, 
          radarData.pointCloud.points[i].x, radarData.pointCloud.points[i].y,
          radarData.pointCloud.points[i].z, radarData.pointCloud.points[i].intensity,
          radarData.pointCloud.points[i].velocity);
        Serial.print(buffer);
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
      
      sprintf(buffer, "* Processing: %u, %u, %u, %u, %u, %u, %u\n\r", processing.activeFrameCPULoad,
        processing.interFrameCPULoad, processing.interFrameProcTime,
        processing.transmitOutputTime, processing.interFrameProcMargin,
        processing.interChirpProcMargin, processing.uartTransmitTime);
      Serial.print(buffer);
      
      sprintf(buffer, "* Temperature: %i, %i, %i, %i, %i, %i, %i, %i, %i, %i\n\r", temperatures.sensor0,
        temperatures.sensor1, temperatures.powerManagement, temperatures.rx0,
        temperatures.rx1, temperatures.rx2, temperatures.rx3, temperatures.tx0,
        temperatures.tx1, temperatures.tx2);
      Serial.print(buffer);
      
      break;    
    }
    
    case RADARIQ_CMD_POINTCLOUD_STATS:
    {
      RadarIQPointcloudStats_t pointcloud;
      
      RadarIQ_getPointCloudStats(myRadar, &pointcloud);
      
      sprintf(buffer, "* Pointcloud: %u, %u, %u, %u, %u, %u, %u, %u\n\r", pointcloud.frameAggregatingTime,
        pointcloud.intensitySortTime, pointcloud.nearestNeighboursTime,
        pointcloud.uartTransmitTime, pointcloud.numFilteredPoints,
        pointcloud.numPointsTransmitted, pointcloud.inputPointsTruncated,
        pointcloud.outputPointsTruncated);
      Serial.print(buffer);      

      break;
    }
    
    default:
    {
    }    
  }
}

//------------------------------------------------------------------------------
// Radar Callback Functions
//--------------------------

static void callbackSendRadarData(uint8_t * const buffer, const uint16_t len)
{
    Serial1.write(buffer, len);
}

static RadarIQUartData_t callbackReadSerialData()
{ 
  RadarIQUartData_t ret;
  
  ret.isReadable = (Serial1.available() > 0);
  if (ret.isReadable)
  {
    int data = Serial1.read();
    if (data < 0)
    {
      ret.isReadable = false;
    }
    else
    {
      ret.data = (uint8_t)data;
    }
  }
  
  return ret;
}

static void callbackRadarLog(char * const buffer)
{
  Serial.println(buffer);    
}

//------------------------------------------------------------------------------
// Assertion Handler
//-------------------

void __assert(const char *__func, const char *__file, int __lineno, const char *__sexp)
{
  char buffer[256];
  sprintf(buffer, "ASSERTION FAILED [%s]: File: %s, Line: %i, Func: %s\n\r", __sexp, __file, __lineno, __func);
  Serial.print(buffer);

  delay(100);
  abort();
}
