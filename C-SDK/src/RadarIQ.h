/*
 * RadarIQ.h
 *
 *  Created on: Aug 26, 2020
 *      Author: Nathan
 */

#ifndef SRC_RADARIQ_H_
#define SRC_RADARIQ_H_

//===============================================================================================//
// GENERAL-PURPOSE MACROS
//===============================================================================================//

#define RADARIQ_RX_BUFFER_SIZE 256u

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

typedef struct radariq_t radariq_t;

//===============================================================================================//
// OBJECTS
//===============================================================================================//

typedef radariq_t* radariqHandle_t;

//===============================================================================================//
// FUNCTIONS
//===============================================================================================//

radariqHandle_t RadarIQ_init();



#endif /* SRC_RADARIQ_H_ */
