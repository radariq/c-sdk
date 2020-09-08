/*
 * main.c
 *
 *  Created on: Aug 26, 2020
 *      Author: Nathan
 */

//------------------------------------------------------------------------------
// Includes
//----------

#include <stdio.h>
#include <stdint.h>
#include <windows.h>

#include "RadarIQ.h"

//------------------------------------------------------------------------------
// Function Prototypes
//---------------------

static void callbackSendRadarData(uint8_t * const buffer, const uint16_t len);
static uint8_t callbackReadSerialData();
static void callbackRadarLog(char * const buffer);

HANDLE open_serial_port(const char * device, uint32_t baud_rate);
int write_port(HANDLE port, uint8_t * buffer, size_t size);
SSIZE_T read_port(HANDLE port, uint8_t * buffer, size_t size);
static HANDLE port = NULL;

//------------------------------------------------------------------------------
// Program Entry Point
//------------------------------------------------------------------------------

int main()
{
	printf("\n*** RIQ-m C-SDK Test Program *** \n\n");

	const char * device = "\\\\.\\COM216";

	const uint32_t baud_rate = 115200;

	port = open_serial_port(device, baud_rate);
	if (port == INVALID_HANDLE_VALUE)
	{
	  return 1;
	}

	RadarIQHandle_t myRadar = RadarIQ_init(callbackSendRadarData, callbackReadSerialData, callbackRadarLog);
	printf("* Created RadarIQ instance, using %u bytes of memory\n", RadarIQ_getMemoryUsage());

	RadarIQ_start(myRadar, 0);

	bool dataReady = false;


	while(!dataReady)
	{
		dataReady = RadarIQ_readSerial(myRadar);
	}
	CloseHandle(port);

	printf("got data\n");

	uint8_t buffer[RADARIQ_PACKET_BUFFER_SIZE];
	uint8_t len = RadarIQ_getDataBuffer(myRadar, buffer);

	for (uint32_t i = 0u; i < len; i++)
	{
		printf("%.2x ", buffer[i]);
	}



	return 0;
}

//------------------------------------------------------------------------------
// Radar Callback Functions
//--------------------------

static void callbackSendRadarData(uint8_t * const buffer, const uint16_t len)
{
	// Debug
	/*
	for (uint16_t i = 0u; i < len; i++)
	{
		printf("%.2x ", buffer[i]);
	}
	printf("\n");
*/
	write_port(port, buffer, len);
}

static uint8_t callbackReadSerialData()
{
	uint8_t rxByte = 0u;

	DWORD dwEventMask;

	SSIZE_T bytesRead = 0u;


	//while (bytesRead == 0u)
	//{
		bytesRead = read_port(port, &rxByte, 1u);
	//}




	//printf("len = %u, %.2x\n", bytesRead, rxByte);

	return rxByte;
}

static void callbackRadarLog(char * const buffer)
{
	printf("* Log: %s\n", buffer);
}

//------------------------------------------------------------------------------
// Windows Serial Functions
//--------------------------

HANDLE open_serial_port(const char * device, uint32_t baud_rate)
{
  HANDLE port = CreateFileA(device, GENERIC_READ | GENERIC_WRITE, 0, NULL,
    OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
  if (port == INVALID_HANDLE_VALUE)
  {
    printf("Failed to open port\n");
    return INVALID_HANDLE_VALUE;
  }

  // Flush away any bytes previously read or written.
  BOOL success = FlushFileBuffers(port);
  if (!success)
  {
    printf("Failed to flush serial port\n");
    CloseHandle(port);
    return INVALID_HANDLE_VALUE;
  }

  // Configure read and write operations to time out after 1s.
  COMMTIMEOUTS timeouts = { 0 };
  timeouts.ReadIntervalTimeout = 50;
  timeouts.ReadTotalTimeoutConstant = 50;
  timeouts.ReadTotalTimeoutMultiplier = 10;
  timeouts.WriteTotalTimeoutConstant = 50;
  timeouts.WriteTotalTimeoutMultiplier = 10;

  success = SetCommTimeouts(port, &timeouts);
  if (!success)
  {
    printf("Failed to set serial timeouts\n");
    CloseHandle(port);
    return INVALID_HANDLE_VALUE;
  }

  DCB state;
  state.DCBlength = sizeof(DCB);
  success = GetCommState(port, &state);
  if (!success)
  {
    printf("Failed to get serial settings\n");
    CloseHandle(port);
    return INVALID_HANDLE_VALUE;
  }

  state.BaudRate = baud_rate;

  success = SetCommState(port, &state);
  if (!success)
  {
    printf("Failed to set serial settings\n");
    CloseHandle(port);
    return INVALID_HANDLE_VALUE;
  }

  success = SetCommMask(port, EV_RXCHAR);
  if (!success)
  {
    printf("Failed to set comm mask\n");
    CloseHandle(port);
    return INVALID_HANDLE_VALUE;
  }

  return port;
}

int write_port(HANDLE port, uint8_t * buffer, size_t size)
{
  DWORD written;
  BOOL success = WriteFile(port, buffer, size, &written, NULL);
  if (!success)
  {
    printf("Failed to write to port\n");
    return -1;
  }
  if (written != size)
  {
    printf("Failed to write all bytes to port\n");
    return -1;
  }
  return 0;
}

SSIZE_T read_port(HANDLE port, uint8_t * buffer, size_t size)
{
  DWORD received;
  BOOL success = ReadFile(port, buffer, size, &received, NULL);
  if (!success)
  {
    printf("Failed to read from port\n");
    return -1;
  }
  return received;
}
