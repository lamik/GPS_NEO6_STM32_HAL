/*
 * gps_neo6.c
 *
 *  Created on: 6.09.2019
 *      Author: Mateusz Salamon
 *		www.msalamon.pl
 *
 *      Website: https://msalamon.pl/lokalizacja-gps-przy-pomocy-neo6mv2-na-stm32/
 *      GitHub: https://github.com/lamik/GPS_NEO6_STM32_HAL
 *      Contact: mateusz@msalamon.pl
 */
#include "main.h"
#include "gps_neo6.h"
#include "string.h" // string manipulations
#include "stdlib.h" // atoi()

volatile uint8_t UartReceivedChar;

/*
 * Behaves like strtok() except that it returns empty tokens also.
 * Found on https://stackoverflow.com/questions/42315585/split-string-into-tokens-in-c-when-there-are-2-delimiters-in-a-row
 */
char* strtoke(char *str, const char *delim)
{
  static char *start = NULL; /* stores string str for consecutive calls */
  char *token = NULL; /* found token */
  /* assign new start in case */
  if (str) start = str;
  /* check whether text to parse left */
  if (!start) return NULL;
  /* remember current start as found token */
  token = start;
  /* find next occurrence of delim */
  start = strpbrk(start, delim);
  /* replace delim with terminator and move start to follower */
  if (start) *start++ = '\0';
  /* done */
  return token;
}

void NEO6_ReceiveUartChar(NEO6_State *GpsStateHandler)
{
	uint8_t TempHead;

	TempHead = (GpsStateHandler->UartBufferHead + 1) % GPS_UART_BUFFER_SIZE;

	if( TempHead == GpsStateHandler->UartBufferTail) // No room for new data
	{
		//
		// Error handle?
		//
	}
	else
	{
		if(UartReceivedChar == 13)
		{
			GpsStateHandler->UartBufferLines++;
			GpsStateHandler->UartBufferHead = TempHead;
			GpsStateHandler->UartBuffer[TempHead] = UartReceivedChar;
		}
		else if((UartReceivedChar == 0) || (UartReceivedChar == 10))
		{
			// Ignore byte 0 and 10 (LF char)
		}
		else
		{
			GpsStateHandler->UartBufferHead = TempHead;
			GpsStateHandler->UartBuffer[TempHead] = UartReceivedChar;
		}

	}

	HAL_UART_Receive_IT(GpsStateHandler->neo6_huart, (uint8_t*)&UartReceivedChar, 1);
}

int NEO6_GetCharFromBuffer(NEO6_State *GpsStateHandler)
{
	if(GpsStateHandler->UartBufferHead == GpsStateHandler->UartBufferTail)
	{
		return -1; // error - no char to return
	}
	GpsStateHandler->UartBufferTail = (GpsStateHandler->UartBufferTail + 1) % GPS_UART_BUFFER_SIZE;

	return GpsStateHandler->UartBuffer[GpsStateHandler->UartBufferTail];
}

int NEO6_GetLineFromBuffer(NEO6_State *GpsStateHandler)
{
	char TempChar;
	char* LinePointer = (char*)GpsStateHandler->WorkingBuffer;
	if(GpsStateHandler->UartBufferLines)
	{
		while((TempChar = NEO6_GetCharFromBuffer(GpsStateHandler)))
		{
			if(TempChar == 13)
			{
				break;
			}
			*LinePointer = TempChar;
			LinePointer++;
		}
		*LinePointer = 0; // end of cstring
		GpsStateHandler->UartBufferLines--; // decrement line counter
	}
	return 0;
}

//
// Recommended minimum specific GPS/Transit data
//
void NEO6_ParseGPRMC(NEO6_State *GpsStateHandler)
{
	// eg1. $GPRMC,081836,A,3751.65,S,14507.36,E,000.0,360.0,130998,011.3,E*62
	// eg2. $GPRMC,225446,A,4916.45,N,12311.12,W,000.5,054.7,191194,020.3,E*68

	char *ParsePoiner;
	uint32_t Temp;

	// Time of FIX
	ParsePoiner = strtoke(NULL, ",");
	if(strlen(ParsePoiner) > 0)
	{
		Temp = atoi(ParsePoiner);
		GpsStateHandler->Second = Temp % 100;
		GpsStateHandler->Minute = (Temp / 100) % 100;
		GpsStateHandler->Hour = (Temp / 10000) % 100;
	}
	// Navigation receiver warning A = OK, V = warning
	ParsePoiner = strtoke(NULL, ",");
	// Latitude
	ParsePoiner = strtoke(NULL, ",");
	if(strlen(ParsePoiner) > 0)
	{
		GpsStateHandler->Latitude = atof(ParsePoiner);
	}
	// Latitude Direction
	ParsePoiner = strtoke(NULL, ",");
	if(strlen(ParsePoiner) > 0)
	{
		GpsStateHandler->LatitudeDirection = *ParsePoiner;
	}
	// Longnitude
	ParsePoiner = strtoke(NULL, ",");
	if(strlen(ParsePoiner) > 0)
	{
		GpsStateHandler->Longitude = atof(ParsePoiner);
	}
	// Longnitude Direction
	ParsePoiner = strtoke(NULL, ",");
	if(strlen(ParsePoiner) > 0)
	{
		GpsStateHandler->LongitudeDirection = *ParsePoiner;
	}
	// Speed over ground, Knots
	ParsePoiner = strtoke(NULL, ",");
	// Course Made Good, True
	ParsePoiner = strtoke(NULL, ",");
	// Date of fix
	ParsePoiner = strtoke(NULL, ",");
	if(strlen(ParsePoiner) > 0)
	{
		Temp = atoi(ParsePoiner);
		GpsStateHandler->Year = Temp % 100;
		GpsStateHandler->Month = (Temp / 100) % 100;
		GpsStateHandler->Day = (Temp / 10000) % 100;
	}
}

//
//	Track Made Good and Ground Speed.
//
void NEO6_ParseGPVTG(NEO6_State *GpsStateHandler)
{
	// eg1. $GPVTG,360.0,T,348.7,M,000.0,N,000.0,K*43
	// eg2. $GPVTG,054.7,T,034.4,M,005.5,N,010.2,K

	char *ParsePoiner;

	// True track made good
	ParsePoiner = strtoke(NULL, ",");
	ParsePoiner = strtoke(NULL, ",");
	// Magnetic track made good
	ParsePoiner = strtoke(NULL, ",");
	ParsePoiner = strtoke(NULL, ",");
	// Ground speed, knots
	ParsePoiner = strtoke(NULL, ",");
	if(strlen(ParsePoiner) > 0)
	{
		GpsStateHandler->SpeedKnots = atof(ParsePoiner);
	}
	ParsePoiner = strtoke(NULL, ",");
	// Ground speed, Kilometers per hour
	ParsePoiner = strtoke(NULL, ",");
	if(strlen(ParsePoiner) > 0)
	{
		GpsStateHandler->SpeedKilometers = atof(ParsePoiner);
	}
}

//
//	Global Positioning System Fix Data
//
void NEO6_ParseGPGGA(NEO6_State *GpsStateHandler)
{
	// eg. $GPGGA,212846.00,5025.81511,N,01639.92090,E,1,04,4.72,281.1,M,42.0,M,,*5F

	char *ParsePoiner;

	// UTC of Position
	ParsePoiner = strtoke(NULL, ",");
	// Latitude
	ParsePoiner = strtoke(NULL, ",");
	// N or S
	ParsePoiner = strtoke(NULL, ",");
	// Longitude
	ParsePoiner = strtoke(NULL, ",");
	// E or W
	ParsePoiner = strtoke(NULL, ",");
	// GPS quality indicator (0=invalid; 1=GPS fix; 2=Diff. GPS fix)
	ParsePoiner = strtoke(NULL, ",");
	if(strlen(ParsePoiner) > 0)
	{
		GpsStateHandler->Quality = atoi(ParsePoiner);
	}
	// Number of satellites in use [not those in view]
	ParsePoiner = strtoke(NULL, ",");
	if(strlen(ParsePoiner) > 0)
	{
		GpsStateHandler->SatelitesNumber = atoi(ParsePoiner);
	}
	// Horizontal dilution of position
	ParsePoiner = strtoke(NULL, ",");
	// Antenna altitude above/below mean sea level (geoid)
	ParsePoiner = strtoke(NULL, ",");
	if(strlen(ParsePoiner) > 0)
	{
		GpsStateHandler->Altitude = atof(ParsePoiner);
	}
}

//
//	GPS DOP and active satellites
//
void NEO6_ParseGPGSA(NEO6_State *GpsStateHandler)
{
	// eg1. $GPGSA,A,3,,,,,,16,18,,22,24,,,3.6,2.1,2.2*3C
	// eg2. $GPGSA,A,3,19,28,14,18,27,22,31,39,,,,,1.7,1.0,1.3*35

	char *ParsePoiner;

	// Mode
	ParsePoiner = strtoke(NULL, ",");
	// 2D/3D Fix
	ParsePoiner = strtoke(NULL, ",");
	if(strlen(ParsePoiner) > 0)
	{
		GpsStateHandler->FixMode = atoi(ParsePoiner);
	}
	// IDs of SVs used in position fix (null for unused fields)
	for(uint8_t i=0; i < 12; i++)
	{
		ParsePoiner = strtoke(NULL, ",");
	}
	// PDOP
	ParsePoiner = strtoke(NULL, ",");
	if(strlen(ParsePoiner) > 0)
	{
		GpsStateHandler->Dop = atof(ParsePoiner);
	}
	// HDOP
	ParsePoiner = strtoke(NULL, ",");
	if(strlen(ParsePoiner) > 0)
	{
		GpsStateHandler->Hdop = atof(ParsePoiner);
	}
	// VDOP
	ParsePoiner = strtoke(NULL, ",");
	if(strlen(ParsePoiner) > 0)
	{
		GpsStateHandler->Vdop = atof(ParsePoiner);
	}
}

//
// GPS Satellites in view
//
void NEO6_ParseGPGSV(NEO6_State *GpsStateHandler)
{
	// eg. $GPGSV,3,1,11,03,03,111,00,04,15,270,00,06,01,010,00,13,06,292,00*74
	//     $GPGSV,3,2,11,14,25,170,00,16,57,208,39,18,67,296,40,19,40,246,00*74
	//     $GPGSV,3,3,11,22,42,067,42,24,14,311,43,27,05,244,00,,,,*4D

	// Todo
	// Do I need to check satellites data?
}

//
// Geographic Position, Latitude / Longitude and time.
//
void NEO6_ParseGPGLL(NEO6_State *GpsStateHandler)
{
	// It's done with other NMEA messages.
	// Could be good for minimal system for LoRa device
}


void NEO6_ParseLine(NEO6_State *GpsStateHandler)
{
	//
	// Nice website with NMEA commuincates description
	//	http://aprs.gids.nl/nmea
	//

	// Header
	char* ParsePoiner = strtoke((char*)GpsStateHandler->WorkingBuffer, ",");

	if(strcmp(ParsePoiner, "$GPRMC") == 0) NEO6_ParseGPRMC(GpsStateHandler);
	else if(strcmp(ParsePoiner, "$GPVTG") == 0) NEO6_ParseGPVTG(GpsStateHandler);
	else if(strcmp(ParsePoiner, "$GPGGA") == 0) NEO6_ParseGPGGA(GpsStateHandler);
	else if(strcmp(ParsePoiner, "$GPGSA") == 0) NEO6_ParseGPGSA(GpsStateHandler);
//	else if(strcmp(ParsePoiner, "$GPGSV") == 0) NEO6_ParseGPGSV(GpsStateHandler);
//	else if(strcmp(ParsePoiner, "$GPGLL") == 0) NEO6_ParseGPGLL(GpsStateHandler);
}

uint8_t NEO6_IsFix(NEO6_State *GpsStateHandler)
{
	return GpsStateHandler->Quality;
}

void NEO6_Task(NEO6_State *GpsStateHandler)
{
	if(GpsStateHandler->UartBufferLines)
	{
		NEO6_GetLineFromBuffer(GpsStateHandler);
		NEO6_ParseLine(GpsStateHandler);
	}
}

void NEO6_Init(NEO6_State *GpsStateHandler, UART_HandleTypeDef *huart)
{
	GpsStateHandler->neo6_huart = huart;
	GpsStateHandler->UartBufferHead = 0;
	GpsStateHandler->UartBufferTail = 0;
	GpsStateHandler->UartBufferLines = 0;

	GpsStateHandler->Hour = 0;
	GpsStateHandler->Minute = 0;
	GpsStateHandler->Second = 0;
	GpsStateHandler->Day = 0;
	GpsStateHandler->Month = 0;
	GpsStateHandler->Year = 0;

	GpsStateHandler->Latitude = 0;
	GpsStateHandler->LatitudeDirection = '0';
	GpsStateHandler->Longitude = 0;
	GpsStateHandler->LongitudeDirection = '0';

	GpsStateHandler->SpeedKilometers = 0;
	GpsStateHandler->SpeedKnots = 0;

	GpsStateHandler->SatelitesNumber = 0;
	GpsStateHandler->Quality = 0;
	GpsStateHandler->Dop = 0;
	GpsStateHandler->Hdop = 0;
	GpsStateHandler->Vdop = 0;

	HAL_UART_Receive_IT(GpsStateHandler->neo6_huart, (uint8_t*)&UartReceivedChar, 1);
}
