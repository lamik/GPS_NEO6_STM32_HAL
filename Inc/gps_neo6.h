/*
 * gps_neo6.h
 *
 *  Created on: 6.09.2019
 *      Author: Mateusz Salamon
 *		www.msalamon.pl
 *
 *      Website: https://msalamon.pl/lokalizacja-gps-przy-pomocy-neo6mv2-na-stm32/
 *      GitHub: https://github.com/lamik/GPS_NEO6_STM32_HAL
 *      Contact: mateusz@msalamon.pl
 */
#ifndef gps_neo6_h
#define gps_neo6_h

#define GPS_UART_BUFFER_SIZE 256
#define GPS_WORKING_BUFFER_SIZE 128
typedef struct
{
	//
	// UART stuff
	//
	UART_HandleTypeDef *neo6_huart;
	uint8_t UartBuffer[GPS_UART_BUFFER_SIZE];
	uint8_t UartBufferHead;
	uint8_t UartBufferTail;
	uint8_t UartBufferLines;

	uint8_t WorkingBuffer[GPS_WORKING_BUFFER_SIZE];

	//
	// Time and Date
	//
	uint8_t Hour;
	uint8_t Minute;
	uint8_t Second;
	uint8_t Day;
	uint8_t Month;
	uint8_t Year;

	//
	//	Position
	//
	double 	Latitude;
	char 	LatitudeDirection;
	double 	Longitude;
	char 	LongitudeDirection;
	double 	Altitude;

	//
	//	Speed
	//
	double 	SpeedKnots;
	double 	SpeedKilometers;

	//
	//	Satelites parameters
	//
	uint8_t SatelitesNumber;
	uint8_t Quality; 	// 0 - no Fix, 1 - Fix, 2 - Dif. Fix
	uint8_t FixMode;	// 1 - no Fiz, 2 - 2D, 3 - 3D
	double 	Dop; 		// Dilution of precision
	double 	Hdop;		// Dilution of precision for flat coords
	double 	Vdop;		// Dilution of precision for height

}NEO6_State;

NEO6_State GpsState;

void NEO6_Init(NEO6_State *GpsStateHandler, UART_HandleTypeDef *huart);
void NEO6_ReceiveUartChar(NEO6_State *GpsStateHandler);
void NEO6_Task(NEO6_State *GpsStateHandler);

uint8_t NEO6_IsFix(NEO6_State *GpsStateHandler);

#endif /* gps_neo6_h */
