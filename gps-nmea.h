/******************************************************************************
 * Copyright (C) 2013 Marco Giammarini <http://www.warcomeb.it>
 * 
 * Author(s):
 *  Marco Giammarini <m.giammarini@warcomeb.it>
 *  
 * Project: gps-nmea
 * Package: -
 * Version: 0.0
 * 
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published
 * by the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>
 ******************************************************************************/

/**
 * @file gps-nmea.h
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief GPS NMEA protocol functions definition.
 */

#ifndef __GPS_NMEA_H
#define __GPS_NMEA_H

/* OHIBoard Library (LGPL) */
#include "libohiboard.h"

typedef enum _GpsNmea_Errors
{
	GPSNMEA_ERROR_OK,
	GPSNMEA_ERROR_NO_ACTIVE,
	GPSNMEA_ERROR_JUST_ACTIVE,
	
	GPSNMEA_ERROR_WRONG_CHAR,
	GPSNMEA_ERROR_MSG_TOO_LONG,
	
	GPSNMEA_ERROR_NOT_COMPLIANT,
	GPSNMEA_ERROR_CHECKSUM,
	GPSNMEA_ERROR_MSG_TYPE,
} GpsNmea_Errors;

typedef enum _GpsNmea_MessageType
{
    GPSNMEA_MSG_EMPTY,
    GPSNMEA_MSG_GGA,
    GPSNMEA_MSG_GLL,
    GPSNMEA_MSG_RMC,
    GPSNMEA_MSG_GSV,
    GPSNMEA_MSG_GSA,
    GPSNMEA_MSG_VTG,
    GPSNMEA_MSG_ZDA,
    GPSNEMA_MSG_PMTK001
} GpsNmea_MessageType;

typedef enum _GpsNmea_DeviceType
{
    GPSNMEA_DEVICE_GPS,
    GPSNMEA_DEVICE_GLONASS,
    GPSNMEA_DEVICE_GNSS
} GpsNmea_DeviceType;

union GpsNmea_StatusType
{
    uint8_t status;
    
    struct {
        uint8_t commandReady        :1;
        uint8_t parsingMessage      :1;
        uint8_t notUsed             :6;
    } flags;
} extern GpsNmea_status;

GpsNmea_Errors GpsNmea_init (Uart_DeviceHandle device);
GpsNmea_Errors GpsNmea_enable (void);
GpsNmea_Errors GpsNmea_disable (void);

GpsNmea_Errors GpsNmea_addReceiveChar (void);
GpsNmea_Errors GpsNmea_parseMessage (void);

#endif /* __GPS_NMEA_H */
