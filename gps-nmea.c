/******************************************************************************
 * Copyright (C) 2013-21014 Marco Giammarini <http://www.warcomeb.it>
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
 * @file gps-nmea.c
 * @author Marco Giammarini <m.giammarini@warcomeb.it>
 * @brief GPS NMEA protocol functions implementation.
 */
#include "gps-nmea.h"

#define GPSNMEA_ACTIVE                     1
#define GPSNMEA_NO_ACTIVE                  0
#define GPSNMEA_BAUDRATE                   9600

#define GPSNMEA_START                      '$'
#define GPSNMEA_STOP                       '*'
#define GPSNMEA_SEPARATOR                  ','
#define GPSNMEA_DECIMAL                    '.'
#define GPSNMEA_END                        "\r\n"
#define GPSNMEA_END1                       '\r'
#define GPSNMEA_END2                       '\n'

#define GPSNMEA_MSG_MAX_LENGTH             80

#define GPSNMEA_RX_BUFFER_MASK             0x7F
#define GPSNMEA_TX_BUFFER_MASK             0x7F

static uint8_t GpsNmea_rxBufferIndex;
static uint8_t GpsNmea_txBufferIndex;

static uint8_t GpsNmea_rxBuffer[GPSNMEA_RX_BUFFER_MASK + 1];

static Uart_DeviceHandle GpsNmea_device;
static uint8_t GpsNmea_active = GPSNMEA_NO_ACTIVE;

#define GPSENMEA_POS_STOP(n)               (n-5)
#define GPSENMEA_POS_END1(n)               (n-2)
#define GPSENMEA_POS_END2(n)               (n-1)

void GpsNmea_init (Uart_DeviceHandle device)
{
	/* TODO: Init periferica! */
	
	GpsNmea_rxBufferIndex = 0;
	GpsNmea_txBufferIndex = 0;
}

/**
 * Enable communication port for GPS device.
 * 
 * @return Return the status of the operation by an element of GpsNmea_Errors
 */
GpsNmea_Errors GpsNmea_enable (void)
{
    if (GpsNmea_active == GPSNMEA_NO_ACTIVE)
    {
        Uart_enable(GpsNmea_device);
        GpsNmea_active = GPSNMEA_ACTIVE;
        return GPSNMEA_ERROR_OK;
    }
    else
    {
        return GPSNMEA_ERROR_JUST_ACTIVE;
    }
}

/**
 * Disable communication port for GPS device.
 * 
 * @return Return the status of the operation by an element of GpsNmea_Errors
 */
GpsNmea_Errors GpsNmea_disable (void)
{
    if (GpsNmea_active == GPSNMEA_ACTIVE)
    {
        Uart_disable(GpsNmea_device);
        GpsNmea_active = GPSNMEA_NO_ACTIVE;
        return GPSNMEA_ERROR_OK;
    }
    else
    {
        return GPSNMEA_ERROR_NO_ACTIVE;
    }
}

static uint8_t GpsNmea_computeChecksum (const uint8_t* data, uint8_t start, uint8_t length)
{
    uint8_t checksum = 0;
    uint8_t i;

     for (i = start; i < length; ++i)
     {
         checksum = checksum ^ data;
         data++;
     }

     return checksum;
}

static GpsNmea_MessageType GpsNmea_getReceiveMessageType (void)
{
    
}

/**
 * 
 * @return Return the status of the operation by an element of GpsNmea_Errors
 */
GpsNmea_Errors GpsNmea_addReceiveChar (void)
{
    char c;
    
    Uart_getChar(GpsNmea_device,&c);
    
    if (IS_DIGIT(c) || IS_UPPERLETTER(c) || IS_UPPERLETTER(c) ||
        (c == GPSNMEA_START) || (c == GPSNMEA_SEPARATOR) ||
        (c == GPSNMEA_DECIMAL) || (c == GPSNMEA_STOP) || (c == GPSNMEA_END1))
    {
    	GpsNmea_rxBuffer[GpsNmea_rxBufferIndex] = c;
    	GpsNmea_rxBufferIndex++;
    }
    else if (c == GPSNMEA_END2)
    {
    	GpsNmea_rxBuffer[GpsNmea_rxBufferIndex] = c;
        GpsNmea_rxBufferIndex++;
    	GpsNmea_status.flags.commandReady = 1;
    }
    else
    {
    	GpsNmea_rxBufferIndex = 0;
    	return GPSNMEA_ERROR_WRONG_CHAR;
    }
    
    if (!GpsNmea_status.flags.commandReady && GpsNmea_rxBufferIndex > GPSNMEA_MSG_MAX_LENGTH)
    {
        GpsNmea_rxBufferIndex = 0;
        return GPSNMEA_ERROR_MSG_TOO_LONG;
    }
    
    return GPSNMEA_ERROR_OK;
}

/**
 * 
 * @return Return the status of the operation by an element of GpsNmea_Errors
 */
GpsNmea_Errors GpsNmea_parseMessage (void)
{
    static uint8_t rxChecksum = 0;
    static uint8_t computeChecksum = 0;
    
    static uint8_t messageLength = 0;

    /* Reset buffer index indicator */
    messageLength = GpsNmea_rxBufferIndex;
    GpsNmea_rxBufferIndex = 0;
    
    /* Control start and end chars of message */
    if ((GpsNmea_rxMessage[0] == GPSNMEA_START) && 
        (GpsNmea_rxMessage[GPSENMEA_POS_STOP(messageLength)] == GPSNMEA_STOP) &&
        (GpsNmea_rxMessage[GPSENMEA_POS_END1(messageLength)] == GPSNMEA_END1) &&
        (GpsNmea_rxMessage[GPSENMEA_POS_END2(messageLength)] == GPSNMEA_END2))
    {
        /* Control checksum */
        xtu8(&GpsNmea_rxMessage[messageLength-4],&rxChecksum,2);
        computeChecksum = GpsNmea_computeChecksum(&GpsNmea_rxMessage[1],1,GPSENMEA_POS_STOP(messageLength));
        if (computeChecksum != rxChecksum)
            return GPSNMEA_ERROR_CHECKSUM; /* Checksum mismatch */

        return GPSNMEA_ERROR_OK;
    }
    else
    {
        return GPSNMEA_ERROR_NOT_COMPLIANT;
    }
}
