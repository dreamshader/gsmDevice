//
// ************************************************************************
//
// gsmDevice (c) 2017 Dirk Schanz aka dreamshader
//    supports a subset of GSM AT commands
//
// ************************************************************************
//
// At this point a "thank you very much" to all the authors sharing
// their work, knowledge and all the very useful stuff.
// You did a great job!
//
// ************************************************************************
//
//  This program is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
// ************************************************************************
//
// -------- brief description ---------------------------------------------
//
// This class library can be used as an interface to a gsm device attached
// on a serial port.
// Originally it was intended to support gsm modules based on A6/A7 chip
// by AI Thinker that are connected to an Arduino.
// While coding this lib the idea came up to support a linux PC as a
// host, too. So now you may use it in the Arduino IDE as well as as
// a Linux library.
// Currently the library supports a subset of the GSM 07.07 AT command 
// set of ETSI. 
// Using it is quite simpel. You create an instance of class gsmDevice
// and call the several methods.
// 
// At this time, the following AT commands are supported:
//
// - ATV     - Set result code format mode
// - ATE     - Enable command echo
// - ATI     - Request manufacturer specific information about the TA
// - AT+CMGF - Select SMS message format
// - AT+COPS - Operator selects
// - AT+CREG - Network registration
// - AT+CSQ  - Signal quality
// - AT+CPOL - Preferred operator list
// - AT+CIMI - Request international mobile subscriber identity
// - AT+EGMR - Read and write IMEI
// - AT+CGMR - Request revision identification
//
// -------- Todo list -----------------------------------------------------
//
// -- check and return results of AT+COPS
// -- check and return results of AT+CPOL
//
//
// -------- Known bugs ----------------------------------------------------
//
//
// -------- History -------------------------------------------------------
//
// - 05/22/17 initial version
// - 06/05/17 pre0.1 release on github 
//            (https://github.com/dreamshader/gsmDevice)
//
// - 06/08/17 
// -- added check and return results of ATI
// -- added check and return results of AT+CSQ
// -- added check and return results of AT+CIMI
// -- added check and return results of AT+EGMR
// -- added check and return results of AT+CGMR
// -- added check and return results of AT+CREG
//
//
//
// ************************************************************************
//

// 
// --------------------------- INCLUDE SECTION ----------------------------
// 


#include <stdio.h>
#include <stdint.h>

#ifndef linux
#include <Arduino.h>
#include <SoftwareSerial.h>
#else // linux
#include <errno.h>
#include <fcntl.h> 
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <sys/signal.h>
#include <sys/types.h>
#include <stdbool.h>
#include <stdint.h>
#include <time.h>
#include <sys/time.h>
#endif // linux


#include "gsmDevice.h"


// 
// ----------------------------- GLOBAL STUFF -----------------------------
//

#ifdef linux

//
// ----------------------------------------------------------------------
//

// ------------------------------------------------------------------------
// int setupSerial(int fd, int speed)
//
// setup serial port 
// ------------------------------------------------------------------------
int gsmDevice::setupSerial(int fd, int speed)
{
    struct termios tty;
    INT16 retVal = -1;

    if (tcgetattr(fd, &tty) < 0) 
    {
        retVal = GSMDEVICE_E_SETUP;
        _lastErrno = errno;
    }
    else
    {
        cfsetospeed(&tty, (speed_t)speed);
        cfsetispeed(&tty, (speed_t)speed);

        tty.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;         /* 8-bit characters */
        tty.c_cflag &= ~PARENB;     /* no parity bit */
        tty.c_cflag &= ~CSTOPB;     /* only need 1 stop bit */
        tty.c_cflag &= ~CRTSCTS;    /* no hardware flowcontrol */

        /* setup for non-canonical mode */
        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
        tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
        tty.c_oflag &= ~OPOST;

        /* fetch bytes as they become available */
        tty.c_cc[VMIN] = 1;
        tty.c_cc[VTIME] = 1;

        if (tcsetattr(fd, TCSANOW, &tty) != 0) 
        {
            retVal = GSMDEVICE_E_SETUP;
            _lastErrno = errno;
        }
        else
        {
            retVal = GSMDEVICE_SUCCESS;
        }
    }

  return( _lastError = retVal );
}

// ----------------------------------------------------------------------
// void setSerialMin(int fd, int mcount)
//
// set minimal input charcters
// ----------------------------------------------------------------------
void gsmDevice::setSerialMin(int fd, int mcount)
{
    struct termios tty;

    if (tcgetattr(fd, &tty) < 0) 
    {
        _lastErrno = errno;
    }
    else
    {
        tty.c_cc[VMIN] = mcount ? 1 : 0;
        tty.c_cc[VTIME] = 5;        /* half second timer */

        if (tcsetattr(fd, TCSANOW, &tty) < 0)
        {
            _lastErrno = errno;
        }
    }
}



long gsmDevice::millis()
{
    long milliseconds;

    struct timeval te; 
    gettimeofday(&te, NULL); // get current time
    milliseconds = (te.tv_sec - _startTime) * 1000 + te.tv_usec/1000; 
//    printf("milliseconds: %ld\n", milliseconds);
    return( milliseconds );
}

void gsmDevice::delay(long msec)
{
    long firstCall;
    bool done;

    firstCall = millis();
    done = false;
    do
    {
        if( millis() >= (firstCall + msec) )
        {
            done = true;
        }
    } while( !done );
}


String::String()
{
    pData = NULL;
}

String::String( const char* p )
{
    if( p != NULL )
    {
        pData = strdup(p);
    }
}

String::String( int v )
{
    char *_tmp;
    if( (_tmp = (char*) malloc(128)) != NULL )
    {
        sprintf(_tmp, "%d", v);
        pData = strdup(_tmp);
        free( _tmp );
    }
    
}

int String::length()
{
    int len = 0;

    if( pData != NULL )
    {
        len = strlen(pData);
    }
    return( len );
}

int String::trim()
{
    if( pData != NULL )
    {
        int done = 0;
        int startIdx;
        char *_tmp;

        for( int i = strlen(pData) - 1; !done && i > 0; i-- )
        {
            switch(pData[i])
            {
                case ' ':
                case '\t':
                    pData[i] = '\0';
                    break;
                default:
                    done = 1;
                    break;
            }
        }

        for( startIdx = 0, done = 0; !done && startIdx < strlen(pData); startIdx++ )
        {
            switch(pData[startIdx])
            {
                case ' ':
                case '\t':
                    break;
                default:
                    done = 1;
                    if( startIdx )
                    {
                        startIdx--;
                    }
                    break;
            }
        }

        free( pData );
        pData = strdup( &pData[startIdx] );

    }
    return( 0 );
}

char *String::c_str()
{
    return( pData );
}


int String::indexOf(String p)
{
    if( pData != NULL && p.c_str() != NULL )
    {
        char *_tmp;
        // char *strcasestr(const char *haystack, const char *needle);
        if( (_tmp = strstr(pData, p.c_str())) != NULL )
        {
            return( _tmp - pData );
        }
        else
        {
            return( -1 );
        }
    }
    return( -1 );
}

String String::operator+=(String p)
{
    char *_tmp;
    if( p.c_str() != NULL )
    {
        if( (_tmp = strdup(p.c_str())) != NULL )
        {
            if( pData != NULL )
            {
                pData = strcat(pData, p.c_str());
            }
            else
            {
                pData = _tmp;
            }
        }
    }
}


String String::operator+=(const char *p)
{
    char *_tmp;
    if( p != NULL )
    {
        if( (_tmp = strdup(p)) != NULL )
        {
            if( pData != NULL )
            {
                pData = strcat(pData, p);
            }
            else
            {
                pData = _tmp;
            }
        }
    }
}

String String::operator+(String p)
{
    return( operator+=(p) );
}

String String::operator+(const char *p)
{
    return( operator+=(p) );
}


#endif // linux



// 
// ---------------------------- PUBLIC METHODS ----------------------------
//

// ************************************************************************
// Create a gsmDevice instance 
//   - initial object creation
//
// Expected arguments:
// - none
//
// Returns:
// - nothing
//
// ************************************************************************
//
gsmDevice::gsmDevice()
{
    majorRelease = 0;
    minorRelease = 1;

#ifdef linux
    time(&_startTime);
    _lastErrno = 0;
#endif // linux

    _gsmDeviceType   = UNKNOWN_GSM_DEVICE;
    _cmdPortType     = nodev;
    _dataPortType    = nodev;
    _devStatus       = created;
    _cmdPortSpeed    = NO_SPEED;
    _cmdPortTimeout  = NO_TIMEOUT;
    _dataPortSpeed   = NO_SPEED;
    _dataPortTimeout = NO_TIMEOUT;
}


// ************************************************************************
// Check a gsmDevice instance 
//   - check for connectivity to attached gsm device
//
// Expected arguments:
// - gsmDevType devType   device type, e.g. AI_A6
//
// Returns an INT16 as status code:
// - GSMDEVICE_SUCCESS on succes, or
// depending of the failure that occurred 
// - GSMDEVICE_E_INIT      instance is not initialized
// - GSMDEVICE_E_RESPONSE  gsm device (e.g. gsm modem) did not respond
// - GSMDEVICE_E_SUPPORTED not yet supported (e.g. stream device)
// - GSMDEVICE_E_DEV_TYPE  devType is not a valid gsm device type
//
// ************************************************************************
//
INT16 gsmDevice::begin(gsmDevType devType)
{
    INT16 retVal;

    if( _cmdPortType  == nodev ||
        _devStatus    == created )
    {
        retVal = GSMDEVICE_E_INIT;
    }
    else
    {
        if( devType != AI_A6 && devType != AI_A7 )
        {
            retVal = GSMDEVICE_E_DEV_TYPE;
        }
        else
        {
            _gsmDeviceType = devType;
            retVal = syncWithResponse( GSMDEV_SYNC_CMD, GSMDEV_SYNC_RSP );
        }
    }

    return( _lastError = retVal );
}

#ifndef linux

// ************************************************************************
// Initialize a gsmDevice instance 
//   - this is used for further use
//
// Expected arguments:
// - INT16 rxPin
//   INT16 txPin
//   INT32 speed
//   INT32 timeout
//
// Returns an INT16 as status code:
// - GSMDEVICE_SUCCESS on succes, or
// depending of the failure that occurred 
// - GSMDEVICE_E_P_RXPIN  invalid value for rxPin
// - GSMDEVICE_E_P_TXPIN  invalid value for txPin
// - GSMDEVICE_E_P_SPEED  invalid baudrate
// - GSMDEVICE_E_P_TMOUT  invalid timeout value
//
// ************************************************************************
//
INT16 gsmDevice::init(INT16 rxPin, INT16 txPin, INT32 speed, 
                      INT32 timeout)
{
    INT16 retVal;

    if( rxPin <= NULL_PIN )
    {
        retVal = GSMDEVICE_E_P_RXPIN;
    }
    else
    {
        if( txPin <= NULL_PIN )
        {
            retVal = GSMDEVICE_E_P_TXPIN;
        }
        else
        {
            if( speed < NO_SPEED || speed > GSMDEVICE_SWSERIAL_MAX_BAUD )
            {
                retVal = GSMDEVICE_E_P_SPEED;
            }
            else
            {
                if( timeout < NO_TIMEOUT )
                {
                    retVal = GSMDEVICE_E_P_TMOUT;
                }
                else
                {
                    _cmdPort._sw    = new SoftwareSerial(rxPin, txPin, false);
                    _cmdPortType    = swSerial;
                    _ownStream      = true;
                    _cmdPortSpeed   = speed;
                    _cmdPortTimeout = timeout;
                    _devStatus      = initialized;

                    if( speed != NO_SPEED )
                    {
                        _cmdPort._sw->begin(speed);
                    }
                    else
                    {
                        _cmdPort._sw->begin( GSMDEVICE_DEF_CMD_SPEED );
                    }

                    if( timeout != NO_TIMEOUT )
                    {
                        _cmdPort._sw->setTimeout(timeout);
                    }

                    retVal = GSMDEVICE_SUCCESS;
                }
            }
        }
    }

    return( _lastError = retVal );
}

// ************************************************************************
// Initialize a gsmDevice instance 
//   - this is used for further use
//
// Expected arguments:
// - INT16 serialno
//   INT32 speed
//   INT32 timeout
//
// Returns an INT16 as status code:
// - GSMDEVICE_SUCCESS on succes, or
// depending of the failure that occurred 
// - GSMDEVICE_E_P_SERIAL  invalid value for # of hw serial
// - GSMDEVICE_E_P_SPEED   invalid baudrate
// - GSMDEVICE_E_P_TMOUT   invalid timeout value
// - GSMDEVICE_E_SUPPORTED not yet supported (e.g. other than serial 0)
//
// ************************************************************************
INT16 gsmDevice::init(INT16 serialNo, INT32 speed, INT32 timeout)
{
    INT16 retVal;

    if( serialNo <= NO_SERIAL )
    {
        retVal = GSMDEVICE_E_P_SERIAL;
    }
    else
    {
        if( speed < NO_SPEED )
        {
            retVal = GSMDEVICE_E_P_SPEED;
        }
        else
        {
            if( timeout < NO_TIMEOUT )
            {
                retVal = GSMDEVICE_E_P_TMOUT;
            }
            else
            {
                if( serialNo == NO_SERIAL )
                {
                    _cmdPort._hw    = &Serial;
                    _cmdPortType    = hwSerial;
                    _ownStream      = false;
                    _cmdPortSpeed   = speed;
                    _cmdPortTimeout = timeout;
                    _devStatus      = initialized;
               

                    if( speed != NO_SPEED )
                    {
                        _cmdPort._hw->begin(speed);
                    }
                    else
                    {
                        _cmdPort._hw->begin( GSMDEVICE_DEF_CMD_SPEED );
                    }

                    if( timeout != NO_TIMEOUT )
                    {
                        _cmdPort._hw->setTimeout(timeout);
                    }

                    retVal = GSMDEVICE_SUCCESS;
                }
                else
                {
                    retVal = GSMDEVICE_E_SUPPORTED;
                }
            }
        }
    }

    return( _lastError = retVal );
}

// ************************************************************************
// Initialize a gsmDevice instance 
//   - this is used for further use
//
// Expected arguments:
// - STREAM output
//   INT32 timeout
//
// Returns an INT16 as status code:
// - GSMDEVICE_SUCCESS on succes, or
// depending of the failure that occurred 
// - GSMDEVICE_E_P_STREAM invalid stream supplied
// - GSMDEVICE_E_P_TMOUT  invalid timeout value
//
// ************************************************************************
INT16 gsmDevice::init(STREAM *output, INT32 timeout)
{
    INT16 retVal;

    if( output == NULL_STREAM )
    {
        retVal = GSMDEVICE_E_P_STREAM;
    }
    else
    {
        if( timeout < NO_TIMEOUT )
        {
            retVal = GSMDEVICE_E_P_TMOUT;
        }
        else
        {
            _cmdPort._str    = output;
            _cmdPortType    = streamType;
            _ownStream      = false;
            _cmdPortTimeout = timeout;
            _devStatus      = initialized;

            if( timeout != NO_TIMEOUT )
            {
                _cmdPort._str->setTimeout(timeout);
            }

            retVal = GSMDEVICE_SUCCESS;
        }
    }

    return( _lastError = retVal );
}

// ************************************************************************
// Initialize a gsmDevice instance 
//   - this is used for further use
//
// Expected arguments:
// - SW_SERIAL output
//   INT32 timeout
//
// Returns an INT16 as status code:
// - GSMDEVICE_SUCCESS on succes, or
// depending of the failure that occurred 
// - GSMDEVICE_E_P_STREAM invalid stream supplied
// - GSMDEVICE_E_P_TMOUT  invalid timeout value
//
// ************************************************************************
INT16 gsmDevice::init(SW_SERIAL *output, INT32 timeout)
{
    INT16 retVal;

    if( output == NULL_STREAM )
    {
        retVal = GSMDEVICE_E_P_STREAM;
    }
    else
    {
        if( timeout < NO_TIMEOUT )
        {
            retVal = GSMDEVICE_E_P_TMOUT;
        }
        else
        {
            _cmdPort._sw    = output;
            _cmdPortType    = swSerial;
            _ownStream      = false;
            _cmdPortTimeout = timeout;
            _devStatus      = initialized;

            if( timeout != NO_TIMEOUT )
            {
                _cmdPort._sw->setTimeout(timeout);
            }

            retVal = GSMDEVICE_SUCCESS;
        }
    }

    return( _lastError = retVal );
}


// ************************************************************************
// Initialize a gsmDevice instance 
//   - this is used for further use
//
// Expected arguments:
// - HW_SERIAL output
//   INT32 timeout
//
// Returns an INT16 as status code:
// - GSMDEVICE_SUCCESS on succes, or
// depending of the failure that occurred 
// - GSMDEVICE_E_P_STREAM invalid stream supplied
// - GSMDEVICE_E_P_TMOUT  invalid timeout value
//
// ************************************************************************
INT16 gsmDevice::init(HW_SERIAL *output, INT32 timeout)
{
    INT16 retVal;

    if( output == NULL_STREAM )
    {
        retVal = GSMDEVICE_E_P_STREAM;
    }
    else
    {
        if( timeout < NO_TIMEOUT )
        {
            retVal = GSMDEVICE_E_P_TMOUT;
        }
        else
        {
            _cmdPort._hw    = output;
            _cmdPortType    = hwSerial;
            _ownStream      = false;
            _cmdPortTimeout = timeout;
            _devStatus      = initialized;

            if( timeout != NO_TIMEOUT )
            {
                _cmdPort._hw->setTimeout(timeout);
            }

            retVal = GSMDEVICE_SUCCESS;
        }
    }

    return( _lastError = retVal );
}

#else // linux platform

// ************************************************************************
// Initialize a gsmDevice instance 
//   - this is used for further use
//
// Expected arguments:
// - DEVICENAME deviceName
// - INT32 speed
// - INT32 timeout
//
// Returns an INT16 as status code:
// - GSMDEVICE_SUCCESS on succes, or
// depending of the failure that occurred 
// - GSMDEVICE_E_P_SERIAL invalid value for # of hw serial
// - GSMDEVICE_E_P_SPEED  invalid baudrate
// - GSMDEVICE_E_P_TMOUT  invalid timeout value
//
// ************************************************************************
INT16 gsmDevice::init(DEVICENAME deviceName, INT32 speed, INT32 timeout)
{
    INT16 retVal;

    if( deviceName.length() == 0 ) // <= NO_SERIAL )
    {
        retVal = GSMDEVICE_E_P_SERIAL;
    }
    else
    {
        if( speed < NO_SPEED )
        {
            retVal = GSMDEVICE_E_P_SPEED;
        }
        else
        {
            if( timeout < NO_TIMEOUT )
            {
                retVal = GSMDEVICE_E_P_TMOUT;
            }
            else
            {

                if( (_cmdPort._dev   =  open( deviceName.c_str(), O_RDWR | O_NOCTTY | O_SYNC)) < 0 )
                {
                    _lastErrno = errno;
                    retVal = GSMDEVICE_E_OPEN;
                }
                else
                {
                    if( speed != NO_SPEED )
                    {
                        retVal = setupSerial(_cmdPort._dev, speed);
                    }
                    else
                    {
                        retVal = setupSerial(_cmdPort._dev, GSMDEVICE_DEF_CMD_SPEED);
                    }

                    if( retVal == GSMDEVICE_SUCCESS )
                    {
                        setSerialMin(_cmdPort._dev, 0);

                        _cmdPortType    = linuxDevice;
                        _ownStream      = false;
                        _cmdPortSpeed   = speed;
                        _cmdPortTimeout = timeout;
                        _devStatus      = initialized;
                    }
                }
            }
        }
    }

    return( _lastError = retVal );
}

#endif // linux

//
// ************************************************************************
// Delete gsmDevice instance 
//
// Expected arguments:
// - none
//
// Returns:
// - nothing
//
// ************************************************************************
gsmDevice::~gsmDevice() 
{
#ifndef linux
    if( _ownStream )
    {
        delete _cmdPort._sw;
    }
#else
    if( _cmdPort._dev > 0 )
    {
        close( _cmdPort._dev );
    }
#endif // linux
}

//
// ************************************************************************
//
// flush gsmDevice
//   - readoff all input from attached gsm device
//
// Expected arguments:
// - none
// 
// Returns an INT16 as status code:
// - GSMDEVICE_SUCCESS on succes, or
// depending of the failure that occurred 
// - GSMDEVICE_E_INIT      instance is not initialized
// - GSMDEVICE_E_SUPPORTED not yet supported (e.g. stream device)
//
// ************************************************************************
//
INT16 gsmDevice::inputFlush()
{
    INT16 retVal;

    if( _cmdPortType  == nodev ||
        _devStatus    == created )
    {
        retVal = GSMDEVICE_E_INIT;
    }
    else
    {

        switch( _cmdPortType )
        {
#ifndef linux
            case swSerial:
                while( _cmdPort._sw->available() )
                {
                    _cmdPort._sw->read();
                }
                retVal = GSMDEVICE_SUCCESS;
                break;
            case hwSerial:
                while( _cmdPort._hw->available() )
                {
                    _cmdPort._hw->read();
                }
                retVal = GSMDEVICE_SUCCESS;
                break;
            case streamType:
#else
            case linuxDevice:
                retVal = GSMDEVICE_E_SUPPORTED;
                break;
#endif // linux

            case nodev:
            default:
                retVal = GSMDEVICE_E_INIT;
        }
    }

    return( _lastError = retVal );
}

//
// ************************************************************************
//
// set result code format (ATV)
// - set the format of response of commands
//
// Expected arguments:
// - gsmCommandMode cmdMode     cmd_execute only
// - cmdResultCodeFormat *pFmt  holds new value or NULL
// - STRING &result             reference to hold result string
// - void *pParam               pointer to additional parameters
// 
// Returns an INT16 as status code:
// - GSMDEVICE_SUCCESS on succes, or
// depending of the failure that occurred 
// - GSMDEVICE_E_INIT      instance is not initialized
// - GSMDEVICE_E_SUPPORTED not yet supported (e.g. stream device)
// - GSMDEVICE_E_CMD_MODE  invalid command mode
//
// ************************************************************************
//
INT16 gsmDevice::resultCodeFormat( gsmCommandMode cmdMode, cmdResultCodeFormat *pFmt, 
                               STRING &result, void *pParam )
{
    INT16 retVal;
    STRING command = EMPTY_STRING;
    STRING dummy = EMPTY_STRING;
    INT16 errNo;

    if( _cmdPortType  == nodev ||
        _devStatus    == created )
    {
        retVal = GSMDEVICE_E_INIT;
    }
    else
    {
        switch( cmdMode )
        {
            case cmd_execute:
                command = GSMDEVICE_ATTENTION;
                command += RESULT_CODE_FORMAT_CMD_EXEC;
                if( pFmt != NULL )
                {
                    if( *pFmt == cmdResultNumeric ||
                        *pFmt == cmdResultText    )
                    {
                        command += STRING(*pFmt);
                        command += CRLF_STRING;
                        retVal = GSMDEVICE_SUCCESS;
                    }
                    else
                    {
                        retVal = GSMDEVICE_E_CMD_MODE;
                    }
                }
                else
                {
                    command += CRLF_STRING;
                    retVal = GSMDEVICE_SUCCESS;
                }
                break;
            default:
                retVal = GSMDEVICE_E_CMD_MODE;
                break;

        }


        if( retVal == GSMDEVICE_SUCCESS )
        {
            result = EMPTY_STRING;

            switch( _cmdPortType )
            {
                case swSerial:
                case hwSerial:
                case linuxDevice:
                    if((retVal=sendCommand(command, true, NO_TIMEOUT)) == GSMDEVICE_SUCCESS)
                    {
                        retVal = readResponse( result, false, NO_TIMEOUT );

                        if( retVal == GSMDEVICE_SUCCESS )
                        {
                            retVal = checkResponse( result, dummy );
                        }
                    }
                    break;
                case streamType:
                    retVal = GSMDEVICE_E_SUPPORTED;
                    break;
                case nodev:
                default:
                    retVal = GSMDEVICE_E_INIT;
            }
        }
    }

    return( _lastError = retVal );
}

//
// ************************************************************************
//
// set sms format (AT+CMGF)
// - set the format of short messages
//
// Expected arguments:
// - gsmCommandMode cmdMode   cmd_test, cmd_read or cmd_set
// - smsMessageFormat *pFmt   holds new/current value
// - STRING &result           reference to hold result string
// - void *pParam             pointer to additional parameters
// 
// Returns an INT16 as status code:
// - GSMDEVICE_SUCCESS on succes, or
// depending of the failure that occurred 
// - GSMDEVICE_E_INIT      instance is not initialized
// - GSMDEVICE_E_SUPPORTED not yet supported (e.g. stream device)
// - GSMDEVICE_E_CMD_MODE  invalid command mode
//
// ************************************************************************
//
INT16 gsmDevice::smsMsgFormat( gsmCommandMode cmdMode, _dataCMGF *pData, 
                               STRING &result, void *pParam )
{
    INT16 retVal;
    char _pattern[GSMDEVICE_RESP_PATTERN_LEN];
    STRING command = EMPTY_STRING;
    STRING dummy = EMPTY_STRING;
    INT16 errNo;
    INT16 dataIndex;

    if( _cmdPortType  == nodev ||
        _devStatus    == created )
    {
        retVal = GSMDEVICE_E_INIT;
    }
    else
    {
        if( pData != NULL )
        {
            switch( cmdMode )
            {
                case cmd_test:
                    command = GSMDEVICE_ATTENTION;
                    command += SMS_MSG_FORMAT_CMD_TEST;
                    command += CRLF_STRING;
                    retVal = GSMDEVICE_SUCCESS;
                    break;
                case cmd_read:
                    command = GSMDEVICE_ATTENTION;
                    command += SMS_MSG_FORMAT_CMD_READ;
                    command += CRLF_STRING;
                    retVal = GSMDEVICE_SUCCESS;
                    break;
                case cmd_set:
                    if( pData->current == smsPDUMode ||
                        pData->current == smsTXTMode    )
                    {
                        command = GSMDEVICE_ATTENTION;
                        command += SMS_MSG_FORMAT_CMD_SET;
                        command += STRING(pData->current);
                        command += CRLF_STRING;
                        retVal = GSMDEVICE_SUCCESS;
                    }
                    else
                    {
                        retVal = GSMDEVICE_E_CMD_MODE;
                    }
                    break;
                default:
                    retVal = GSMDEVICE_E_CMD_MODE;
                    break;

            }


            if( retVal == GSMDEVICE_SUCCESS )
            {
                result = EMPTY_STRING;

                switch( _cmdPortType )
                {
                    case swSerial:
                    case hwSerial:
                    case linuxDevice:
                        if((retVal=sendCommand(command, true, NO_TIMEOUT)) == GSMDEVICE_SUCCESS)
                        {
                            memset( _pattern, '\0', GSMDEVICE_RESP_PATTERN_LEN );
                            retVal = readResponse( result, false, NO_TIMEOUT );

                            if( retVal == GSMDEVICE_SUCCESS )
                            {
                                if( (retVal = checkResponse( result, dummy )) == GSMDEVICE_SUCCESS )
                                {
                                    if( (retVal = getDataIndex(result, _pattern, GSMDEVICE_RESP_PATTERN_LEN, 
                                                                &dataIndex ) == GSMDEVICE_SUCCESS) )
                                    {
// fprintf(stderr, "scan for [%s] from index %d\n", _pattern, dataIndex);
                                        retVal = parseCMGF( cmdMode, result, _pattern, dataIndex, pData );
                                    }
                                }
                            }
                        }
                        break;
                    case streamType:
                        retVal = GSMDEVICE_E_SUPPORTED;
                        break;
                    case nodev:
                    default:
                        retVal = GSMDEVICE_E_INIT;
                }
            }
        }
        else
        {
            retVal = GSMDEVICE_E_P_NULL;
        }
    }

    return( _lastError = retVal );
}

//
// ************************************************************************
//
// operator selects (AT+COPS)
// - retrieve/set operator connected to
//
// Expected arguments:
// - gsmCommandMode cmdMode   cmd_test, cmd_read or cmd_set
// - opSelectMode *pFmt       holds new/current value
// - STRING &result           reference to hold result string
// - void *pParam             pointer to additional parameters
// 
// Returns an INT16 as status code:
// - GSMDEVICE_SUCCESS on succes, or
// depending of the failure that occurred 
// - GSMDEVICE_E_INIT      instance is not initialized
// - GSMDEVICE_E_SUPPORTED not yet supported (e.g. stream device)
// - GSMDEVICE_E_CMD_MODE  invalid command mode
// - GSMDEVICE_E_P_PARAM   pointer to additional parameters is NULL
//
// ************************************************************************
//
INT16 gsmDevice::operatorSelects( gsmCommandMode cmdMode, opSelectMode *pFmt, 
                                  STRING &result, void *pParam )
{
    INT16 retVal;
    STRING command = EMPTY_STRING;
    STRING dummy = EMPTY_STRING;
    INT16 errNo;

    if( _cmdPortType  == nodev ||
        _devStatus    == created )
    {
        retVal = GSMDEVICE_E_INIT;
    }
    else
    {
        switch( cmdMode )
        {
            case cmd_test:
                command = GSMDEVICE_ATTENTION;
                command += OPERATOR_SELECT_CMD_TEST;
                command += CRLF_STRING;
                retVal = GSMDEVICE_SUCCESS;
                break;
            case cmd_read:
                command = GSMDEVICE_ATTENTION;
                command += OPERATOR_SELECT_CMD_READ;
                command += CRLF_STRING;
                retVal = GSMDEVICE_SUCCESS;
                break;
            case cmd_set:
                if( *pFmt == opSelectAuto ||
                    *pFmt == opSelectManual ||
                    *pFmt == opSelectDeregister ||
                    *pFmt == opSelectFormatOnly ||
                    *pFmt == opSelectManualAuto )

                {
                    command = GSMDEVICE_ATTENTION;
                    command += OPERATOR_SELECT_CMD_SET;
                    command += STRING(*pFmt);
                    command += CRLF_STRING;
                    retVal = GSMDEVICE_SUCCESS;
                }
                else
                {
                    retVal = GSMDEVICE_E_CMD_MODE;
                }
                break;
            default:
                retVal = GSMDEVICE_E_CMD_MODE;
                break;

        }


        if( retVal == GSMDEVICE_SUCCESS )
        {
            result = EMPTY_STRING;

            switch( _cmdPortType )
            {
                case swSerial:
                case hwSerial:
                case linuxDevice:
                    if((retVal=sendCommand(command, true, NO_TIMEOUT)) == GSMDEVICE_SUCCESS)
                    {
                        retVal = readResponse( result, false, 120000 );

                        if( retVal == GSMDEVICE_SUCCESS )
                        {
                            retVal = checkResponse( result, dummy );
                        }
                    }
                    break;
                case streamType:
                    retVal = GSMDEVICE_E_SUPPORTED;
                    break;
                case nodev:
                default:
                    retVal = GSMDEVICE_E_INIT;
            }
        }
    }

    return( _lastError = retVal );
}

//
// ************************************************************************
//
// echo commands (ATE)
// - enable/disable echo of commands sent to attached gsm device
//
// Expected arguments:
// - gsmCommandMode cmdMode   cmd_execute only
// - cmdEcho *pFmt            holds new value or NULL
// - STRING &result           reference to hold result string
// - void *pParam             pointer to additional parameters
// 
// Returns an INT16 as status code:
// - GSMDEVICE_SUCCESS on succes, or
// depending of the failure that occurred 
// - GSMDEVICE_E_INIT      instance is not initialized
// - GSMDEVICE_E_SUPPORTED not yet supported (e.g. stream device)
// - GSMDEVICE_E_CMD_MODE  invalid command mode
//
// ************************************************************************
//
INT16 gsmDevice::commandEcho( gsmCommandMode cmdMode, cmdEcho *pFmt, 
                                  STRING &result, void *pParam )
{
    INT16 retVal;
    STRING command = EMPTY_STRING;
    STRING dummy = EMPTY_STRING;
    INT16 errNo;

    if( _cmdPortType  == nodev ||
        _devStatus    == created )
    {
        retVal = GSMDEVICE_E_INIT;
    }
    else
    {
        switch( cmdMode )
        {
            case cmd_execute:
                command = GSMDEVICE_ATTENTION;
                command += ECHO_COMMAND_CMD_EXEC;
                if( pFmt != NULL )
                {
                    if( *pFmt == cmdEchoOff ||
                        *pFmt == cmdEchoOn )

                    {
                        command += STRING(*pFmt);
                        command += CRLF_STRING;
                        retVal = GSMDEVICE_SUCCESS;
                    }
                    else
                    {
                        retVal = GSMDEVICE_E_CMD_MODE;
                    }
                }
                else
                {
                    command += CRLF_STRING;
                    retVal = GSMDEVICE_SUCCESS;
                }
                break;
            default:
                retVal = GSMDEVICE_E_CMD_MODE;
                break;

        }


        if( retVal == GSMDEVICE_SUCCESS )
        {
            result = EMPTY_STRING;

            switch( _cmdPortType )
            {
                case swSerial:
                case hwSerial:
                case linuxDevice:
                    if((retVal=sendCommand(command, true, NO_TIMEOUT)) == GSMDEVICE_SUCCESS)
                    {
                        retVal = readResponse( result, false, NO_TIMEOUT );

                        if( retVal == GSMDEVICE_SUCCESS )
                        {
                            retVal = checkResponse( result, dummy );
                        }
                    }
                    break;
                case streamType:
                    retVal = GSMDEVICE_E_SUPPORTED;
                    break;
                case nodev:
                default:
                    retVal = GSMDEVICE_E_INIT;
            }
        }
    }

    return( _lastError = retVal );
}

//
// ************************************************************************
//
// network registration (AT+CREG)
// - detect/set mode of network registration
//
// Expected arguments:
// - gsmCommandMode cmdMode     cmd_test, cmd_read or cmd_set
// - _dataCREG *pData           holds new/current value
// - STRING &result             reference to hold result string
// - void *pParam               pointer to additional parameters
// 
// Returns an INT16 as status code:
// - GSMDEVICE_SUCCESS on succes, or
// depending of the failure that occurred 
// - GSMDEVICE_E_INIT      instance is not initialized
// - GSMDEVICE_E_SUPPORTED not yet supported (e.g. stream device)
// - GSMDEVICE_E_CMD_MODE  invalid command mode
//
// ************************************************************************
//
INT16 gsmDevice::networkRegistration( gsmCommandMode cmdMode, _dataCREG *pData, 
                                  STRING &result, void *pParam )
{
    INT16 retVal;
    STRING command = EMPTY_STRING;
    STRING dummy = EMPTY_STRING;
    INT16 errNo;
    char _pattern[GSMDEVICE_RESP_PATTERN_LEN];
    INT16 dataIndex;

    if( _cmdPortType  == nodev ||
        _devStatus    == created )
    {
        retVal = GSMDEVICE_E_INIT;
    }
    else
    {
        if( pData != NULL )
        {
            switch( cmdMode )
            {
                case cmd_test:
                    command = GSMDEVICE_ATTENTION;
                    command += NETWORK_REGISTRATION_CMD_TEST;
                    command += CRLF_STRING;
                    retVal = GSMDEVICE_SUCCESS;
                    break;
                case cmd_read:
                    command = GSMDEVICE_ATTENTION;
                    command += NETWORK_REGISTRATION_CMD_READ;
                    command += CRLF_STRING;
                    retVal = GSMDEVICE_SUCCESS;
                    break;
                case cmd_set:
                    if( pData->mode == disableNetwRegUnsol ||
                        pData->mode == enableNetwRegUnsol ||
                        pData->mode == enableNetwRegUnsolLoc )

                    {
                        command = GSMDEVICE_ATTENTION;
                        command += NETWORK_REGISTRATION_CMD_SET;
                        command += STRING(pData->mode);
                        command += CRLF_STRING;
                        retVal = GSMDEVICE_SUCCESS;
                    }
                    else
                    {
                        retVal = GSMDEVICE_E_CMD_MODE;
                    }
                    break;
                default:
                    retVal = GSMDEVICE_E_CMD_MODE;
                    break;
            }


            if( retVal == GSMDEVICE_SUCCESS )
            {
                result = EMPTY_STRING;

                switch( _cmdPortType )
                {
                    case swSerial:
                    case hwSerial:
                    case linuxDevice:
                        if((retVal=sendCommand(command, true, NO_TIMEOUT)) == GSMDEVICE_SUCCESS)
                        {
                            memset( _pattern, '\0', GSMDEVICE_RESP_PATTERN_LEN );
                            retVal = readResponse( result, false, NO_TIMEOUT );

                            if( retVal == GSMDEVICE_SUCCESS )
                            {
                                if( (retVal = checkResponse( result, dummy )) == GSMDEVICE_SUCCESS )
                                {
                                    if( (retVal = getDataIndex(result, _pattern, GSMDEVICE_RESP_PATTERN_LEN, 
                                                                &dataIndex ) == GSMDEVICE_SUCCESS) )
                                    {
// fprintf(stderr, "scan for [%s] from index %d\n", _pattern, dataIndex);
                                        retVal = parseCREG( cmdMode, result, _pattern, dataIndex, pData );
                                    }
                                }
                            }
                        }
                        break;
                    case streamType:
                        retVal = GSMDEVICE_E_SUPPORTED;
                        break;
                    case nodev:
                    default:
                        retVal = GSMDEVICE_E_INIT;
                }
            }
        }
        else
        {
            retVal = GSMDEVICE_E_P_NULL;
        }
    }

    return( _lastError = retVal );
}

//
// ************************************************************************
//
// signal quality (AT+CSQ)
// - detect current signal quality
//
// Expected arguments:
// - gsmCommandMode cmdMode        cmd_test or cmd_execute
// - struct _dataCSQ *pData   to hold current values
// - STRING &result                reference to hold result string
// - void *pParam                  pointer to additional parameters
// 
// Returns an INT16 as status code:
// - GSMDEVICE_SUCCESS on succes, or
// depending of the failure that occurred 
// - GSMDEVICE_E_INIT      instance is not initialized
// - GSMDEVICE_E_SUPPORTED not yet supported (e.g. stream device)
// - GSMDEVICE_E_CMD_MODE  invalid command mode
//
// ************************************************************************
//
INT16 gsmDevice::signalQuality( gsmCommandMode cmdMode, struct _dataCSQ *pData,
                                  STRING &result, void *pParam )
{
    INT16 retVal;
    STRING command = EMPTY_STRING;
    STRING dummy = EMPTY_STRING;
    INT16 errNo;
    char _pattern[GSMDEVICE_RESP_PATTERN_LEN];
    INT16 dataIndex;

    if( _cmdPortType  == nodev ||
        _devStatus    == created )
    {
        retVal = GSMDEVICE_E_INIT;
    }
    else
    {
        switch( cmdMode )
        {
            case cmd_test:
                command = GSMDEVICE_ATTENTION;
                command += SIGNAL_QUALITY_CMD_TEST;
                command += CRLF_STRING;
                retVal = GSMDEVICE_SUCCESS;
                break;
            case cmd_execute:
                command = GSMDEVICE_ATTENTION;
                command += SIGNAL_QUALITY_CMD_EXEC;
                command += CRLF_STRING;
                retVal = GSMDEVICE_SUCCESS;
                break;
            default:
                retVal = GSMDEVICE_E_CMD_MODE;
                break;
        }


        if( retVal == GSMDEVICE_SUCCESS )
        {
            result = EMPTY_STRING;

            switch( _cmdPortType )
            {
                case swSerial:
                case hwSerial:
                case linuxDevice:
                    if((retVal=sendCommand(command, true, NO_TIMEOUT)) == GSMDEVICE_SUCCESS)
                    {
                        memset( _pattern, '\0', GSMDEVICE_RESP_PATTERN_LEN );
                        retVal = readResponse( result, false, NO_TIMEOUT );

                        if( retVal == GSMDEVICE_SUCCESS )
                        {
                            if( (retVal = checkResponse( result, dummy )) == GSMDEVICE_SUCCESS )
                            {
                                if( (retVal = getDataIndex(result, _pattern, GSMDEVICE_RESP_PATTERN_LEN, 
                                                            &dataIndex ) == GSMDEVICE_SUCCESS) )
                                {
// fprintf(stderr, "scan for [%s] from index %d\n", _pattern, dataIndex);
                                    retVal = parseCSQ( cmdMode, result, _pattern, dataIndex, pData );
                                }
                            }
                        }
                    }
                    break;
                case streamType:
                    retVal = GSMDEVICE_E_SUPPORTED;
                    break;
                case nodev:
                default:
                    retVal = GSMDEVICE_E_INIT;
            }
        }
    }

    return( _lastError = retVal );
}

//
// ************************************************************************
//
// preferred operator list (AT+CPOL)
// - manage list with preferred operators
//
// Expected arguments:
// - gsmCommandMode cmdMode         cmd_test, cmd_read or cmd_set
// - prefOperList *pMode            holds new/current value
// - STRING &result                 reference to hold result string
// - void *pParam                   pointer to additional parameters
// 
// Returns an INT16 as status code:
// - GSMDEVICE_SUCCESS on succes, or
// depending of the failure that occurred 
// - GSMDEVICE_E_INIT      instance is not initialized
// - GSMDEVICE_E_SUPPORTED not yet supported (e.g. stream device)
// - GSMDEVICE_E_CMD_MODE  invalid command mode
//
// ************************************************************************
//
INT16 gsmDevice::preferredOperatorList( gsmCommandMode cmdMode, prefOperList *pMode, 
                                  STRING &result, void *pParam )
{
    INT16 retVal;
    STRING command = EMPTY_STRING;
    STRING dummy = EMPTY_STRING;
    INT16 errNo;

    if( _cmdPortType  == nodev ||
        _devStatus    == created )
    {
        retVal = GSMDEVICE_E_INIT;
    }
    else
    {
        switch( cmdMode )
        {
            case cmd_test:
                command = GSMDEVICE_ATTENTION;
                command += PREF_OPERATOR_LIST_CMD_TEST;
                command += CRLF_STRING;
                retVal = GSMDEVICE_SUCCESS;
                break;
            case cmd_read:
                command = GSMDEVICE_ATTENTION;
                command += PREF_OPERATOR_LIST_CMD_READ;
                command += CRLF_STRING;
                retVal = GSMDEVICE_SUCCESS;
                break;
            case cmd_set:
                if( *pMode == prefOperLongAlphaMode ||
                    *pMode == prefOperShortAlphaMode ||
                    *pMode == prefOperNumericMode )

                {
                    command = GSMDEVICE_ATTENTION;
                    command += PREF_OPERATOR_LIST_CMD_SET;
                    command += STRING(*pMode);
                    command += CRLF_STRING;
                    retVal = GSMDEVICE_SUCCESS;
                }
                else
                {
                    retVal = GSMDEVICE_E_CMD_MODE;
                }
                break;
            default:
                retVal = GSMDEVICE_E_CMD_MODE;
                break;
        }


        if( retVal == GSMDEVICE_SUCCESS )
        {
            result = EMPTY_STRING;

            switch( _cmdPortType )
            {
                case swSerial:
                case hwSerial:
                case linuxDevice:
                    if((retVal=sendCommand(command, true, NO_TIMEOUT)) == GSMDEVICE_SUCCESS)
                    {
                        retVal = readResponse( result, false, NO_TIMEOUT );

                        if( retVal == GSMDEVICE_SUCCESS )
                        {
                            retVal = checkResponse( result, dummy );
                        }
                    }
                    break;
                case streamType:
                    retVal = GSMDEVICE_E_SUPPORTED;
                    break;
                case nodev:
                default:
                    retVal = GSMDEVICE_E_INIT;
            }
        }
    }

    return( _lastError = retVal );
}

//
// ************************************************************************
//
// request IMSI (AT+CIMI)
// - get international mobile subsciber identification
//
// Expected arguments:
// - gsmCommandMode cmdMode         cmd_test or cmd_set
// - struct _dataIMSI *pData        holds parameter
// - STRING &result                 reference to hold result string
// - void *pParam                   pointer to additional parameters
// 
// Returns an INT16 as status code:
// - GSMDEVICE_SUCCESS on succes, or
// depending of the failure that occurred 
// - GSMDEVICE_E_INIT      instance is not initialized
// - GSMDEVICE_E_SUPPORTED not yet supported (e.g. stream device)
// - GSMDEVICE_E_CMD_MODE  invalid command mode
//
// ************************************************************************
//
INT16 gsmDevice::requestIMSI( gsmCommandMode cmdMode, struct _dataIMSI *pData, 
                                  STRING &result, void *pParam )
{
    INT16 retVal;
    STRING command = EMPTY_STRING;
    STRING dummy = EMPTY_STRING;
    INT16 errNo;

    if( _cmdPortType  == nodev ||
        _devStatus    == created )
    {
        retVal = GSMDEVICE_E_INIT;
    }
    else
    {
        switch( cmdMode )
        {
            case cmd_test:
                command = GSMDEVICE_ATTENTION;
                command += REQUEST_IMSI_CMD_TEST;
                command += CRLF_STRING;
                retVal = GSMDEVICE_SUCCESS;
                break;
            case cmd_set:
                command = GSMDEVICE_ATTENTION;
                command += REQUEST_IMSI_CMD_SET;
                command += CRLF_STRING;
                retVal = GSMDEVICE_SUCCESS;
                break;
            default:
                retVal = GSMDEVICE_E_CMD_MODE;
                break;
        }


        if( retVal == GSMDEVICE_SUCCESS )
        {
            result = EMPTY_STRING;

            switch( _cmdPortType )
            {
                case swSerial:
                case hwSerial:
                case linuxDevice:
                    if((retVal=sendCommand(command, true, NO_TIMEOUT)) == GSMDEVICE_SUCCESS)
                    {
                        retVal = readResponse( result, false, NO_TIMEOUT );

                        if( retVal == GSMDEVICE_SUCCESS )
                        {
                            if( (retVal = checkResponse( result, dummy )) == GSMDEVICE_SUCCESS )
                            {
                                retVal = parseIMSI( cmdMode, result, pData );

                            }
                        }
                    }
                    break;
                case streamType:
                    retVal = GSMDEVICE_E_SUPPORTED;
                    break;
                case nodev:
                default:
                    retVal = GSMDEVICE_E_INIT;
            }
        }
    }

    return( _lastError = retVal );
}

//
// ************************************************************************
//
// read/write IMEI (AT+EGMR)
// - get international mobile equipment identity
//
// Expected arguments:
// - gsmCommandMode cmdMode         cmd_test or cmd_set
// - struct _dataEGMR *pData        parameters
// - STRING &result                 reference to hold result string
// - void *pParam                   pointer to additional parameters
// 
// Returns an INT16 as status code:
// - GSMDEVICE_SUCCESS on succes, or
// depending of the failure that occurred 
// - GSMDEVICE_E_INIT      instance is not initialized
// - GSMDEVICE_E_P_NULL    pData is a null pointer
// - GSMDEVICE_E_SUPPORTED not yet supported (e.g. stream device)
// - GSMDEVICE_E_CMD_MODE  invalid command mode
//
// ************************************************************************
//
INT16 gsmDevice::readWriteIMEI( gsmCommandMode cmdMode, struct _dataEGMR *pData, 
                                  STRING &result, void *pParam )
{
    INT16 retVal;
    STRING command = EMPTY_STRING;
    STRING dummy = EMPTY_STRING;
    INT16 errNo;
    INT16 dataIndex;
    char _pattern[GSMDEVICE_RESP_PATTERN_LEN];

    if( _cmdPortType  == nodev ||
        _devStatus    == created )
    {
        retVal = GSMDEVICE_E_INIT;
    }
    else
    {
        if( pData != NULL )
        {
            pData->format = 7;

            switch( cmdMode )
            {
                case cmd_test:
                    command = GSMDEVICE_ATTENTION;
                    command += READ_WRITE_IMEI_CMD_TEST;
                    command += CRLF_STRING;
                    retVal = GSMDEVICE_SUCCESS;
                    break;
                case cmd_set:
                    command = GSMDEVICE_ATTENTION;
                    command += READ_WRITE_IMEI_CMD_SET;
                    command += STRING(pData->mode);
                    command += STRING(",");
                    command += STRING(pData->format);
                    command += CRLF_STRING;
                    retVal = GSMDEVICE_SUCCESS;
                    break;
                default:
                    retVal = GSMDEVICE_E_CMD_MODE;
                    break;
            }


            if( retVal == GSMDEVICE_SUCCESS )
            {
                result = EMPTY_STRING;

                switch( _cmdPortType )
                {
                    case swSerial:
                    case hwSerial:
                    case linuxDevice:
                        if((retVal=sendCommand(command, true, NO_TIMEOUT)) == GSMDEVICE_SUCCESS)
                        {
                            memset( _pattern, '\0', GSMDEVICE_RESP_PATTERN_LEN );
                            retVal = readResponse( result, false, NO_TIMEOUT );

                            if( retVal == GSMDEVICE_SUCCESS )
                            {
                                if( (retVal = checkResponse( result, dummy )) == GSMDEVICE_SUCCESS )
                                {
                                    if( (retVal = getDataIndex(result, _pattern, GSMDEVICE_RESP_PATTERN_LEN, 
                                                                &dataIndex ) == GSMDEVICE_SUCCESS) )
                                    {
// fprintf(stderr, "scan for [%s] from index %d\n", _pattern, dataIndex);
                                        retVal = parseEGMR( cmdMode, result, _pattern, dataIndex, pData );
                                    }
                                }
                            }
                        }
                        break;
                    case streamType:
                        retVal = GSMDEVICE_E_SUPPORTED;
                        break;
                    case nodev:
                    default:
                        retVal = GSMDEVICE_E_INIT;
                }
            }
        }
        else
        {
            retVal = GSMDEVICE_E_P_NULL;
        }
    }

    return( _lastError = retVal );
}

//
// ************************************************************************
//
// request revision id (AT+CGMR)
// - get revision identifaction
//
// Expected arguments:
// - gsmCommandMode cmdMode         cmd_test or cmd_set
// - void *pIgnored                 may be NULL, is ignored
// - STRING &result                 reference to hold result string
// - void *pParam                   pointer to additional parameters
// 
// Returns an INT16 as status code:
// - GSMDEVICE_SUCCESS on succes, or
// depending of the failure that occurred 
// - GSMDEVICE_E_INIT      instance is not initialized
// - GSMDEVICE_E_SUPPORTED not yet supported (e.g. stream device)
// - GSMDEVICE_E_CMD_MODE  invalid command mode
//
// ************************************************************************
//
INT16 gsmDevice::requestRevisionId( gsmCommandMode cmdMode, void *pIgnored, 
                                  STRING &result, void *pParam )
{
    INT16 retVal;
    STRING command = EMPTY_STRING;
    STRING dummy = EMPTY_STRING;
    INT16 errNo;

    if( _cmdPortType  == nodev ||
        _devStatus    == created )
    {
        retVal = GSMDEVICE_E_INIT;
    }
    else
    {
        switch( cmdMode )
        {
            case cmd_test:
                command = GSMDEVICE_ATTENTION;
                command += REQUEST_REV_ID_CMD_TEST;
                command += CRLF_STRING;
                retVal = GSMDEVICE_SUCCESS;
                break;
            case cmd_set:
                command = GSMDEVICE_ATTENTION;
                command += REQUEST_REV_ID_CMD_SET;
                command += CRLF_STRING;
                retVal = GSMDEVICE_SUCCESS;
                break;
            default:
                retVal = GSMDEVICE_E_CMD_MODE;
                break;
        }


        if( retVal == GSMDEVICE_SUCCESS )
        {
            result = EMPTY_STRING;

            switch( _cmdPortType )
            {
                case swSerial:
                case hwSerial:
                case linuxDevice:
                    if((retVal=sendCommand(command, true, NO_TIMEOUT)) == GSMDEVICE_SUCCESS)
                    {
                        retVal = readResponse( result, false, NO_TIMEOUT );

                        if( retVal == GSMDEVICE_SUCCESS )
                        {
                            if( (retVal = checkResponse( result, dummy )) == GSMDEVICE_SUCCESS )
                            {
                                retVal = removeEcho( result, dummy );
                            }
                        }
                    }
                    break;
                case streamType:
                    retVal = GSMDEVICE_E_SUPPORTED;
                    break;
                case nodev:
                default:
                    retVal = GSMDEVICE_E_INIT;
            }
        }
    }

    return( _lastError = retVal );
}

//
// ************************************************************************
//
// request manufaturer data (ATI)
// - get manufactor specific information
//
// Expected arguments:
// - gsmCommandMode cmdMode         cmd_set only
// - INT16 infoValue                manufacturer specific 
// - STRING &result                 reference to hold result string
// - void *pParam                   pointer to additional parameters
// 
// Returns an INT16 as status code:
// - GSMDEVICE_SUCCESS on succes, or
// depending of the failure that occurred 
// - GSMDEVICE_E_INIT      instance is not initialized
// - GSMDEVICE_E_SUPPORTED not yet supported (e.g. stream device)
// - GSMDEVICE_E_CMD_MODE  invalid command mode
//
// ************************************************************************
//
INT16 gsmDevice::requestManufacturerData( gsmCommandMode cmdMode, INT16 infoValue, 
                                  STRING &result, void *pParam )
{
    INT16 retVal;
    STRING command = EMPTY_STRING;
    STRING dummy = EMPTY_STRING;
    INT16 errNo;

    if( _cmdPortType  == nodev ||
        _devStatus    == created )
    {
        retVal = GSMDEVICE_E_INIT;
    }
    else
    {
        switch( cmdMode )
        {
            case cmd_set:
                command = GSMDEVICE_ATTENTION;
                command += REQUEST_MANSPEC_INFO_CMD_SET;
                command += STRING(infoValue);
                command += CRLF_STRING;
                retVal = GSMDEVICE_SUCCESS;
                break;
            default:
                retVal = GSMDEVICE_E_CMD_MODE;
                break;
        }


        if( retVal == GSMDEVICE_SUCCESS )
        {
            result = EMPTY_STRING;

            switch( _cmdPortType )
            {
                case swSerial:
                case hwSerial:
                case linuxDevice:
                    if((retVal=sendCommand(command, true, NO_TIMEOUT)) == GSMDEVICE_SUCCESS)
                    {
                        retVal = readResponse( result, false, NO_TIMEOUT );

                        if( retVal == GSMDEVICE_SUCCESS )
                        {
                            if( (retVal = checkResponse( result, dummy )) == GSMDEVICE_SUCCESS )
                            {
                                retVal = removeEcho( result, dummy );
                            }
                        }
                    }
                    break;
                case streamType:
                    retVal = GSMDEVICE_E_SUPPORTED;
                    break;
                case nodev:
                default:
                    retVal = GSMDEVICE_E_INIT;
            }
        }
    }

    return( _lastError = retVal );
}

//
// ************************************************************************
// Get last error information
// - request a string with human friendly error description

// Expected arguments:
// - none
//
// Returns:
// - STRING  human readable error message
//
// ************************************************************************
INT16 gsmDevice::getError( STRING &errMsg )
{
    INT16 retVal;

    switch( _lastError )
    {
        case GSMDEVICE_SUCCESS:
        case GSMDEVICE_ERROR:
        case GSMDEVICE_E_P_RXPIN:
        case GSMDEVICE_E_P_TXPIN:
        case GSMDEVICE_E_P_SPEED:
        case GSMDEVICE_E_P_TMOUT:
        case GSMDEVICE_E_P_SERIAL:
        case GSMDEVICE_E_P_STREAM:
        case GSMDEVICE_E_P_NULL:
        case GSMDEVICE_E_P_PARAM:
        case GSMDEVICE_E_INIT:
        case GSMDEVICE_E_RESPONSE:
        case GSMDEVICE_E_SUPPORTED:
        case GSMDEVICE_E_DEV_TYPE:
        case GSMDEVICE_E_SEND:
        case GSMDEVICE_E_MATCH:
        case GSMDEVICE_E_CMD_MODE:
        case GSMDEVICE_E_OPEN:
        case GSMDEVICE_E_SETUP:
        case GSMDEVICE_E_PATTERN:
        case GSMDEVICE_E_RESULT:
        case GSMDEVICE_E_INVAL:
        case GSMDEVICE_E_TOO_SHORT:
        case GSMDEVICE_E_CME_UNKNOWN:
        case GSMDEVICE_E_CMS_UNKNOWN:
            errMsg = "Somethind went wrong!\n";
            break;
            retVal = cmeErrorMsg( errMsg );
            break;
        case GSMDEVICE_E_CMS:
            retVal = cmsErrorMsg( errMsg );
            break;
        default:
            retVal = GSMDEVICE_E_UNSPECIFIC;
            errMsg = "Somethind else went wrong!\n";
            break;
    }

    return( _lastError = retVal );
}


// 
// ----------------------------- PRIVATE STUFF ----------------------------
//

// ////////////////////////////////////////////////////////////////////////
//
// get status
//   - check response to detect status of a command
//
// Expected arguments:
// - STRING result           resultstring to scan
// - STRING &dummy           no function yet
// 
// Returns an INT16 as status code:
// - GSMDEVICE_SUCCESS on succes, or
// depending of the failure that occurred 
// - GSMDEVICE_E_CMS  a CMS error occured
// - GSMDEVICE_E_CME  a CME error occured
// - GSMDEVICE_ERROR  some error, not specified
//
// ////////////////////////////////////////////////////////////////////////
//
INT16 gsmDevice::checkResponse( STRING result, STRING &dummy )
{
    INT16 retVal;
    INT16 errNo;

    // result.length()
    if((retVal = parseResponse( result, GSMDEVICE_OK_MSG, dummy )) != GSMDEVICE_SUCCESS)
    {
        if((retVal = parseResponse( result, GSMDEVICE_E_CME_MSG, dummy )) != GSMDEVICE_SUCCESS)
        {
            if((retVal = parseResponse(result, GSMDEVICE_E_CMS_MSG, dummy )) != GSMDEVICE_SUCCESS)
            {
                retVal = GSMDEVICE_ERROR;
            }
            else
            {
                if( (retVal = scanCMSErrNum( result, errNo )) == GSMDEVICE_SUCCESS )
                {
                    _cmsLastError = errNo;
                    retVal = GSMDEVICE_E_CMS;
                }
                else
                {
                    retVal = GSMDEVICE_ERROR;
                }
            }
        }
        else
        {
            if( (retVal = scanCMEErrNum( result, errNo )) == GSMDEVICE_SUCCESS )
            {
                _cmeLastError = errNo;
// fprintf(stderr, "got CME error %d\n", _cmeLastError);
                retVal = GSMDEVICE_E_CME;
            }
            else
            {
                retVal = GSMDEVICE_ERROR;
            }
        }
    }
    else
    {
        retVal = GSMDEVICE_SUCCESS;
    }

    return ( _lastError = retVal );
}

// ////////////////////////////////////////////////////////////////////////
//
// Syncronize gsmDevice
//   - check whether gsm device is responding and ready
//
// Expected arguments:
// - none
// 
// Returns an INT16 as status code:
// - GSMDEVICE_SUCCESS on succes, or
// depending of the failure that occurred 
// - GSMDEVICE_E_INIT     instance is not initialized
// - GSMDEVICE_E_RESPONSE gsm device (e.g. gsm modem) did not respond
//
// ////////////////////////////////////////////////////////////////////////
//
INT16 gsmDevice::syncWithResponse( STRING cmd, STRING expect )
{
    INT16 retVal;
    int triesDone;
    bool devReady;
    int wrote;
    STRING response;

    if( _cmdPortType  == nodev ||
        _devStatus    == created )
    {
        retVal = GSMDEVICE_E_INIT;
    }
    else
    {

        devReady  = false;
        triesDone = 0;

        inputFlush();

        do
        {
            if( (triesDone % GSMDEV_SYNC_MAX_CMD) == 0 )
            {
                sendCommand( cmd, false, NO_TIMEOUT );
            }

            readResponse( response, false, NO_TIMEOUT );
//            response.trim();
            if(response.indexOf(expect) >= 0)
            {
                devReady = true;
            }

            delay(GSMDEV_SYNC_DELAY);

// Serial.print("RESPONSE: ");
// Serial.println(response);

        } while( !devReady && triesDone++ < GSMDEV_SYNC_MAX_TOTAL );

// Serial.print("index of: ");
// Serial.print(expect);
// Serial.print(" = ");
// Serial.println(response.indexOf(expect));

        if( devReady )
        {
            retVal = GSMDEVICE_SUCCESS;
        }
        else
        {
            retVal = GSMDEVICE_E_RESPONSE;
        }
    }

    return( _lastError = retVal );

}

// ////////////////////////////////////////////////////////////////////////
//
// Send command
//   - sends a given command to the attached device
//
// Expected arguments:
// - STRING cmd           command to send
// - BOOL   flushBefore   flag whether to flush before send
// 
// Returns an INT16 as status code:
// - GSMDEVICE_SUCCESS on succes, or
// depending of the failure that occurred 
// - GSMDEVICE_E_INIT      instance is not initialized
// - GSMDEVICE_E_SUPPORTED not yet supported (e.g. stream device)
// - GSMDEVICE_E_SEND      cannot send to gsm device (e.g. gsm modem)
//
// ////////////////////////////////////////////////////////////////////////
//
INT16 gsmDevice::sendCommand( STRING cmd, BOOL flushBefore, INT32 timeout )
{

    INT16 retVal;
    UINT16 wroteBytes;

    if( _cmdPortType  == nodev ||
        _devStatus    == created )
    {
        retVal = GSMDEVICE_E_INIT;
    }
    else
    {

        if( flushBefore )
        {
            inputFlush();
        }

        switch( _cmdPortType )
        {
#ifndef linux
            case swSerial:
                // TODO: check whether println works
                wroteBytes = _cmdPort._sw->println(cmd);
                if( wroteBytes >= cmd.length() )
                {
                    retVal = GSMDEVICE_SUCCESS;
                }
                else
                {
                    retVal = GSMDEVICE_E_SEND;
                }
                break;
            case hwSerial:
                // TODO: check whether println works
                wroteBytes = _cmdPort._hw->println(cmd);
                if( wroteBytes >= cmd.length() )
                {
                    retVal = GSMDEVICE_SUCCESS;
                }
                else
                {
                    retVal = GSMDEVICE_E_SEND;
                }
                break;
            case streamType:
                retVal = GSMDEVICE_E_SUPPORTED;
                break;
#else
            case linuxDevice:
                wroteBytes = write(_cmdPort._dev, cmd.c_str(), cmd.length() ); 
                if( wroteBytes >= cmd.length() )
                {
                    retVal = GSMDEVICE_SUCCESS;
                }
                else
                {
                    retVal = GSMDEVICE_E_SEND;
                }
                break;
#endif // linux
            case nodev:
            default:
                retVal = GSMDEVICE_E_INIT;
                break;
        }
    }

    return( _lastError = retVal );
}

// ////////////////////////////////////////////////////////////////////////
//
// read response 
//   - read the current response of the attached gsm device
//
// Expected arguments:
// - STRING &response    reference to string to hold data
// - BOOL   flushAfter   flag whether to flush after reading
// 
// Returns an INT16 as status code:
// - GSMDEVICE_SUCCESS on succes, or
// depending of the failure that occurred 
// - GSMDEVICE_E_INIT     instance is not initialized
// - GSMDEVICE_E_RESPONSE gsm device (e.g. gsm modem) did not respond
//
// ////////////////////////////////////////////////////////////////////////
//
INT16 gsmDevice::readResponse( STRING &response, BOOL flushAfter, INT32 timeout )
{

#ifdef linux
    char rawData[512];
#endif // linux

    INT16 retVal;
    BOOL tmOut;
    INT32 startTimeout;

    if( _cmdPortType  == nodev ||
        _devStatus    == created )
    {
        retVal = GSMDEVICE_E_INIT;
    }
    else
    {

        switch( _cmdPortType )
        {
#ifndef linux
            case swSerial:
                tmOut = false;
                startTimeout = 0;
                if( timeout == NO_TIMEOUT )
                {
                    timeout = 200;
                }

                do
                {
                    if( _cmdPort._sw->available() )
                    {
                        response += _cmdPort._sw->readString();
                    }
                    else
                    {
                        if( startTimeout == 0 )
                        {
                            startTimeout = millis();
                        }
                        else
                        {
                            if( (millis() - startTimeout) >= timeout )
                            {
                                tmOut = true;
                            }
                        }

                        delay(GSMDEV_READ_DELAY);
                    }
                } while(!tmOut);

                if( flushAfter )
                {
                    inputFlush();
                }
    
                if( response.length() )
                {
                    retVal = GSMDEVICE_SUCCESS;
                }
                else
                {
                    retVal = GSMDEVICE_E_RESPONSE;
                }
                break;
            case hwSerial:
                tmOut = false;
                startTimeout = 0;
                if( timeout == NO_TIMEOUT )
                {
                    timeout = 200;
                }

                do
                {
                    if( _cmdPort._sw->available() )
                    {
                        response += _cmdPort._hw->readString();
                    }
                    else
                    {
                        if( startTimeout == 0 )
                        {
                            startTimeout = millis();
                        }
                        else
                        {
                            if( (millis() - startTimeout) >= timeout )
                            {
                                tmOut = true;
                            }
                        }

                        delay(GSMDEV_READ_DELAY);
                    }
                } while(!tmOut);

                if( flushAfter )
                {
                    inputFlush();
                }
    
                if( response.length() )
                {
                    retVal = GSMDEVICE_SUCCESS;
                }
                else
                {
                    retVal = GSMDEVICE_E_RESPONSE;
                }
                break;
            case streamType:
                retVal = GSMDEVICE_E_SUPPORTED;
                break;
#else
            case linuxDevice:
                memset( rawData, '\0', sizeof(rawData) );
                retVal = uartReadResponse( _cmdPort._dev, rawData, sizeof(rawData), timeout );
                if( strlen(rawData) )
                {
                    response = String( (const char*) rawData);
                    retVal = GSMDEVICE_SUCCESS;
                }
                else
                {
                    retVal = GSMDEVICE_E_RESPONSE;
                }
                break;
#endif // linux
            case nodev:
            default:
                retVal = GSMDEVICE_E_INIT;
                break;
        }
    }

    return( _lastError = retVal );
}

#ifdef linux

// ////////////////////////////////////////////////////////////////////////
//
// read string from uart
//   - read uart until '\n'
//
// Expected arguments:
// - int  fd          filedescriptor to open device
// - char *pResponse  buffer to hold device response
// - int  maxLen      size of buffer
// - long timeout     only test at this time ... timeout value
// 
// Returns an INT16 as status code:
// - GSMDEVICE_SUCCESS on succes, or
// depending of the failure that occurred 
// - GSMDEVICE_E_P_NULL     buffer points to NULL
// - GSMDEVICE_E_RESPONSE   gsm device (e.g. gsm modem) did not respond
//
// ////////////////////////////////////////////////////////////////////////
//
INT16 gsmDevice::removeNullChars( char *pResponse, int maxLen )
{
    INT16 retVal;

    while( maxLen > 0 && *(pResponse+maxLen-1) == '\0' )
    {
        maxLen--;
    }

    return( maxLen );
}


// ////////////////////////////////////////////////////////////////////////
//
// read string from uart
//   - read uart until '\n'
//
// Expected arguments:
// - int  fd          filedescriptor to open device
// - char *pResponse  buffer to hold device response
// - int  maxLen      size of buffer
// - long timeout     only test at this time ... timeout value
// 
// Returns an INT16 as status code:
// - GSMDEVICE_SUCCESS on succes, or
// depending of the failure that occurred 
// - GSMDEVICE_E_P_NULL     buffer points to NULL
// - GSMDEVICE_E_RESPONSE   gsm device (e.g. gsm modem) did not respond
//
// ////////////////////////////////////////////////////////////////////////
//
INT16 gsmDevice::uartReadString( int fd, char *pResponse, int maxLen, int *pRead, long timeout )
{
    INT16 retVal;
    BOOL done, tmOut;
    INT16 rdLen, totalRead;
    long timeOutStart;

    done = tmOut = false;
    timeOutStart = 0;
    totalRead = 0;

    do
    {
        if( (rdLen = read(fd, pResponse, maxLen)) > 0 )
        {
            rdLen = removeNullChars( pResponse, rdLen );
            if( *(pResponse+rdLen-1) == '\n' )
            {
                done = true;
                retVal = GSMDEVICE_SUCCESS;
            }
            else
            {
                pResponse += rdLen;
                maxLen -= rdLen;

                if( timeOutStart != 0 )
                {
                    timeOutStart = 0;
                }
            }

            totalRead += rdLen;
        }
        else
        {
            if( timeOutStart == 0 )
            {
                timeOutStart = millis();
            }
            else
            {
                if( (millis() - timeOutStart) >= timeout )
                {
                    tmOut = true;
                    retVal = GSMDEVICE_E_RESPONSE;
// printf("TIMEOUT: %ld started, now %ld, value was %ld\n", timeOutStart, millis(), timeout);

                }
            }

            _lastErrno = errno;
            
        }

    } while( !done && !tmOut );

    *pRead = totalRead;

    return( _lastError = retVal );
}

// ////////////////////////////////////////////////////////////////////////
//
// read uart response 
//   - read the current response of the attached gsm device
//
// Expected arguments:
// - int  fd          filedescriptor to open device
// - char *pResponse  buffer to hold device response
// - int  maxLen      size of buffer
// - long timeout     only test at this time ... timeout value
// 
// Returns an INT16 as status code:
// - GSMDEVICE_SUCCESS on succes, or
// depending of the failure that occurred 
// - GSMDEVICE_E_P_NULL     buffer points to NULL
// - GSMDEVICE_E_RESPONSE   gsm device (e.g. gsm modem) did not respond
//
// ////////////////////////////////////////////////////////////////////////
//
INT16 gsmDevice::uartReadResponse( int fd, char *pResponse, int maxLen, long timeout )
{
    INT16 retVal;
    int rdLen;
    bool tmOut;
    bool done;
    time_t now;
    int geeks;

    if( pResponse != NULL )
    {
        while( (retVal = uartReadString( fd, pResponse, maxLen, &rdLen, timeout )) == GSMDEVICE_SUCCESS )
        {
// printf("GOT ->[%s]\n", pResponse);
            if( maxLen > rdLen )
            {
                pResponse += rdLen;
                maxLen -= rdLen;
            }
            else
            {
                break;
            }
        }
    }
    else
    {
      retVal = GSMDEVICE_E_P_NULL;
    }

    return( _lastError = retVal );

}

#endif // linux


// ////////////////////////////////////////////////////////////////////////
//
// retrieve CME error message 
//   - read the current response of the attached gsm device
//
// Expected arguments:
// - STRING &errmsg    reference to hold error message
// - 
// 
// Returns an INT16 as status code:
// - GSMDEVICE_SUCCESS on succes, or
// depending of the failure that occurred 
// - GSMDEVICE_E_CME_UNKNOWN   no matching message was found
//
// ////////////////////////////////////////////////////////////////////////
//
INT16 gsmDevice::cmeErrorMsg( STRING &errmsg )
{
    INT16 retVal;
    BOOL msgFound;
    int currIdx;

    static struct _gsm_errcode2msg cme_error[] = {
        {   0, "Phone failure" },
        {   1, "No connection to phone" },
        {   2, "Phone adapter link reserved" },
        {   3, "Operation not allowed" },
        {   4, "Operation not supported" },
        {   5, "PH_SIM PIN required" },
        {   6, "PH_FSIM PIN required" },
        {   7, "PH_FSIM PUK required" },
        {  10, "SIM not inserted" },
        {  11, "SIM PIN required" },
        {  12, "SIM PUK required" },
        {  13, "SIM failure" },
        {  14, "SIM busy" },
        {  15, "SIM wrong" },
        {  16, "Incorrect password" },
        {  17, "SIM PIN2 required" },
        {  18, "SIM PUK2 required" },
        {  20, "Memory full" },
        {  21, "Invalid index" },
        {  22, "Not found" },
        {  23, "Memory failure" },
        {  24, "Text string too long" },
        {  25, "Invalid characters in text string" },
        {  26, "Dial string too long" },
        {  27, "Invalid characters in dial string" },
        {  30, "No network service" },
        {  31, "Network timeout" },
        {  32, "Network not allowed, emergency calls only" },
        {  40, "Network personalization PIN required" },
        {  41, "Network personalization PUK required" },
        {  42, "Network subset personalization PIN required" },
        {  43, "Network subset personalization PUK required" },
        {  44, "Service provider personalization PIN required" },
        {  45, "Service provider personalization PUK required" },
        {  46, "Corporate personalization PIN required" },
        {  47, "Corporate personalization PUK required" },
        {  48, "PH-SIM PUK required" },
        { 100, "Unknown error" },
        { 103, "Illegal MS" },
        { 106, "Illegal ME" },
        { 107, "GPRS services not allowed" },
        { 111, "PLMN not allowed" },
        { 112, "Location area not allowed" },
        { 113, "Roaming not allowed in this location area" },
        { 126, "Operation temporary not allowed" },
        { 132, "Service operation not supported" },
        { 133, "Requested service option not subscribed" },
        { 134, "Service option temporary out of order" },
        { 148, "Unspecified GPRS error" },
        { 149, "PDP authentication failure" },
        { 150, "Invalid mobile class" },
        { 256, "Operation temporarily not allowed" },
        { 257, "Call barred" },
        { 258, "Phone is busy" },
        { 259, "User abort" },
        { 260, "Invalid dial string" },
        { 261, "SS not executed" },
        { 262, "SIM Blocked" },
        { 263, "Invalid block" },
        { 772, "SIM powered down" },
        { -1 , "" }
    };

    msgFound = false;
    currIdx = 0;

    do
    {
        if( cme_error[currIdx].errcode == _cmeLastError )
        {
            msgFound = true;
            errmsg = STRING(cme_error[currIdx].pMessage);
        }
    
    } while( !msgFound && cme_error[currIdx++].errcode >= 0 );

    if( msgFound )
    {
        retVal = GSMDEVICE_SUCCESS;
    }
    else
    {
        retVal = GSMDEVICE_E_CME_UNKNOWN;
    }

    return( _lastError = retVal );
}

// ////////////////////////////////////////////////////////////////////////
//
// retrieve CMS error message 
//   - read the current response of the attached gsm device
//
// Expected arguments:
// - STRING &errmsg    reference to hold error message
// - 
// 
// Returns an INT16 as status code:
// - GSMDEVICE_SUCCESS on succes, or
// depending of the failure that occurred 
// - GSMDEVICE_E_CMS_UNKNOWN   no matching message was found
//
// ////////////////////////////////////////////////////////////////////////
//
INT16 gsmDevice::cmsErrorMsg( STRING &errmsg )
{
    INT16 retVal;
    BOOL msgFound;
    int currIdx;

    static struct _gsm_errcode2msg cms_error[] = {
        {   1, "Unassigned number" },
        {   8, "Operator determined barring" },
        {  10, "Call bared" },
        {  21, "Short message transfer rejected" },
        {  27, "Destination out of service" },
        {  28, "Unindentified subscriber" },
        {  29, "Facility rejected" },
        {  30, "Unknown subscriber" },
        {  38, "Network out of order" },
        {  41, "Temporary failure" },
        {  42, "Congestion" },
        {  47, "Recources unavailable" },
        {  50, "Requested facility not subscribed" },
        {  69, "Requested facility not implemented" },
        {  81, "Invalid short message transfer reference value" },
        {  95, "Invalid message unspecified" },
        {  96, "Invalid mandatory information" },
        {  97, "Message type non existent or not implemented" },
        {  98, "Message not compatible with short message protocol" },
        {  99, "Information element non-existent or not implemente" },
        { 111, "Protocol error, unspecified" },
        { 127, "Internetworking , unspecified" },
        { 128, "Telematic internetworking not supported" },
        { 129, "Short message type 0 not supported" },
        { 130, "Cannot replace short message" },
        { 143, "Unspecified TP-PID error" },
        { 144, "Data code scheme not supported" },
        { 145, "Message class not supported" },
        { 159, "Unspecified TP-DCS error" },
        { 160, "Command cannot be actioned" },
        { 161, "Command unsupported" },
        { 175, "Unspecified TP-Command error" },
        { 176, "TPDU not supported" },
        { 192, "SC busy" },
        { 193, "No SC subscription" },
        { 194, "SC System failure" },
        { 195, "Invalid SME address" },
        { 196, "Destination SME barred" },
        { 197, "SM Rejected-Duplicate SM" },
        { 198, "TP-VPF not supported" },
        { 199, "TP-VP not supported" },
        { 208, "D0 SIM SMS Storage full" },
        { 209, "No SMS Storage capability in SIM" },
        { 210, "Error in MS" },
        { 211, "Memory capacity exceeded" },
        { 212, "Sim application toolkit busy" },
        { 213, "SIM data download error" },
        { 255, "Unspecified error cause" },
        { 300, "ME Failure" },
        { 301, "SMS service of ME reserved" },
        { 302, "Operation not allowed" },
        { 303, "Operation not supported" },
        { 304, "Invalid PDU mode parameter" },
        { 305, "Invalid Text mode parameter" },
        { 310, "SIM not inserted" },
        { 311, "SIM PIN required" },
        { 312, "PH-SIM PIN required" },
        { 313, "SIM failure" },
        { 314, "SIM busy" },
        { 315, "SIM wrong" },
        { 316, "SIM PUK required" },
        { 317, "SIM PIN2 required" },
        { 318, "SIM PUK2 required" },
        { 320, "Memory failure" },
        { 321, "Invalid memory index" },
        { 322, "Memory full" },
        { 330, "SMSC address unknown" },
        { 331, "No network service" },
        { 332, "Network timeout" },
        { 340, "No +CNMA expected" },
        { 500, "Unknown error" },
        { 512, "User abort" },
        { 513, "Unable to store" },
        { 514, "Invalid Status" },
        { 515, "Device busy or Invalid Character in string" },
        { 516, "Invalid length" },
        { 517, "Invalid character in PDU" },
        { 518, "Invalid parameter" },
        { 519, "Invalid length or character" },
        { 520, "Invalid character in text" },
        { 521, "Timer expired" },
        { 522, "Operation temporary not allowed" },
        { 532, "SIM not ready" },
        { 534, "Cell Broadcast error unknown" },
        { 535, "Protocol stack busy" },
        { 538, "Invalid parameter " },
        { -1 , "" }
    };


    msgFound = false;
    currIdx = 0;

    do
    {
        if( cms_error[currIdx].errcode == _cmsLastError )
        {
            msgFound = true;
            errmsg = STRING(cms_error[currIdx].pMessage);
        }
    
    } while( !msgFound && cms_error[currIdx++].errcode >= 0 );

    if( msgFound )
    {
        retVal = GSMDEVICE_SUCCESS;
    }
    else
    {
        retVal = GSMDEVICE_E_CMS_UNKNOWN;
    }

    return( _lastError = retVal );
}

// ////////////////////////////////////////////////////////////////////////
//
// parse response 
//   - parse response from a attached gsm device
//
// Expected arguments:
// - STRING response    holds response string
// - STRING expect      holds the expected result string
// - STRING &result     reference to hold result string
// 
// Returns an INT16 as status code:
// - GSMDEVICE_SUCCESS on succes, or
// depending of the failure that occurred 
// - GSMDEVICE_E_INIT       instance is not initialized
// - GSMDEVICE_E_MATCH      no match, result contains parsed data
// - GSMDEVICE_E_SUPPORTED  not supported
//
// ////////////////////////////////////////////////////////////////////////
//
INT16 gsmDevice::parseResponse( STRING response, STRING expect, STRING &result )
{
    INT16 retVal;

    if( _cmdPortType  == nodev ||
        _devStatus    == created )
    {
        retVal = GSMDEVICE_E_INIT;
    }
    else
    {
        if(response.indexOf(expect) > 0)
        {
            retVal = GSMDEVICE_SUCCESS;
        }
        else
        {
            retVal = GSMDEVICE_E_MATCH;
        }
    }

    return( _lastError = retVal );
}

// ////////////////////////////////////////////////////////////////////////
//
// scan cms error 
//   - parse response to get CMS error as a numeric value
//
// Expected arguments:
// - STRING response    holds response string
// - INT16 &errNo       reference to hold scanned error number
// 
// Returns an INT16 as status code:
// - GSMDEVICE_SUCCESS on succes, or
// depending of the failure that occurred 
// - GSMDEVICE_E_MATCH      no match, errNo is not useable
//
// ////////////////////////////////////////////////////////////////////////
//
INT16 gsmDevice::scanCMSErrNum( STRING response, INT16 &errNo )
{
    INT16 retVal;
    char *_tmpRawString;
    INT16 errorIndex;
    int scanValue;

    if( (errorIndex = response.indexOf(GSMDEVICE_E_CMS_MSG >= 0)) )
    {
// fprintf(stderr, "Data index of CMS err message is %d\n", errorIndex);

        if( errorIndex + strlen(GSMDEVICE_E_CMS_MSG) < response.length() )
        {
            if( (_tmpRawString = response.c_str()) != NULL )
            {
                _tmpRawString += errorIndex;  // points after error string
                if( sscanf(_tmpRawString, GSMDEVICE_E_CMS_FMT_STR, &scanValue) == 1 )
                {
                    errNo = scanValue;
                    retVal = GSMDEVICE_SUCCESS;
                }
                else
                {
                    retVal = GSMDEVICE_E_RESULT;
                }
            }
            else
            {
                retVal = GSMDEVICE_E_P_NULL;
            }
        }
        else
        {
            retVal = GSMDEVICE_E_TOO_SHORT;
        }
 
    }
    else
    {
        retVal = GSMDEVICE_E_PATTERN;
    }

    return( _lastError = retVal );
}

// ////////////////////////////////////////////////////////////////////////
//
// scan cme error 
//   - parse response to get CME error as a numeric value
//
// Expected arguments:
// - STRING response    holds response string
// - INT16 &errNo       reference to hold scanned error number
// 
// Returns an INT16 as status code:
// - GSMDEVICE_SUCCESS on succes, or
// depending of the failure that occurred 
// - GSMDEVICE_E_MATCH      no match, errNo is not useable
//
// ////////////////////////////////////////////////////////////////////////
//
INT16 gsmDevice::scanCMEErrNum( STRING response, INT16 &errNo )
{
    INT16 retVal;
    char *_tmpRawString;
    INT16 errorIndex;
    int scanValue;

    if( (errorIndex = response.indexOf(GSMDEVICE_E_CME_MSG)) >= 0 )
    {
// fprintf(stderr, "Data index of CME err message is %d\n", errorIndex);

        if( errorIndex + strlen(GSMDEVICE_E_CME_MSG) < response.length() )
        {
            if( (_tmpRawString = response.c_str()) != NULL )
            {
                _tmpRawString += errorIndex;  // points after error string
// fprintf(stderr, "scan for CME err message >%s\n", _tmpRawString );
                if( sscanf(_tmpRawString, GSMDEVICE_E_CME_FMT_STR, &scanValue) == 1 )
                {
                    errNo = scanValue;
                    retVal = GSMDEVICE_SUCCESS;
                }
                else
                {
                    retVal = GSMDEVICE_E_RESULT;
                }
            }
            else
            {
                retVal = GSMDEVICE_E_P_NULL;
            }
        }
        else
        {
            retVal = GSMDEVICE_E_TOO_SHORT;
        }
 
    }
    else
    {
        retVal = GSMDEVICE_E_PATTERN;
    }

    return( _lastError = retVal );

}

// ////////////////////////////////////////////////////////////////////////
//
// get data index
//   - parse response and get position of parameters
//
// Expected arguments:
// - STRING response     holds response string
// - char _pattern[]     pattern will be stored here
// - INT16 patternLength give lenght of char array _pattern
// - INT16 *pIndex       points to an INT16 to hold index within string
// 
// Returns an INT16 as status code:
// - GSMDEVICE_SUCCESS on succes, or
// depending of the failure that occurred 
// - GSMDEVICE_E_PATTERN  no pattern in response found
// - GSMDEVICE_E_P_NULL   response is empty string or NULL
// - GSMDEVICE_E_RESULT   no result found
//
// ////////////////////////////////////////////////////////////////////////
//
INT16 gsmDevice::getDataIndex( STRING response, char _pattern[], INT16 patternLength, INT16 *pIndex )
{
    INT16 retVal;
    INT16 echoIndex;
    INT16 dataIndex;
    INT16 patternIndex;
    char *_tmpRawString;
    BOOL done;
    BOOL fail;

    retVal = GSMDEVICE_SUCCESS;

    if( (echoIndex = response.indexOf("+")) >= 0)
    {
// fprintf(stderr, "Data index of + is %d\n", echoIndex);

        memset( _pattern, '\0', patternLength );

        if( (_tmpRawString = response.c_str()) != NULL )
        {
            _tmpRawString += echoIndex;  // points to +

            for( done = false, fail = false, patternIndex = 0;
                 !done && !fail && *_tmpRawString != '\0'; _tmpRawString++  )
            {
                if( (*_tmpRawString >= 'A' && *_tmpRawString <= 'Z') ||
                     *_tmpRawString == '+' )
                {
                    // ok, echo of command
                    _pattern[patternIndex] = *_tmpRawString;
                    _pattern[patternIndex+1] = ':';
                    if( patternIndex < (patternLength-1) )
                    {
                        patternIndex++;
                    }
                    else
                    {
                        fail = true;
                    }
                }
                else
                {
                    done = true;
                }
            }

            if( fail )
            {
// fprintf(stderr, "pattern get failed!\n");
                retVal = GSMDEVICE_E_PATTERN;
            }
            else
            {
                if( done )
                {
// fprintf(stderr, "pattern found: %s\n", _pattern);

                    if( _pattern[0] == '+' )
                    {
                        if( (dataIndex = response.indexOf(_pattern)) >= 0)
                        {
// fprintf(stderr, "Results present!\n");
                            if( (_tmpRawString = response.c_str()) != NULL )
                            {
                                _tmpRawString += dataIndex;
// fprintf(stderr, "Results go from here >%s", _tmpRawString);
                                *pIndex = dataIndex;
                                retVal = GSMDEVICE_SUCCESS;
                            }
                            else
                            {
// fprintf(stderr, "Geeks ... NULL!\n");
                                *pIndex = -1;
                                retVal = GSMDEVICE_E_P_NULL;
                            }
                        }
                        else
                        {
// fprintf(stderr, "NO Results present!\n");
                            *pIndex = -1;
                            retVal = GSMDEVICE_E_RESULT;
                        }
                    }
                    else
                    {
// fprintf(stderr, "Missing trailing '+' ... expecting no results!\n");
                    }
                }
                else
                {
// fprintf(stderr, "pattern get failed!\n");
// fprintf(stderr, "pattern fragment: [%s]\n", _pattern);
                    retVal = GSMDEVICE_E_PATTERN;
                }
            }
        }
        else
        {
            retVal = GSMDEVICE_E_P_NULL;
        }
    }
    else
    {
        retVal = GSMDEVICE_E_PATTERN;
    }

    return( _lastError = retVal );
}

// ////////////////////////////////////////////////////////////////////////
//
// parseCMGF
//   - parse response of AT+CMGF command
//
// Expected arguments:
// - gsmCommandMode cmdMode  mode of call
// - STRING response         holds response string
// - char _pattern[]         pattern will be stored here
// - INT16 dataIndex         index within string
// - struct _dataCMGF *pData points to data structure
// 
// Returns an INT16 as status code:
// - GSMDEVICE_SUCCESS on succes, or
// depending of the failure that occurred 
// - GSMDEVICE_E_RESULT    no result found
// - GSMDEVICE_E_INVAL     invalid result parameter
// - GSMDEVICE_E_TOO_SHORT response string too short
// - GSMDEVICE_E_CMD_MODE  command mode invalid
// - GSMDEVICE_E_P_NULL    response is empty string or NULL
//
// - GSMDEVICE_E_PATTERN  no pattern in response found
// ////////////////////////////////////////////////////////////////////////
//
INT16 gsmDevice::parseCMGF( gsmCommandMode cmdMode, STRING response, char _pattern[], INT16 dataIndex, struct _dataCMGF *pData )
{
    INT16 retVal;
    char *_tmpRawString;

    if( pData != NULL )
    {
        if( (dataIndex+strlen(_pattern)) < response.length() )
        {
            if( (_tmpRawString = response.c_str()) != NULL )
            {
                _tmpRawString += dataIndex;

                if( *_tmpRawString != _pattern[0] )
                {
// fprintf(stderr, "seems to be no data >%s\n", _tmpRawString );
                    retVal = GSMDEVICE_E_RESULT;
                }
                else
                {
// fprintf(stderr, "Go skipping pattern [%s] >%s\n", _pattern, _tmpRawString ); 
                    _tmpRawString += strlen(_pattern);
// fprintf(stderr, "skipped pattern >%s\n",  _tmpRawString ); 

                    switch( cmdMode )
                    {
                        case cmd_test:
                            if( sscanf( _tmpRawString, testResponseFmtCMGF, &pData->from, &pData->to ) == 2 )
                            {
                                switch( pData->from )
                                {
                                    case smsPDUMode:
                                    case smsTXTMode:
// fprintf(stderr, "VALID cmgfMode 1 found: %d\n", pData->from); 
                                        switch( pData->to )
                                        {
                                            case smsPDUMode:
                                            case smsTXTMode:
// fprintf(stderr, "VALID cmgfMode 2 found: %d\n", pData->to); 
                                                retVal = GSMDEVICE_SUCCESS;
                                                break;
                                            default:
// fprintf(stderr, "NO VALID cmgfMode found\n"); 
                                                retVal = GSMDEVICE_E_INVAL;
                                                break;
                                        }
                                        break;
                                    default:
                                        fprintf(stderr, "NO VALID cmgfMode found\n"); 
                                        retVal = GSMDEVICE_E_INVAL;
                                        break;
                                }
                            }
                            else
                            {
// fprintf(stderr, "seems to be no cmgfMode\n"); 
                                retVal = GSMDEVICE_E_RESULT;
                            }
                            break;
                        case cmd_read:
                            if( sscanf( _tmpRawString, readResponseFmtCMGF, (int*) &pData->current ) == 1 )
                            {
                                switch( pData->current )
                                {
                                    case smsPDUMode:
                                    case smsTXTMode:
// fprintf(stderr, "VALID cmgfMode found: %d\n", pData->current); 
                                        retVal = GSMDEVICE_SUCCESS;
                                        break;
                                    default:
// fprintf(stderr, "NO VALID cmgfMode found: %d\n", pData->current); 
                                        retVal = GSMDEVICE_E_INVAL;
                                        break;
                                }
                            }
                            else
                            {
// fprintf(stderr, "seems to be no cmgfMode\n"); 
                                retVal = GSMDEVICE_E_RESULT;
                            }
                            break;
                        case cmd_set:
                        case cmd_execute:
                            break;
                        default:
                            retVal = GSMDEVICE_E_CMD_MODE;
                    }
                }
            }
            else
            {
                retVal = GSMDEVICE_E_P_NULL;
            }
        }
        else
        {
            retVal = GSMDEVICE_E_TOO_SHORT;
        }
    }
    else
    {
        retVal = GSMDEVICE_E_P_NULL;
    }


    return( _lastError = retVal );
}

// ////////////////////////////////////////////////////////////////////////
//
// parseCSQ
//   - parse response of AT+CSQ command
//
// Expected arguments:
// - gsmCommandMode cmdMode  mode of call
// - STRING response         holds response string
// - char _pattern[]         pattern will be stored here
// - INT16 dataIndex         index within string
// - struct _dataCSQ *pData  points to data structure
// 
// Returns an INT16 as status code:
// - GSMDEVICE_SUCCESS on succes, or
// depending of the failure that occurred 
// - GSMDEVICE_E_RESULT    no result found
// - GSMDEVICE_E_INVAL     invalid result parameter
// - GSMDEVICE_E_TOO_SHORT response string too short
// - GSMDEVICE_E_CMD_MODE  command mode invalid
// - GSMDEVICE_E_P_NULL    response is empty string or NULL
//
// - GSMDEVICE_E_PATTERN  no pattern in response found
// ////////////////////////////////////////////////////////////////////////
//
INT16 gsmDevice::parseCSQ( gsmCommandMode cmdMode, STRING response, char _pattern[], INT16 dataIndex, struct _dataCSQ *pData )
{
    INT16 retVal;
    char *_tmpRawString;

    if( pData != NULL )
    {
        if( (dataIndex+strlen(_pattern)) < response.length() )
        {
            if( (_tmpRawString = response.c_str()) != NULL )
            {
                _tmpRawString += dataIndex;

                if( *_tmpRawString != _pattern[0] )
                {
// fprintf(stderr, "seems to be no data >%s\n", _tmpRawString );
                    retVal = GSMDEVICE_E_RESULT;
                }
                else
                {
// fprintf(stderr, "Go skipping pattern [%s] >%s\n", _pattern, _tmpRawString ); 
                    _tmpRawString += strlen(_pattern);
// fprintf(stderr, "skipped pattern >%s\n",  _tmpRawString ); 

                    switch( cmdMode )
                    {
                        case cmd_test:
                            if( sscanf( _tmpRawString, testResponseFmtCSQ, &pData->rssiFrom, &pData->rssiTo,
                                        &pData->rssiUnknown, &pData->berFrom, &pData->berTo, &pData->berUnknown ) == 6 )
                            {

// fprintf(stderr, "VALID CSQ data found for test mode\n");
// fprintf(stderr, "     rssiFrom: %d, rssiTo: %d, rssiUnknown: %d\n", pData->rssiFrom, pData->rssiTo, pData->rssiUnknown );
// fprintf(stderr, "     berFrom: %d, berTo: %d, berUnknown: %d\n", pData->berFrom, pData->berTo, pData->berUnknown );

                                retVal = GSMDEVICE_SUCCESS;
                            }
                            else
                            {
                                retVal = GSMDEVICE_E_RESULT;
                            }
                            break;
                        case cmd_read:
                        case cmd_set:
                            break;
                        case cmd_execute:
                            if( sscanf( _tmpRawString, executeResponseFmtCSQ, &pData->rssi, &pData->ber ) == 2 )
                            {
// fprintf(stderr, "VALID CSQ data found for execute mode\n");
// fprintf(stderr, "     rssi: %d, ber: %d\n", pData->rssi, pData->ber );

                                retVal = GSMDEVICE_SUCCESS;
                            }
                            else
                            {
                                retVal = GSMDEVICE_E_RESULT;
                            }
                            break;
                        default:
                            retVal = GSMDEVICE_E_CMD_MODE;
                    }
                }
            }
            else
            {
                retVal = GSMDEVICE_E_P_NULL;
            }
        }
        else
        {
            retVal = GSMDEVICE_E_TOO_SHORT;
        }
    }
    else
    {
        retVal = GSMDEVICE_E_P_NULL;
    }

    return( _lastError = retVal );
}

// ////////////////////////////////////////////////////////////////////////
//
// parseIMSI
//   - parse response of AT+IMSI command
//
// Expected arguments:
// - gsmCommandMode cmdMode   mode of call
// - STRING response          holds response string
// - struct _dataIMSI *pData  points to data structure
// 
// Returns an INT16 as status code:
// - GSMDEVICE_SUCCESS on succes, or
// depending of the failure that occurred 
// - GSMDEVICE_E_RESULT    no result found
// - GSMDEVICE_E_INVAL     invalid result parameter
// - GSMDEVICE_E_CMD_MODE  command mode invalid
// - GSMDEVICE_E_P_NULL    response is empty string or NULL
//
// ////////////////////////////////////////////////////////////////////////
//
INT16 gsmDevice::parseIMSI( gsmCommandMode cmdMode, STRING response, struct _dataIMSI *pData )
{
    INT16 retVal;
    char *_tmpRawString;
    int maxLen;
    BOOL done;

    if( pData != NULL )
    {
        if( (_tmpRawString = response.c_str()) != NULL )
        {
            switch( cmdMode )
            {
                case cmd_test:
                case cmd_read:
                    break;
                case cmd_set:
                    pData->_length = 0;
                    memset( pData->_raw, '\0', GSM_MAX_IMSI_LENGTH+1 );

                    maxLen = strlen( _tmpRawString );

// fprintf(stderr, "lookup for IMSI ->%s\n", _tmpRawString); 
                    if( strncmp( _tmpRawString, GSMDEVICE_ATTENTION, strlen(GSMDEVICE_ATTENTION) ) == 0 )
                    {
// fprintf(stderr, "skipping echo\n"); 
                        done = false;
                        do
                        {
                            if( *_tmpRawString++ == '\n' )
                            {
                                done = true;
                            }

                        } while(!done && maxLen-- > 0 );
                    }
// fprintf(stderr, "now lookup for IMSI ->%s\n", _tmpRawString); 

                    if( sscanf( _tmpRawString, setResponseFmtIMSI, pData->_raw ) )
                    {
// fprintf(stderr, "VALID IMSI found for set mode[%s]\n", pData->_raw);
                        pData->_length = strlen( pData->_raw );
                        retVal = GSMDEVICE_SUCCESS;
                    }
                    else
                    {
// fprintf(stderr, "NO VALID IMSI found for set mode\n");
                        retVal = GSMDEVICE_E_RESULT;
                    }
                    break;
                case cmd_execute:
                    break;
                default:
                    retVal = GSMDEVICE_E_CMD_MODE;
            }
        }
        else
        {
            retVal = GSMDEVICE_E_P_NULL;
        }
    }
    else
    {
        retVal = GSMDEVICE_E_P_NULL;
    }

    return( _lastError = retVal );
}

// ////////////////////////////////////////////////////////////////////////
//
// parseEGMR
//   - parse response of AT+EGMR command
//
// Expected arguments:
// - gsmCommandMode cmdMode   mode of call
// - STRING response          holds response string
// - struct _dataEGMR *pData  points to data structure
// 
// Returns an INT16 as status code:
// - GSMDEVICE_SUCCESS on succes, or
// depending of the failure that occurred 
// - GSMDEVICE_E_RESULT    no result found
// - GSMDEVICE_E_INVAL     invalid result parameter
// - GSMDEVICE_E_CMD_MODE  command mode invalid
// - GSMDEVICE_E_P_NULL    response is empty string or NULL
//
// ////////////////////////////////////////////////////////////////////////
//
INT16 gsmDevice::parseEGMR( gsmCommandMode cmdMode, STRING response, char _pattern[], INT16 dataIndex, struct _dataEGMR *pData )
{
    INT16 retVal;
    char *_tmpRawString;


    if( pData != NULL )
    {
        if( (dataIndex+strlen(_pattern)) < response.length() )
        {
            if( (_tmpRawString = response.c_str()) != NULL )
            {
                _tmpRawString += dataIndex;

                if( *_tmpRawString != _pattern[0] )
                {
// fprintf(stderr, "seems to be no data >%s\n", _tmpRawString );
                    retVal = GSMDEVICE_E_RESULT;
                }
                else
                {
// fprintf(stderr, "Go skipping pattern [%s] >%s\n", _pattern, _tmpRawString ); 
                    _tmpRawString += strlen(_pattern);
// fprintf(stderr, "skipped pattern >%s\n",  _tmpRawString ); 

                    switch( cmdMode )
                    {
                        case cmd_test:
                            break;
                        case cmd_read:
                            break;
                        case cmd_set:
// fprintf(stderr, "now lookup for IMEI ->%s\n", _tmpRawString); 

                            if( sscanf( _tmpRawString, setResponseFmtIMSI, pData->_raw ) )
                            {
// fprintf(stderr, "VALID IMEI found for set mode[%s]\n", pData->_raw);
                                pData->_length = strlen( pData->_raw );
                                retVal = GSMDEVICE_SUCCESS;
                            }
                            else
                            {
// fprintf(stderr, "NO VALID IMEI found for set mode\n");
                                retVal = GSMDEVICE_E_RESULT;
                            }
                            break;
                        case cmd_execute:
                            break;
                        default:
                            retVal = GSMDEVICE_E_CMD_MODE;
                    }
                }
            }
            else
            {
                retVal = GSMDEVICE_E_P_NULL;
            }
        }
        else
        {
            retVal = GSMDEVICE_E_TOO_SHORT;
        }
    }
    else
    {
        retVal = GSMDEVICE_E_P_NULL;
    }


    return( _lastError = retVal );
}


// ////////////////////////////////////////////////////////////////////////
//
// remove echo
//   - remove leading command echo
//
// Expected arguments:
// - STRING result           resultstring to scan
// - STRING &dummy           no function yet
// 
// Returns an INT16 as status code:
// - GSMDEVICE_SUCCESS on succes, or
// depending of the failure that occurred 
// - GSMDEVICE_E_CMS  a CMS error occured
// - GSMDEVICE_E_CME  a CME error occured
// - GSMDEVICE_ERROR  some error, not specified
//
// ////////////////////////////////////////////////////////////////////////
//
INT16 gsmDevice::removeEcho( STRING &result, STRING &dummy )
{
    INT16 retVal;
    char *_tmpRawString;
    int maxLen;
    BOOL done;

    if( (_tmpRawString = result.c_str()) != NULL )
    {
        maxLen = strlen( _tmpRawString );

        if( strncmp( _tmpRawString, GSMDEVICE_ATTENTION, strlen(GSMDEVICE_ATTENTION) ) == 0 )
        {
            done = false;
            do
            {
                if( *_tmpRawString++ == '\n' )
                {
                    done = true;
                    while( maxLen > 0 && (
                           *_tmpRawString == '\n' ||
                           *_tmpRawString == '\r' ||
                           *_tmpRawString == ' ' ) )
                     {

                         _tmpRawString++;
                         maxLen--;
                     }
                }

            } while(!done && maxLen-- > 0 );

            if( (maxLen = strlen( _tmpRawString ) - 1) > 0 )
            {
                done = false;

                do
                {
                    switch( _tmpRawString[maxLen] ) 
                    {
                        case '\r':
                        case '\n':
                        case 'O':
                        case 'K':
                            _tmpRawString[maxLen] = '\0';
                            break;
                        default:
                            done = true;
                    }
                }
                while(!done && --maxLen > 0 );
            }
            result =  _tmpRawString; 
        }

        retVal = GSMDEVICE_SUCCESS;
    }
    else
    {
        retVal = GSMDEVICE_E_P_NULL;
    }

    return( _lastError = retVal );
}
// ////////////////////////////////////////////////////////////////////////
//
// parseCREG
//   - parse response of AT+CREG command
//
// Expected arguments:
// - gsmCommandMode cmdMode  mode of call
// - STRING response         holds response string
// - char _pattern[]         pattern will be stored here
// - INT16 dataIndex         index within string
// - struct _dataCREG *pData points to data structure
// 
// Returns an INT16 as status code:
// - GSMDEVICE_SUCCESS on succes, or
// depending of the failure that occurred 
// - GSMDEVICE_E_RESULT    no result found
// - GSMDEVICE_E_INVAL     invalid result parameter
// - GSMDEVICE_E_TOO_SHORT response string too short
// - GSMDEVICE_E_CMD_MODE  command mode invalid
// - GSMDEVICE_E_P_NULL    response is empty string or NULL
//
// - GSMDEVICE_E_PATTERN  no pattern in response found
// ////////////////////////////////////////////////////////////////////////
//
INT16 gsmDevice::parseCREG( gsmCommandMode cmdMode, STRING response, char _pattern[], INT16 dataIndex, struct _dataCREG *pData )
{
    INT16 retVal;
    char *_tmpRawString;
    int items;

    if( pData != NULL )
    {
        if( (dataIndex+strlen(_pattern)) < response.length() )
        {
            if( (_tmpRawString = response.c_str()) != NULL )
            {
                _tmpRawString += dataIndex;

                if( *_tmpRawString != _pattern[0] )
                {
// fprintf(stderr, "seems to be no data >%s\n", _tmpRawString );
                    retVal = GSMDEVICE_E_RESULT;
                }
                else
                {
// fprintf(stderr, "Go skipping pattern [%s] >%s\n", _pattern, _tmpRawString ); 
                    _tmpRawString += strlen(_pattern);
// fprintf(stderr, "skipped pattern >%s\n",  _tmpRawString ); 

                    switch( cmdMode )
                    {
                        case cmd_test:
                            memset( pData->lac, '\0', GSM_BASESTATION_LAC_LENGTH+1);
                            memset( pData->ci, '\0', GSM_BASESTATION_CI_LENGTH+1);

                            if( (items = sscanf( _tmpRawString, testResponseFmtCREG, &pData->fromMode, 
                                        &pData->toMode )) >= 2 )
                            {
// fprintf(stderr, "VALID CREG data found for test mode\n");
// fprintf(stderr, "     from mode: %d, to mode: %d\n", pData->fromMode, pData->toMode);
                                retVal = GSMDEVICE_SUCCESS;
                            }
                            else
                            {
                                retVal = GSMDEVICE_E_RESULT;
                            }
                            break;
                        case cmd_read:
                            memset( pData->lac, '\0', GSM_BASESTATION_LAC_LENGTH+1);
                            memset( pData->ci, '\0', GSM_BASESTATION_CI_LENGTH+1);

                            if( (items = sscanf( _tmpRawString, readResponseFmtCREG, (int*) &pData->mode, 
                                        &pData->stat, pData->lac, pData->ci )) == 4 ||
                                 items == 2 )
                            {
// fprintf(stderr, "VALID CREG data found for test mode\n");
// fprintf(stderr, "     mode: %d, stat: %d, lac: %s, ci: %s\n", pData->mode, pData->stat, pData->lac, pData->ci );
                                retVal = GSMDEVICE_SUCCESS;
                            }
                            else
                            {
                                retVal = GSMDEVICE_E_RESULT;
                            }
                            break;
                        case cmd_set:
                        case cmd_execute:
                            break;
                        default:
                            retVal = GSMDEVICE_E_CMD_MODE;
                    }
                }
            }
            else
            {
                retVal = GSMDEVICE_E_P_NULL;
            }
        }
        else
        {
            retVal = GSMDEVICE_E_TOO_SHORT;
        }
    }
    else
    {
        retVal = GSMDEVICE_E_P_NULL;
    }


    return( _lastError = retVal );
}
























// 
// -------------------NOTHING IMPORTAN BEHIND THIS LINE -----------------
// 
#ifdef NEVERDEF

COMMAND NO RESPONSE!


TEMPLATE FOR SCANNER


// ////////////////////////////////////////////////////////////////////////
//
// parseCSQ
//   - parse response of AT+CSQ command
//
// Expected arguments:
// - gsmCommandMode cmdMode  mode of call
// - STRING response         holds response string
// - char _pattern[]         pattern will be stored here
// - INT16 dataIndex         index within string
// - struct _dataCSQ *pData  points to data structure
// 
// Returns an INT16 as status code:
// - GSMDEVICE_SUCCESS on succes, or
// depending of the failure that occurred 
// - GSMDEVICE_E_RESULT    no result found
// - GSMDEVICE_E_INVAL     invalid result parameter
// - GSMDEVICE_E_TOO_SHORT response string too short
// - GSMDEVICE_E_CMD_MODE  command mode invalid
// - GSMDEVICE_E_P_NULL    response is empty string or NULL
//
// - GSMDEVICE_E_PATTERN  no pattern in response found
// ////////////////////////////////////////////////////////////////////////
//
INT16 gsmDevice::parseCSQ( gsmCommandMode cmdMode, STRING response, char _pattern[], INT16 dataIndex, struct _dataCSQ *pData )
{
    INT16 retVal;
    char *_tmpRawString;

    int cmgfMode;
    int cmgfMode1, cmgfMode2;

    if( pData != NULL )
    {
        if( (dataIndex+strlen(_pattern)) < response.length() )
        {
            if( (_tmpRawString = response.c_str()) != NULL )
            {
                _tmpRawString += dataIndex;

                if( *_tmpRawString != _pattern[0] )
                {
// fprintf(stderr, "seems to be no data >%s\n", _tmpRawString );
                    retVal = GSMDEVICE_E_RESULT;
                }
                else
                {
// fprintf(stderr, "Go skipping pattern [%s] >%s\n", _pattern, _tmpRawString ); 
                    _tmpRawString += strlen(_pattern);
// fprintf(stderr, "skipped pattern >%s\n",  _tmpRawString ); 

                    switch( cmdMode )
                    {
                        case cmd_test:
                            break;
                        case cmd_read:
                            break;
                        case cmd_set:
                        case cmd_execute:
                            break;
                        default:
                            retVal = GSMDEVICE_E_CMD_MODE;
                    }
                }
            }
            else
            {
                retVal = GSMDEVICE_E_P_NULL;
            }
        }
        else
        {
            retVal = GSMDEVICE_E_TOO_SHORT;
        }
    }
    else
    {
        retVal = GSMDEVICE_E_P_NULL;
    }


    return( _lastError = retVal );
}



// 
// -------------------NOTHING IMPORTAN BEHIND THIS LINE -----------------
// 
#endif // NEVERDEF




