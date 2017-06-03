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
//
//
// -------- History -------------------------------------------------------
//
// 1st version: 05/22/17
//         basic function
// update:
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
#endif // linux

// #include <inttypes.h>

#include "gsmDevice.h"


// 
// ----------------------------- GLOBAL STUFF -----------------------------
//

#ifdef linux

//
// ----------------------------------------------------------------------
//

// ----------------------------------------------------------------------
// void sigHandler (int status)
//
// handler for I/O signal asynchronous receive

// ------------------------------------------------------------------------
// Create a gsmDevice instance 
//   - initial object creation
//
// Expected arguments:
// - none
//
// Returns:
// - nothing
//
// ------------------------------------------------------------------------
//
void gsmDevice::sigHandler (int status)
{
  if( !dataAvailable )
  {
    dataAvailable = true;
  }
}
   


 
// ------------------------------------------------------------------------
// int setupSerial(int fd, int speed)
//
// setup serial port 
// ------------------------------------------------------------------------
int gsmDevice::setupSerial(int fd, int speed)
{
  struct termios tty;
  int retVal = -1;

  if (tcgetattr(fd, &tty) < 0) 
  {
    printf("Error from tcgetattr: %s\n", strerror(errno));
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
      printf("Error from tcsetattr: %s\n", strerror(errno));
    }
    else
    {
      retVal = 0;
    }
  }

  return( retVal );
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
      printf("Error tcgetattr: %s\n", strerror(errno));
  }
  else
  {
    tty.c_cc[VMIN] = mcount ? 1 : 0;
    tty.c_cc[VTIME] = 5;        /* half second timer */

    if (tcsetattr(fd, TCSANOW, &tty) < 0)
    {
      printf("Error tcsetattr: %s\n", strerror(errno));
    }
  }
}




long millis()
{
    return 0L;
}

void delay(long msec)
{

}



String::~String()
{
    if( pData != NULL )
    {
        free(pData);
    }
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
        for( int i = strlen(pData); !done && i > 0; i-- )
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

        for( int i = 0; !done && i < strlen(pData); i++ )
        {
            switch(pData[i])
            {
                case ' ':
                case '\t':
                    break;
                default:
                    done = 1;
                    startIdx = i;
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
    }
    return( 0 );
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
    minorRelease = 4;

#ifdef linux

    void sigHandler (int status); 

    dataAvailable = false;
    /* install the signal handler before making the device asynchronous */
    saio.sa_handler = sigHandler;
    sigemptyset(&saio.sa_mask);
    saio.sa_flags = 0;
    saio.sa_restorer = NULL;
    sigaction(SIGIO,&saio,NULL);
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
// - GSMDEVICE_E_P_SERIAL invalid value for # of hw serial
// - GSMDEVICE_E_P_SPEED  invalid baudrate
// - GSMDEVICE_E_P_TMOUT  invalid timeout value
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

    if( deviceName.length() ) // <= NO_SERIAL )
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


                if( (_cmdPort._dev   =  open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_SYNC)) < 0 )
                {
                  printf("Error opening %s: %s\n", "/dev/ttyUSB0", strerror(errno));
                  return -1;
                }

                /* 38400N81 */
                setupSerial(_cmdPort._dev, B9600);

                //setSerialMin(_cmdPort._dev, 0);                /* set to pure timed read */
                fcntl(_cmdPort._dev, F_SETOWN, getpid());
                fcntl(_cmdPort._dev, F_SETFL, FASYNC);

                _cmdPortType    = linuxDevice;
                _ownStream      = false;
                _cmdPortSpeed   = speed;
                _cmdPortTimeout = timeout;
                _devStatus      = initialized;

                if( speed != NO_SPEED )
                {
//                    _cmdPort._hw->begin(speed);
                }
                else
                {
//                    _cmdPort._hw->begin( GSMDEVICE_DEF_CMD_SPEED );
                }

                if( timeout != NO_TIMEOUT )
                {
//                    _cmdPort._hw->setTimeout(timeout);
                }

                retVal = GSMDEVICE_SUCCESS;
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
INT16 gsmDevice::flush()
{
    int retVal;

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
// set sms format
// - set the format of short messages
//
// Expected arguments:
// - gsmCommandMode cmdMode   get or set
// - smsMessageFormat *pFmt   holds new/current value
// - STRING &result           reference to hold result string
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
INT16 gsmDevice::smsMsgFormat( gsmCommandMode cmdMode, smsMessageFormat *pFmt, 
                               STRING &result )
{
    int retVal;
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
                command = STRING(SMS_MSG_FORMAT_CMD_TEST) + STRING(CRLF_STRING);
                retVal = GSMDEVICE_SUCCESS;
                break;
            case cmd_get:
                command = STRING(SMS_MSG_FORMAT_CMD_GET) + STRING(CRLF_STRING);
                retVal = GSMDEVICE_SUCCESS;
                break;
            case cmd_set:
                if( *pFmt == smsPDUMode ||
                    *pFmt == smsTXTMode    )
                {
                    command = STRING(SMS_MSG_FORMAT_CMD_SET);
                    command += STRING(*pFmt);
                    command += STRING(CRLF_STRING);
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
                    if((retVal=sendCommand(command, true)) == GSMDEVICE_SUCCESS)
                    {
                        long lastCall = millis();
                        do
                        {
                            if((retVal = readResponse( result, false )) != 
                                GSMDEVICE_SUCCESS )
                            {
                                delay(GSMDEV_READ_DELAY);
                            }

                        } while( retVal != GSMDEVICE_SUCCESS &&
                                 millis() -lastCall < GSMDEV_READ_TIMEOUT );

                        if( retVal == GSMDEVICE_SUCCESS )
                        {
                            // result.length()
                            if(parseResponse( result, GSMDEVICE_OK_MSG, 
                                   dummy ) != GSMDEVICE_SUCCESS)
                            {
                                if(parseResponse( result, GSMDEVICE_E_CME_MSG, 
                                       dummy ) != GSMDEVICE_SUCCESS)
                                {
                                    if(parseResponse(result, GSMDEVICE_E_CMS_MSG, 
                                           dummy ) != GSMDEVICE_SUCCESS)
                                    {
                                        retVal = GSMDEVICE_ERROR;
                                    }
                                    else
                                    {
                                        if( (retVal = scanCMSErrNum( result, 
                                             errNo )) == GSMDEVICE_SUCCESS )
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
                                    if( (retVal = scanCMEErrNum( result, 
                                         errNo )) == GSMDEVICE_SUCCESS )
                                    {
                                        _cmeLastError = errNo;
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
                        }
                    }
                    break;
                case streamType:
                case linuxDevice:
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
STRING gsmDevice::getError()
{
    STRING retVal = EMPTY_STRING;

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
        case GSMDEVICE_E_INIT:
        case GSMDEVICE_E_RESPONSE:
        case GSMDEVICE_E_SUPPORTED:
        case GSMDEVICE_E_DEV_TYPE:
        case GSMDEVICE_E_CME:
        case GSMDEVICE_E_CMS:
            break;
        default:
            break;
    }

    return( retVal );
}


// 
// ----------------------------- PRIVATE STUFF ----------------------------
//

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
    int retVal;
    int triesDone;
    bool devReady;
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

        flush();

        do
        {
            if( (triesDone % GSMDEV_SYNC_MAX_CMD) == 0 )
            {
                switch( _cmdPortType )
                {
#ifndef linux
                    case swSerial:
                        // TODO: check whether println works
                        _cmdPort._sw->println(cmd);
                        break;
                    case hwSerial:
                        // TODO: check whether println works
                        _cmdPort._hw->println(cmd);
                        break;
                    case streamType:
                        break;
#else
                    case linuxDevice:
                        break;
#endif // linux
                    case nodev:
                    default:
                        retVal = GSMDEVICE_E_INIT;
                }
            }

            switch( _cmdPortType )
            {
#ifndef linux
                case swSerial:
                    if( _cmdPort._sw->available() )
                    {
                        response = _cmdPort._sw->readString();
                        response.trim();
                        if(response.indexOf(expect) > 0)
                        {
                            devReady = true;
                        }
                    }
                    break;
                case hwSerial:
                    if( _cmdPort._hw->available() )
                    {
                        response = _cmdPort._hw->readString();
                        response.trim();
                        if(response.indexOf(expect) > 0)
                        {
                            devReady = true;
                        }
                    }
                    break;
                case streamType:
                    break;
#else // linux
                case linuxDevice:
                    break;
#endif // linux
                case nodev:
                default:
                    break;
            }

            delay(GSMDEV_SYNC_DELAY);

        } while( !devReady && triesDone++ < GSMDEV_SYNC_MAX_TOTAL );

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
INT16 gsmDevice::sendCommand( STRING cmd, BOOL flushBefore )
{
    int retVal;
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
            flush();
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
                retVal = GSMDEVICE_E_SUPPORTED;
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
INT16 gsmDevice::readResponse( STRING &response, BOOL flushAfter )
{
    int retVal;
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
                    response += _cmdPort._sw->readString();
                    response.trim();
                }
    
                if( flushAfter )
                {
                    flush();
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
                while( _cmdPort._hw->available() )
                {
                    response += _cmdPort._hw->readString();
                    response.trim();
                }
    
                if( flushAfter )
                {
                    flush();
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
                retVal = GSMDEVICE_E_SUPPORTED;
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
// retieve CME error message 
//   - read the current response of the attached gsm device
//
// Expected arguments:
// - STRING &errmsg    reference to hold error message
// - 
// 
// Returns an INT16 as status code:
// - GSMDEVICE_SUCCESS on succes, or
// depending of the failure that occurred 
// - GSMDEVICE_E_CME_NOMSG   no matching message was found
//
// ////////////////////////////////////////////////////////////////////////
//
INT16 gsmDevice::cmeErrorMsg( STRING &errmsg )
{
    int retVal;
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
        retVal = GSMDEVICE_E_CMS_NOMSG;
    }

    return( _lastError = retVal );
}

// ////////////////////////////////////////////////////////////////////////
//
// retieve CMS error message 
//   - read the current response of the attached gsm device
//
// Expected arguments:
// - STRING &errmsg    reference to hold error message
// - 
// 
// Returns an INT16 as status code:
// - GSMDEVICE_SUCCESS on succes, or
// depending of the failure that occurred 
// - GSMDEVICE_E_CMS_NOMSG   no matching message was found
//
// ////////////////////////////////////////////////////////////////////////
//
INT16 gsmDevice::cmsErrorMsg( STRING &errmsg )
{
    int retVal;
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
        retVal = GSMDEVICE_E_CMS_NOMSG;
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
    int retVal;

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
    int retVal;

    retVal = GSMDEVICE_SUCCESS;
    errNo = 22;

// #define GSMDEVICE_E_CMS_MSG       "+CMS ERROR:"
// #define GSMDEVICE_E_CMS_SCAN_STR  "+CMS ERROR:%d"

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
    int retVal;

    retVal = GSMDEVICE_SUCCESS;
    errNo = 55;

// #define GSMDEVICE_E_CME_MSG       "+CME ERROR:"
// #define GSMDEVICE_E_CME_SCAN_STR  "+CME ERROR:%d"

    return( _lastError = retVal );
}


// 
// -------------------NOTHING IMPORTAN BEHIND THIS LINE -----------------
// 
#ifdef NEVERDEF



// 
// -------------------NOTHING IMPORTAN BEHIND THIS LINE -----------------
// 
#endif // NEVERDEF



