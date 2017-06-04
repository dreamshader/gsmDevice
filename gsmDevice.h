#ifndef _GSMDEVICE_H_
#define _GSMDEVICE_H_




#ifndef linux
  #if defined(ARDUINO) && ARDUINO >= 100
    #include "Arduino.h"
  #else
    #include "WProgram.h"
  #endif
  #include <SoftwareSerial.h>
#else // on a linux platform

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


class String {

private:
    char* pData;

public:
    ~String(){};
    String();
    String( const char* p);
    String( int v );
    int length();
    int trim();
    int indexOf(String p);
    char *c_str();

    String operator+=(String p);
    String operator+=(const char *p);

    String operator+(String p);
    String operator+(const char *p);
};

#endif // linux


#ifdef __cplusplus
extern "C" {
#endif

// 
// ---------------------------- DEFINITIONS -----------------------------
//
// internal used definitions

#define GSMDEV_SYNC_MAX_CMD     15
#define GSMDEV_SYNC_MAX_TOTAL  100
#define GSMDEV_SYNC_DELAY      100
#define GSMDEV_SYNC_CMD        "AT\r\n"
#define GSMDEV_SYNC_RSP        "OK"
#define GSMDEV_READ_TIMEOUT  30000   // up to 30 sec. response time
#define GSMDEV_READ_DELAY      100

//
// parameter values

#define NULL_PIN                 0
#define NULL_STREAM           NULL
#define NO_SPEED                 0
#define NO_TIMEOUT               0
#define NO_SERIAL                0

#define GSMDEVICE_DEF_CMD_SPEED  9600

//
// unified error codes

#define GSMDEVICE_SUCCESS        0   // no error
#define GSMDEVICE_ERROR         -1   // unspecified/unknown error

#define GSMDEVICE_E_P_RXPIN    -10
#define GSMDEVICE_E_P_TXPIN    -11
#define GSMDEVICE_E_P_SPEED    -12
#define GSMDEVICE_E_P_TMOUT    -13
#define GSMDEVICE_E_P_SERIAL   -14
#define GSMDEVICE_E_P_STREAM   -15
#define GSMDEVICE_E_P_NULL     -16

#define GSMDEVICE_E_INIT       -30
#define GSMDEVICE_E_RESPONSE   -31
#define GSMDEVICE_E_SUPPORTED  -32
#define GSMDEVICE_E_DEV_TYPE   -33
#define GSMDEVICE_E_SEND       -34
#define GSMDEVICE_E_MATCH      -35
#define GSMDEVICE_E_CMD_MODE   -36
#define GSMDEVICE_E_OPEN       -37
#define GSMDEVICE_E_SETUP      -38


#define GSMDEVICE_E_CME       -100
#define GSMDEVICE_E_CME_NOMSG -101
#define GSMDEVICE_E_CMS       -200
#define GSMDEVICE_E_CMS_NOMSG -201



//
// GSM response messages

#define GSMDEVICE_OK_MSG            "OK"
#define GSMDEVICE_E_CME_MSG         "+CME ERROR:"
#define GSMDEVICE_E_CMS_MSG         "+CMS ERROR:"


//
// GSM response scan format 

#define GSMDEVICE_E_CME_SCAN_STR    "+CME ERROR:%d"
#define GSMDEVICE_E_CMS_SCAN_STR    "+CMS ERROR:%d"


//
// GSM AT commands

#define SMS_MSG_FORMAT_CMD_TEST     "AT+CMGF=?"
#define SMS_MSG_FORMAT_CMD_GET      "AT+CMGF?"
#define SMS_MSG_FORMAT_CMD_SET      "AT+CMGF="

#define RESULT_CODE_FORMAT_CMD_GET  "ATV"
#define RESULT_CODE_FORMAT_CMD_SET  "ATV"


//
// misc. definitons

#define EMPTY_STRING          ""
#define CR_STRING             "\r"
#define LF_STRING             "\n"
#define CRLF_STRING           "\r\n"

// 
// ------------------------------- TYPES --------------------------------
//


typedef bool                  BOOL;
typedef uint8_t               BYTE;
typedef int8_t                INT8;
typedef uint8_t               UNINT8;
typedef int16_t               INT16;
typedef uint16_t              UINT16;
typedef int32_t               INT32;
typedef uint32_t              UNINT32;
typedef int64_t               INT64;
typedef uint64_t              UINT64;
typedef String                STRING;
#ifndef linux
typedef Stream                STREAM;
typedef SoftwareSerial        SW_SERIAL;
typedef HardwareSerial        HW_SERIAL;
#else // linux platform
typedef STRING                DEVICENAME;
#endif // linux

enum portType 
{
    swSerial,
    hwSerial,
    streamType,
    linuxDevice,
    nodev
};


union _port
{
#ifndef linux
    STREAM    *_str;
    SW_SERIAL *_sw;
    HW_SERIAL *_hw;
#else // linux platform
    INT16     _dev;
#endif // linux
};

enum devStatus
{
    created,
    initialized,
    ready,
    online,
    offline,
    not_responding
};

enum gsmDevType 
{
    AI_A6,
    AI_A7,
    UNKNOWN_GSM_DEVICE
};

enum gsmCommandMode
{
    cmd_test,
    cmd_get,
    cmd_set

};

enum smsMessageFormat
{
     smsPDUMode = 0,
     smsTXTMode = 1
};


enum cmdResultCodeFormat
{
     cmdResultNumeric = 0,
     cmdResultText    = 1
};

struct _gsm_errcode2msg {
    INT16 errcode;
    const char *pMessage;
};


#ifdef __cplusplus
}
#endif

// 
// ------------------------------- OBJECTS ------------------------------
//

class gsmDevice {

public:

    BYTE majorRelease;
    BYTE minorRelease;

private:

#ifdef linux
    INT16 _lastErrno;
#endif // linux

    gsmDevType  _gsmDeviceType;

    union _port _cmdPort;
    portType    _cmdPortType;
    INT32       _cmdPortSpeed;
    INT32       _cmdPortTimeout;

    union _port _dataPort;
    portType    _dataPortType;
    INT32       _dataPortSpeed;
    INT32       _dataPortTimeout;

    devStatus _devStatus;

    BOOL   _ownStream;
    INT16  _lastError;
    INT16  _cmeLastError;
    INT16  _cmsLastError;

public:

    gsmDevice();
    ~gsmDevice();

    STRING getError();
    INT16 begin(gsmDevType devType);

#ifndef linux
    INT16 init(INT16 rxPin = NULL_PIN, INT16 txPin = NULL_PIN, 
             INT32 speed = NO_SPEED, INT32 timeout = NO_TIMEOUT);
    INT16 init(INT16 serialNo = NO_SERIAL, INT32 speed = NO_SPEED, 
             INT32 timeout = NO_TIMEOUT);
    INT16 init(STREAM *output = NULL_STREAM, INT32 timeout = NO_TIMEOUT);
    INT16 init(SW_SERIAL *output = NULL_STREAM, INT32 timeout = NO_TIMEOUT);
    INT16 init(HW_SERIAL *output = NULL_STREAM, INT32 timeout = NO_TIMEOUT);
#else // linux platform
    INT16 init(DEVICENAME deviceName, INT32 speed = NO_SPEED, INT32 timeout = NO_TIMEOUT);
    INT16 uartReadResponse( int fd, char *pResponse, int maxLen, long timeout );
#endif // linux

    INT16 flush();

    //
    //  gsm commands supported
    //
    INT16 smsMsgFormat( gsmCommandMode cmdMode, smsMessageFormat *pFmt, 
                        STRING &result );
    INT16 resultCodeFormat( gsmCommandMode cmdMode, cmdResultCodeFormat *pFmt, 
                        STRING &result );

 

private:

    INT16 synchronize( STRING cmd, STRING expect );
    INT16 syncWithResponse( STRING cmd,  STRING expect);
    INT16 readResponse( STRING &response, BOOL flushAfter );
    INT16 sendCommand( STRING cmd, BOOL flushBefore );
    INT16 checkResponse( STRING result, STRING &dummy );
    INT16 cmsErrorMsg( STRING &errmsg );
    INT16 cmeErrorMsg( STRING &errmsg );
    INT16 parseResponse( STRING response, STRING expect, STRING &result );
    INT16 scanCMSErrNum( STRING response, INT16 &errNo );
    INT16 scanCMEErrNum( STRING response, INT16 &errNo );

#ifdef linux
    void sigHandler (int status);
    int setupSerial(int fd, int speed);
    void setSerialMin(int fd, int mcount);
#endif // linux
 

};


#endif // _GSMDEVICE_H_

// 
// -------------------NOTHING IMPORTAN BEHIND THIS LINE -----------------

