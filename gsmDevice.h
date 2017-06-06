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

#define GSMDEV_SYNC_MAX_CMD            10
#define GSMDEV_SYNC_MAX_TOTAL         200
#define GSMDEV_SYNC_DELAY              20
#define GSMDEV_SYNC_CMD               "AT\r\n"
#define GSMDEV_SYNC_RSP               "OK"
#define GSMDEV_READ_TIMEOUT         30000   // up to 30 sec. response time
#define GSMDEV_READ_DELAY             100

//
// parameter values

#define NULL_PIN                        0
#define NULL_STREAM                  NULL
#define NO_SPEED                        0
#define NO_TIMEOUT                      0
#define NO_SERIAL                       0

#define GSMDEVICE_DEF_CMD_SPEED      9600
#define GSMDEVICE_SWSERIAL_MAX_BAUD 19200

//
// unified error codes

#define GSMDEVICE_SUCCESS               0   // no error
#define GSMDEVICE_ERROR                -1   // unspecified/unknown error

#define GSMDEVICE_E_P_RXPIN           -10
#define GSMDEVICE_E_P_TXPIN           -11
#define GSMDEVICE_E_P_SPEED           -12
#define GSMDEVICE_E_P_TMOUT           -13
#define GSMDEVICE_E_P_SERIAL          -14
#define GSMDEVICE_E_P_STREAM          -15
#define GSMDEVICE_E_P_NULL            -16
#define GSMDEVICE_E_P_PARAM           -17

#define GSMDEVICE_E_INIT              -30
#define GSMDEVICE_E_RESPONSE          -31
#define GSMDEVICE_E_SUPPORTED         -32
#define GSMDEVICE_E_DEV_TYPE          -33
#define GSMDEVICE_E_SEND              -34
#define GSMDEVICE_E_MATCH             -35
#define GSMDEVICE_E_CMD_MODE          -36
#define GSMDEVICE_E_OPEN              -37
#define GSMDEVICE_E_SETUP             -38


#define GSMDEVICE_E_CME              -100
#define GSMDEVICE_E_CME_NOMSG        -101
#define GSMDEVICE_E_CMS              -200
#define GSMDEVICE_E_CMS_NOMSG        -201


//
// misc. definitons

#define EMPTY_STRING          ""
#define CR_STRING             "\r"
#define LF_STRING             "\n"
#define CRLF_STRING           "\r\n"

// 
// ---------------------- TYPES - NOT GSM RELATED -----------------------
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

struct _gsm_errcode2msg {
    INT16 errcode;
    const char *pMessage;
};


// 
// ------------------------------ GSM STUFF -----------------------------
//

//
// GSM response messages

#define GSMDEVICE_OK_MSG              "OK"
#define GSMDEVICE_E_CME_MSG           "+CME ERROR:"
#define GSMDEVICE_E_CMS_MSG           "+CMS ERROR:"


//
// GSM response scan format 

#define GSMDEVICE_E_CME_SCAN_STR      "+CME ERROR:%d"
#define GSMDEVICE_E_CMS_SCAN_STR      "+CMS ERROR:%d"


//
// GSM AT commands

enum gsmCommandMode
{
    cmd_test,
    cmd_get,
    cmd_set

};

// -------------

#define SMS_MSG_FORMAT_CMD_TEST       "AT+CMGF=?"
#define SMS_MSG_FORMAT_CMD_GET        "AT+CMGF?"
#define SMS_MSG_FORMAT_CMD_SET        "AT+CMGF="

enum smsMessageFormat
{
     smsPDUMode = 0,
     smsTXTMode = 1
};

// -------------

#define RESULT_CODE_FORMAT_CMD_GET    "ATV"
#define RESULT_CODE_FORMAT_CMD_SET    "ATV"

enum cmdResultCodeFormat
{
     cmdResultNumeric = 0,
     cmdResultText    = 1
};

// -------------

#define OPERATOR_SELECT_CMD_TEST      "AT+COPS=?"
#define OPERATOR_SELECT_CMD_GET       "AT+COPS?"
#define OPERATOR_SELECT_CMD_SET       "AT+COPS="

enum opSelectMode
{
    opSelectAuto       = 0,
    opSelectManual     = 1,
    opSelectDeregister = 2,
    opSelectFormatOnly = 3,
    opSelectManualAuto = 4
};

// -------------

#define ECHO_COMMAND_CMD_SET          "ATE"

enum cmdEcho
{
    cmdEchoOff = 0,
    cmdEchoOn  = 1
};

// -------------

#define NETWORK_REGISTRATION_CMD_TEST "AT+CREG=?"
#define NETWORK_REGISTRATION_CMD_GET  "AT+CREG?"
#define NETWORK_REGISTRATION_CMD_SET  "AT+CREG="
enum networkRegistrationMode
{
    disableNetwRegUnsol   = 0,
    enableNetwRegUnsol    = 1,
    enableNetwRegUnsolLoc = 2
};

// -------------

#define SIGNAL_QUALITY_CMD_TEST       "AT+CSQ=?"
#define SIGNAL_QUALITY_CMD_GET        "AT+CSQ"

struct signalQuality {
    INT16 rssi;
    INT16 ber;
};

// -------------

#define PREF_OPERATOR_LIST_CMD_TEST   "AT+CPOL=?"
#define PREF_OPERATOR_LIST_CMD_GET    "AT+CPOL?"
#define PREF_OPERATOR_LIST_CMD_SET    "AT+CPOL="

enum prefOperList
{
    preOperLongAlphaMode  = 0,
    preOperShortAlphaMode = 1,
    preOperNumericMode    = 2
};

// -------------


#define REQUEST_IMSI_CMD_TEST         "AT+CIMI=?"
#define REQUEST_IMSI_CMD_SET          "AT+CIMI"

// no additional data

// -------------

#define READ_WRITE_IMEI_CMD_TEST      "AT+EGMR=?"
#define READ_WRITE_IMEI_CMD_SET       "AT+EGMR="

enum rwIMEIMode
{
    imeiWriteMode = 1,
    imeiReadMode = 2
};

struct rwIMEIData {
    rwIMEIMode mode;
    INT16 format;
    STRING IMEI;
};

// -------------

#define REQUEST_REV_ID_CMD_TEST       "AT+CMGR=?"
#define REQUEST_REV_ID_CMD_SET        "AT+CMGR"

// no additional data


#define REQUEST_MANSPEC_INFO_CMD_SET  "AT+CMGR"

// no additional data



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
    time_t _startTime;
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
    INT16 uartReadString( int fd, char *pResponse, int maxLen, int *pRead, long timeout );
#endif // linux

    INT16 inputFlush();

    //
    //  gsm commands supported
    //
    INT16 smsMsgFormat( gsmCommandMode cmdMode, smsMessageFormat *pFmt, 
                        STRING &result, void *pParam = NULL );
    INT16 resultCodeFormat( gsmCommandMode cmdMode, cmdResultCodeFormat *pFmt, 
                        STRING &result, void *pParam = NULL );
    INT16 operatorSelects( gsmCommandMode cmdMode, opSelectMode *pFmt, 
                                  STRING &result, void *pParam = NULL );
    INT16 commandEcho( gsmCommandMode cmdMode, cmdEcho *pFmt, 
                                  STRING &result, void *pParam = NULL );
    INT16 networkRegistration( gsmCommandMode cmdMode, networkRegistrationMode *pFmt, 
                                  STRING &result, void *pParam = NULL );
    INT16 signalQuality( gsmCommandMode cmdMode, struct signalQuality *pData,
                                  STRING &result, void *pParam = NULL );
    INT16 preferredOperatorList( gsmCommandMode cmdMode, prefOperList *pMode, 
                                  STRING &result, void *pParam = NULL );
    INT16 requestIMSI( gsmCommandMode cmdMode, void *pIgnored, 
                                  STRING &result, void *pParam = NULL );
    INT16 readWriteIMEI( gsmCommandMode cmdMode, struct rwIMEIData *pData, 
                                  STRING &result, void *pParam = NULL );
    INT16 requestRevisionId( gsmCommandMode cmdMode, void *pIgnored, 
                                  STRING &result, void *pParam = NULL );
    INT16 requestManufacturerData( gsmCommandMode cmdMode, INT16 infoValue, 
                                  STRING &result, void *pParam = NULL );



private:

    INT16 synchronize( STRING cmd, STRING expect );
    INT16 syncWithResponse( STRING cmd,  STRING expect);
    INT16 readResponse( STRING &response, BOOL flushAfter, INT32 timeout );
    INT16 sendCommand( STRING cmd, BOOL flushBefore, INT32 timeout );
    INT16 checkResponse( STRING result, STRING &dummy );
    INT16 cmsErrorMsg( STRING &errmsg );
    INT16 cmeErrorMsg( STRING &errmsg );
    INT16 parseResponse( STRING response, STRING expect, STRING &result );
    INT16 scanCMSErrNum( STRING response, INT16 &errNo );
    INT16 scanCMEErrNum( STRING response, INT16 &errNo );

#ifdef linux
    long millis();
    void delay(long msec);
    int setupSerial(int fd, int speed);
    void setSerialMin(int fd, int mcount);
#endif // linux
 

};


#endif // _GSMDEVICE_H_

// 
// -------------------NOTHING IMPORTAN BEHIND THIS LINE -----------------

