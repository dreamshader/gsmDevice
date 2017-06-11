//
// ************************************************************************
//
// part of gsmDevice (c) 2017 Dirk Schanz aka dreamshader
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

#define GSMDEVICE_SYNC_MAX_CMD         10
#define GSMDEVICE_SYNC_MAX_TOTAL      200
#define GSMDEVICE_SYNC_DELAY           20
#define GSMDEVICE_SYNC_CMD            "AT\r\n"
#define GSMDEVICE_SYNC_RSP            "OK"
#define GSMDEVICE_READ_TIMEOUT      30000   // up to 30 sec. response time
#define GSMDEVICE_READ_DELAY          100

//
// parameter values

#define GSMDEVICE_NULL_PIN              0
#define GSMDEVICE_NULL_STREAM        NULL
#define GSMDEVICE_NO_SPEED              0
#define GSMDEVICE_NO_TIMEOUT            0
#define GSMDEVICE_NO_SERIAL             0

#define GSMDEVICE_DEF_CMD_SPEED      9600
#define GSMDEVICE_SWSERIAL_MAX_BAUD 19200

//
// unified error codes

#define GSMDEVICE_SUCCESS               0   // no error
#define GSMDEVICE_ERROR                -1   // general error
#define GSMDEVICE_E_UNSPECIFIC         -2   // unspecified/unknown error

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

#define GSMDEVICE_E_PATTERN           -40
#define GSMDEVICE_E_RESULT            -41
#define GSMDEVICE_E_INVAL             -42
#define GSMDEVICE_E_TOO_SHORT         -43
#define GSMDEVICE_E_FMT               -44

#define GSMDEVICE_LIST_E_TYPE         -50
#define GSMDEVICE_LIST_E_ACTION       -51
#define GSMDEVICE_LIST_E_PATTERN      -52
#define GSMDEVICE_LIST_E_NULL         -53
#define GSMDEVICE_LIST_E_EMPTY        -54
#define GSMDEVICE_LIST_E_FOUND        -55
#define GSMDEVICE_LIST_E_NO_MORE      -56
#define GSMDEVICE_LIST_E_NO_PREV      -57
#define GSMDEVICE_LIST_E_SUPPORTED    -58

#define GSMDEVICE_E_CME              -100
#define GSMDEVICE_E_CME_UNKNOWN      -101
#define GSMDEVICE_E_CMS              -200
#define GSMDEVICE_E_CMS_UNKNOWN      -201
 
 

//
// misc. definitons

#define GSMDEVICE_EMPTY_STRING        ""
#define GSMDEVICE_CR_STRING           "\r"
#define GSMDEVICE_LF_STRING           "\n"
#define GSMDEVICE_CRLF_STRING         "\r\n"

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
// --------------------------- LIST MANAGEMENT --------------------------
//

#define GSMDEVICE_LIST_BUFFER_LEN     130

enum _listType 
{
    listTypeCPOL,
    listTypeOTHER
};

enum _listAction
{
    firstItem,
    nextItem,
    prevItem,
    lastItem,
    currItem,
    findItem,
    findPrevItem,
    findNextItem,
    findLastItem
};


struct _listInString {
    _listType _type;
    int       _current;
    int       _iCurrent;
    char      _tmpBuffer[GSMDEVICE_LIST_BUFFER_LEN+1];
    char     *_list;
};


// 
// ------------------------------ GSM STUFF -----------------------------
//

//
// GSM response messages

#define GSMDEVICE_OK_MSG              "OK"
#define GSMDEVICE_ATTENTION           "AT"
#define GSMDEVICE_E_CME_MSG           "+CME ERROR:"
#define GSMDEVICE_E_CMS_MSG           "+CMS ERROR:"

#define GSMDEVICE_RESP_PATTERN_LEN   10
//
// GSM response scan format 

#define GSMDEVICE_E_CME_FMT_STR       "+CME ERROR:%d"
#define GSMDEVICE_E_CMS_FMT_STR       "+CMS ERROR:%d"


//
// GSM AT commands

enum gsmCommandMode
{
    cmd_test,
    cmd_read,
    cmd_set,
    cmd_execute

};

// -------------

#define SMS_MSG_FORMAT_CMD_TEST       "+CMGF=?"
#define SMS_MSG_FORMAT_CMD_READ       "+CMGF?"
#define SMS_MSG_FORMAT_CMD_SET        "+CMGF="

enum smsMessageFormat
{
     smsPDUMode = 0,
     smsTXTMode = 1
};

#define readResponseFmtCMGF           "%d \r\n"
#define testResponseFmtCMGF           "(%d,%d)\r\n"

struct _dataCMGF {
    smsMessageFormat current;
    int from;
    int to;
};

// -------------

#define RESULT_CODE_FORMAT_CMD_EXEC   "V"

enum cmdResultCodeFormat
{
     cmdResultNumeric = 0,
     cmdResultText    = 1
};

// -------------

#define OPERATOR_SELECT_CMD_TEST      "+COPS=?"
#define OPERATOR_SELECT_CMD_READ      "+COPS?"
#define OPERATOR_SELECT_CMD_SET       "+COPS="

enum opSelectMode
{
    opSelectAuto       = 0,
    opSelectManual     = 1,
    opSelectDeregister = 2,
    opSelectFormatOnly = 3,
    opSelectManualAuto = 4
};







// -------------

#define ECHO_COMMAND_CMD_EXEC         "E"

enum cmdEcho
{
    cmdEchoOff = 0,
    cmdEchoOn  = 1
};

// -------------

#define NETWORK_REGISTRATION_CMD_TEST "+CREG=?"
#define NETWORK_REGISTRATION_CMD_READ "+CREG?"
#define NETWORK_REGISTRATION_CMD_SET  "+CREG="
enum networkRegistrationMode
{
    disableNetwRegUnsol   = 0,
    enableNetwRegUnsol    = 1,
    enableNetwRegUnsolLoc = 2
};

#define GSM_BASESTATION_LAC_LENGTH     4
#define GSM_BASESTATION_CI_LENGTH      4

#define testResponseFmtCREG            "%d, %d\r\n"
#define readResponseFmtCREG            "%d, %d, \"%4s\", \"%4s\" \r\n"

struct _dataCREG {
    networkRegistrationMode mode;
    int fromMode;
    int toMode;
    int stat;
    char lac[GSM_BASESTATION_LAC_LENGTH+1];
    char ci[GSM_BASESTATION_CI_LENGTH+1];
};


// -------------

#define SIGNAL_QUALITY_CMD_TEST       "+CSQ=?"
#define SIGNAL_QUALITY_CMD_EXEC       "+CSQ"

#define executeResponseFmtCSQ         "%d, %d \r\n"
#define testResponseFmtCSQ            "(%d-%d,%d),(%d-%d,%d)\r\n"

struct _dataCSQ {
    int rssi;
    int ber;

    int rssiFrom;
    int rssiTo;
    int rssiUnknown;

    int berFrom;
    int berTo;
    int berUnknown;
};

// -------------

#define PREF_OPERATOR_LIST_CMD_TEST   "+CPOL=?"
#define PREF_OPERATOR_LIST_CMD_READ   "+CPOL?"
#define PREF_OPERATOR_LIST_CMD_SET    "+CPOL="

enum prefOperList
{
    prefOperLongAlphaMode  = 0,
    prefOperShortAlphaMode = 1,
    prefOperNumericMode    = 2,
    prefOperIgnore         = 3
};

#define testResponseFmtCPOL           "(%d-%d),(%d,%d)\r\n"
#define readResponseFmtCPOL           "+CPOL: %d,%d,%s\r\n"
#define GSMDEVICE_CMD_CPOL_NULL_INDEX -1

struct _dataCPOL {
    // cmd_test param
    int fromIndex;
    int toIndex;
    prefOperList fromFormat;
    prefOperList toFormat;
    // cmd_set param
    int index;
    prefOperList format;
    STRING oper;
    // cmd_read param
    STRING list;
};
















// -------------


#define REQUEST_IMSI_CMD_TEST         "+CIMI=?"
#define REQUEST_IMSI_CMD_SET          "+CIMI"

// no additional data

#define GSM_MAX_IMSI_LENGTH           15

struct _dataIMSI {
    INT16 _length;
    char _raw[GSM_MAX_IMSI_LENGTH+1];
};

#define setResponseFmtIMSI            "%15s\r\n"


// -------------

#define READ_WRITE_IMEI_CMD_TEST      "+EGMR=?"
#define READ_WRITE_IMEI_CMD_SET       "+EGMR="

enum rwIMEIMode
{
    imeiWriteMode = 1,
    imeiReadMode = 2
};

#define GSM_MAX_IMEI_LENGTH           15

struct _dataEGMR {
    rwIMEIMode mode;
    INT16 format;
    STRING IMEI;
    char _raw[GSM_MAX_IMEI_LENGTH+1];
    int _length;
};

#define setResponseFmtEGMR            "%15s\r\n"

// -------------

#define REQUEST_REV_ID_CMD_TEST       "+CGMR=?"
#define REQUEST_REV_ID_CMD_SET        "+CGMR"

// no additional data


#define REQUEST_MANSPEC_INFO_CMD_SET  "I"

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

    INT16 getError( STRING &errMsg );
    INT16 begin(gsmDevType devType);

#ifndef linux
    INT16 init(INT16 rxPin = GSMDEVICE_NULL_PIN, INT16 txPin = GSMDEVICE_NULL_PIN, 
             INT32 speed = GSMDEVICE_NO_SPEED, INT32 timeout = GSMDEVICE_NO_TIMEOUT);
    INT16 init(INT16 serialNo = GSMDEVICE_NO_SERIAL, INT32 speed = GSMDEVICE_NO_SPEED, 
             INT32 timeout = GSMDEVICE_NO_TIMEOUT);
    INT16 init(STREAM *output = GSMDEVICE_NULL_STREAM, INT32 timeout = GSMDEVICE_NO_TIMEOUT);
    INT16 init(SW_SERIAL *output = GSMDEVICE_NULL_STREAM, INT32 timeout = GSMDEVICE_NO_TIMEOUT);
    INT16 init(HW_SERIAL *output = GSMDEVICE_NULL_STREAM, INT32 timeout = GSMDEVICE_NO_TIMEOUT);
#else // linux platform
    INT16 init(DEVICENAME deviceName, INT32 speed = GSMDEVICE_NO_SPEED, INT32 timeout = GSMDEVICE_NO_TIMEOUT);
    INT16 uartReadResponse( int fd, char *pResponse, int maxLen, long timeout );
    INT16 uartReadString( int fd, char *pResponse, int maxLen, int *pRead, long timeout );
    INT16 removeNullChars( char *pResponse, int maxLen );
#endif // linux

    INT16 inputFlush();

    //
    //  gsm commands supported
    //
    INT16 smsMsgFormat( gsmCommandMode cmdMode, _dataCMGF *pData, 
                        STRING &result, void *pParam = NULL );
    INT16 resultCodeFormat( gsmCommandMode cmdMode, cmdResultCodeFormat *pFmt, 
                        STRING &result, void *pParam = NULL );
    INT16 operatorSelects( gsmCommandMode cmdMode, opSelectMode *pFmt, 
                                  STRING &result, void *pParam = NULL );
    INT16 commandEcho( gsmCommandMode cmdMode, cmdEcho *pFmt, 
                                  STRING &result, void *pParam = NULL );
    INT16 networkRegistration( gsmCommandMode cmdMode, _dataCREG *pData, 
                                  STRING &result, void *pParam = NULL );
    INT16 signalQuality( gsmCommandMode cmdMode, struct _dataCSQ *pData,
                                  STRING &result, void *pParam = NULL );
    INT16 preferredOperatorList( gsmCommandMode cmdMode, _dataCPOL *pData, 
                                  STRING &result, void *pParam = NULL );
    INT16 requestIMSI( gsmCommandMode cmdMode, struct _dataIMSI *pData, 
                                  STRING &result, void *pParam = NULL );
    INT16 readWriteIMEI( gsmCommandMode cmdMode, struct _dataEGMR *pData, 
                                  STRING &result, void *pParam = NULL );
    INT16 requestRevisionId( gsmCommandMode cmdMode, void *pIgnored, 
                                  STRING &result, void *pParam = NULL );
    INT16 requestManufacturerData( gsmCommandMode cmdMode, INT16 infoValue, 
                                  STRING &result, void *pParam = NULL );


    INT16 listMan( struct _listInString *list, _listAction action, char *_pattern, int _patLen, STRING &result, void *pData );
    INT16 listManInitList( _listType type, struct _listInString *list, STRING srcList );


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
    INT16 getDataIndex( STRING response, char _pattern[], INT16 patternLength, INT16 *pIndex );
    INT16 parseCMGF( gsmCommandMode cmdMode, STRING response, char _pattern[], INT16 dataIndex, struct _dataCMGF *pData );
    INT16 parseCSQ( gsmCommandMode cmdMode, STRING response, char _pattern[], INT16 dataIndex, struct _dataCSQ *pData );
    INT16 parseIMSI( gsmCommandMode cmdMode, STRING response,  struct _dataIMSI *pData );
    INT16 parseEGMR( gsmCommandMode cmdMode, STRING response, char _pattern[], INT16 dataIndex, struct _dataEGMR *pData );
    INT16 removeEcho( STRING &result, STRING &dummy );
    INT16 parseCREG( gsmCommandMode cmdMode, STRING response, char _pattern[], INT16 dataIndex, struct _dataCREG *pData );
    INT16 parseCPOL( gsmCommandMode cmdMode, STRING response, char _pattern[], INT16 dataIndex, struct _dataCPOL *pData );


//    INT16 listMan( struct _listInString *list, _listAction action, char *_pattern, int _patLen, STRING &result, void *pData );
//    INT16 listManInitList( _listType type, struct _listInString *list, STRING srcList );

    INT16 listManFirst( struct _listInString *list, STRING &result, void *pData  = NULL );
    INT16 listManNext( struct _listInString *list, STRING &result, void *pData = NULL  );
    INT16 listManPrev( struct _listInString *list, STRING &result, void *pData = NULL  );
    INT16 listManLast( struct _listInString *list, STRING &result, void *pData = NULL  );
    INT16 listManCPOLFirst( struct _listInString *list, STRING &result, void *pData = NULL  );
    INT16 listManCPOLNext( struct _listInString *list, STRING &result, void *pData = NULL  );
    INT16 listManCPOLPrev( struct _listInString *list, STRING &result, void *pData = NULL  );
    INT16 listManCPOLLast( struct _listInString *list, STRING &result, void *pData = NULL  );
    INT16 listManGetLine( struct _listInString *list );




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

