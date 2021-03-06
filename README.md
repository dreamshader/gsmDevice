# gsmDevice

### A class for Arduino and Linux to handle a gsm module like a A6 gsm breakout board.

### Warning! This is an absolutely work in progress project. For most stable version use pre0.2 branch/pre-release


----------


### **class reference:**

***gsmDevice()***

Constructor. Creates a gsmDevice object. There are no parameters.


----------


***~gsmDevice()***

Destructor. Deletes an instance of class gsmdevice. 

**Note:**Do not call in your program!*


----------


***STRING getError()***

Returns a string containing a description of the last error occured.

**Note:**Is not supported, yet.*


----------


***INT16 begin(gsmDevType devType)***

Begin communication with and initialize the attached GSM module.
devType may be one of AI_A6 or AI_A7 as defined in gsmDevice.h.

**Note:**Only AI_A6 is supported, yet.*


----------


***INT16 inputFlush()***

This method reads off all chracters in the input queue of the serial device, the gsm module is attached to.
There is no parameter.
inputFlush() returns GSMDEVICE_SUCCESS. If something went wrong, one of the following values is returned:

 - GSMDEVICE_E_INIT (gsm device instance is not initialized)
 - GSMDEVICE_E_SUPPORTED (not supported, e.g. stream device)

**Note:**On a Linux platform at this time inputFlush() always will return GSMDEVICE_E_SUPPORTED.*

----------

#### **Init methods:**

**The following init-methods are availabe in the Arduino-IDE only:**

The init method tries to communicate with the attached gsm device and initializese it for further operations.
In case of success each init returns GSMDEVICE_SUCCESS. 
Depending of an error, that occurred, resultcode may differ. 


----------


***INT16 init(INT16 rxPin = NULL_PIN, INT16 txPin = NULL_PIN, INT32 speed = NO_SPEED, INT32 timeout = NO_TIMEOUT)***

Expects two valid pins. These pins will be used as Rx and Tx for a Software-Serial connection to talk to the attached module.
Except the pins the other parameter are optional. Default speed is 9600 baud, default timeout is around 200 ms.

**Note:**Software Serial is not very reliable above 19200 baud. You will get an error if you specify more than GSMDEVICE_SWSERIAL_MAX_BAUD.*

If something went wrong, the returned value is one of

 - GSMDEVICE_E_P_RXPIN  (invalid value for rxPin specified).
 - GSMDEVICE_E_P_TXPIN  (invalid value for txPin specified).
 - GSMDEVICE_E_P_SPEED  (invalid baudrate specified).
 - GSMDEVICE_E_P_TMOUT  (invalid timeout value specified).


----------


***INT16 init(INT16 serialNo = NO_SERIAL, INT32 speed = NO_SPEED, INT32 timeout = NO_TIMEOUT)***

Expects the number of UART to use. If serialNo is NO_SERIAL, the default, the first UART is used. On a MEGA2560 up to 4 UARTS (0 to 3) are available. 
The parameters speed and timeout are optional, too. Default speed is 9600 baud, default timeout is around 200 ms.

**Note:**This method is not well tested an may contain faulty code. In addition, any other value than the default as serialNo will cause the error GSMDEVICE_E_SUPPORTED.*

If something went wrong, the returned value is one of

 - GSMDEVICE_E_P_SERIAL (invalid  number for hw serial)
 - GSMDEVICE_E_P_SPEED  (invalid baudrate specified).
 - GSMDEVICE_E_P_TMOUT  (invalid timeout value specified).
 - GSMDEVICE_E_SUPPORTED (not yet supported)


----------


The following three methods use a preconfigured stream resp. serial device. The settings of this device are not altered in any way, so that correct settings are your responsibility.

***INT16 init(STREAM \*output = NULL_STREAM, INT32 timeout = NO_TIMEOUT)***

Expects a pointer to an open stream to communicate with the attached gsm device. Timeout is optional and as a default roughly 200 ms.

If something went wrong, the returned value is one of

 - GSMDEVICE_E_P_STREAM (invalid stream supplied)
 - GSMDEVICE_E_P_TMOUT  (invalid value for timeout)

***INT16 init(SW_SERIAL \*output = NULL_STREAM, INT32 timeout = NO_TIMEOUT)***

Expects a pointer to an open software serial connection to communicate with the attached gsm device. Timeout is optional and as a default roughly 200 ms.

If something went wrong, the returned value is one of

 - GSMDEVICE_E_P_STREAM (invalid software serial supplied)
 - GSMDEVICE_E_P_TMOUT  (invalid value for timeout)

***INT16 init(HW_SERIAL \*output = NULL_STREAM, INT32 timeout = NO_TIMEOUT)***

Expects a pointer to an open serial connection to communicate with the attached gsm device. Timeout is optional and as a default roughly 200 ms.

If something went wrong, the returned value is one of

 - GSMDEVICE_E_P_STREAM (invalid serial supplied)
 - GSMDEVICE_E_P_TMOUT  (invalid value for timeout)

**Note:**None of these methods are tested, yet.*

**Note:**Init with an open stream is not supported at this time.*


----------


**On a Linux platform, there is an own init method:**

----------


***INT16 init(DEVICENAME deviceName, INT32 speed = NO_SPEED, INT32 timeout = NO_TIMEOUT)***

Expects valid path to an UART, e.g. PL3203 USB device or /dev/ttyS0 as a serial communication port.
The speed and timeout parameters are optional. Their defaults are the same as in the init-methods for Arduino, 9600 baud resp. around 200 ms as the timeout value.

If something went wrong, the returned value is one of

 - GSMDEVICE_E_P_SERIAL (invalid path to serial device)
 - GSMDEVICE_E_P_SPEED  (invalid baudrate)
 - GSMDEVICE_E_P_TMOUT  (invalid timeout value)

----------


#### **Supported gsm commands:**

**Quite all gsm commands expect the following parameters:**

*gsmCommandMode cmdMode* - command mode. Must be one of *cmd_test*, *cmd_read*,  *cmd_execute* or *cmd_set* as defined in gsmDevice.h.
*STRING &result* - a reference to a string object to hold the response of the attached gsm device.
void *pParam is a pointer to additional parameters needed. This is not supported and may be ommited at this time. 
Only the second parameter differs in type and meaning. See specific command for more detailed description.

**Quite all gsm commands return GSMDEVICE_SUCCESS the on successful operation.**

Exception: if an error occurred, a specific error code is returned. See each command for more detailed description.

**Note:**Not all gsm commands support all four command modes cmd_test, cmd_read, cmd_execute and cmd_set. You will get the error GSMDEVICE_E_CMD_MODE if supplied command mode is not supported.*

----------

**Select SMS message format (AT+CMGF)**
***INT16 smsMsgFormat( gsmCommandMode cmdMode, struct _dataCMGF \*pData, STRING &result , void \*pParam = NULL)***

Set format of SMS messages to PDU or text format.

**Note:**PDU format is not really readable by humans.*
Supports: gsmCommandMode *cmd_test*, *cmd_set* and *cmd_read*.
Expects: a pointer to a vaiable of type struct _dataCMGF. In case of *cmd_set* this variable must contain either *smsPDUMode* or *smsTXTMode*. All other values will cause an error. After a call with gsmCommandMode *cmd_get* the variable will contain the current SMS message format, either *smsPDUMode* or *smsTXTMode*.
On error it returns:

 - GSMDEVICE_E_INIT (gsmDevice instance is not initialized)
 - GSMDEVICE_E_SUPPORTED (not yet supported e.g. on a stream device)
 - GSMDEVICE_E_CMD_MODE  (invalid command mode)

----------

**Set result code format mode(ATV)**
***INT16 resultCodeFormat( gsmCommandMode cmdMode, cmdResultCodeFormat \*pFmt, STRING &result, void \*pParam)***

Switch resultcode representation of gsm module to numerical only resp. long text.
Supports: gsmCommandMode *cmd_execute* only.
Expects: a pointer to a vaiable of type cmdResultCodeFormat. This variable must contain either *cmdResultNumeric* or *cmdResultText*. All other values will cause an error.
On error it returns:

 - GSMDEVICE_E_INIT (gsmDevice instance is not initialized)
 - GSMDEVICE_E_SUPPORTED (not yet supported e.g. on a stream device)
 - GSMDEVICE_E_CMD_MODE  (invalid command mode)

----------

**Operator selects (AT+COPS):**
***INT16 operatorSelects( gsmCommandMode cmdMode, struct _dataCOPS \*pFmt, STRING &result, void \*pParam = NULL)***

Selects mode to connect to available operators.
Supports: gsmCommandMode *cmd_test*, *cmd_set* and *cmd_read*.
Expects: a pointer to a vaiable of type struct _dataCOPS. In case of *cmd_set* this variable must contain either *opSelectAuto*, *opSelectManual*, *opSelectDeregister*, *opSelectFormatOnly* or *opSelectManualAuto*. All other values will cause an error. After a call with gsmCommandMode *cmd_get* the variable will contain the current operator mode, either *opSelectAuto*, *opSelectManual*, *opSelectDeregister*, *opSelectFormatOnly* or *opSelectManualAuto*.
On error it returns:

 - GSMDEVICE_E_INIT (gsmDevice instance is not initialized)
 - GSMDEVICE_E_SUPPORTED (not yet supported e.g. on a stream device)
 - GSMDEVICE_E_CMD_MODE  (invalid command mode)

**Note:**The command mode cmd_set is not implemented, yet.*

**Warning! In test mode (cmdMode = cmd_test) this command is very slow and may need up to one minute and more to respond. Be patient and use it with caution, because it will block all further communication with the attached gsm device.**


----------

**Enable command echo (ATE):**
***INT16 commandEcho( gsmCommandMode cmdMode, cmdEcho \*pFmt, STRING &result, void \*pParam = NULL)***

Enable/disable echo of commands sent.
Supports: gsmCommandMode *cmd_execute* only.
Expects: a pointer to a vaiable of type cmdEcho. This variable must contain either *cmdEchoOff* or *cmdEchoOn*. All other values will cause an error.
On error it returns:

 - GSMDEVICE_E_INIT (gsmDevice instance is not initialized)
 - GSMDEVICE_E_SUPPORTED (not yet supported e.g. on a stream device)
 - GSMDEVICE_E_CMD_MODE  (invalid command mode)

----------
**Network registration (AT+CREG)**
***INT16 networkRegistration( gsmCommandMode cmdMode, struct _dataCREG \*pData, STRING &result, void \*pParam = NULL)***

Query the current network register status.
Supports: gsmCommandMode *cmd_test*, *cmd_set* and *cmd_read*.
Expects: a pointer to a vaiable of type struct _dataCREG. In case of *cmd_set* this variable must contain either *disableNetwRegUnsol*, *enableNetwRegUnsol* or *enableNetwRegUnsolLoc*. All other values will cause an error. After a call with gsmCommandMode *cmd_read* the variable will contain the current registration mode, either *disableNetwRegUnsol*, *enableNetwRegUnsol* or *enableNetwRegUnsolLoc*.
On error it returns:

 - GSMDEVICE_E_INIT (gsmDevice instance is not initialized)
 - GSMDEVICE_E_SUPPORTED (not yet supported e.g. on a stream device)
 - GSMDEVICE_E_CMD_MODE  (invalid command mode)

----------

**Signal quality (AT+CSQ)**
***INT16 signalQuality( gsmCommandMode cmdMode, struct _dataCSQ \*pData, STRING &result, void \*pParam = NULL)***

Query the quality of the signal.
Supports: gsmCommandMode *cmd_test* and *cmd_execute*.
Expects: a pointer to a vaiable of struct _dataCSQ. In case of *cmd_execute* this structure will hold the current values on successful execution.
In case of *cmd_test* this parameter is ommitted and may be NULL.
On error it returns:

 - GSMDEVICE_E_INIT (gsmDevice instance is not initialized)
 - GSMDEVICE_E_SUPPORTED (not yet supported e.g. on a stream device)
 - GSMDEVICE_E_CMD_MODE  (invalid command mode)

----------

**Preferred operator list (AT+CPOL)**
***INT16 preferredOperatorList( gsmCommandMode cmdMode, struct _dataCPOL \*pMode, STRING &result, void \*pParam = NULL)***

Edit and query the list with preferred operators.
Supports: gsmCommandMode *cmd_test*, *cmd_set* and *cmd_read*.
Expects: a pointer to a vaiable of type struct _dataCPOL. In case of *cmd_set* this variable must contain either *prefOperLongAlphaMode*, *prefOperShortAlphaMode* or *prefOperNumericMode*. All other values will cause an error. After a call with gsmCommandMode *cmd_read* the STRING result will contain the current list of preferred operators.
On error it returns:

 - GSMDEVICE_E_INIT (gsmDevice instance is not initialized)
 - GSMDEVICE_E_SUPPORTED (not yet supported e.g. on a stream device)
 - GSMDEVICE_E_CMD_MODE  (invalid command mode)

**Note:**cmd_read is currently not completely impemented.*

----------

**Request international mobile subscriber identity (AT+CIMI)**
***INT16 requestIMSI( gsmCommandMode cmdMode, struct _dataIMSI \*pData, STRING &result, void \*pParam = NULL)***

Enable/disable echo of commands sent.
Supports: gsmCommandMode *cmd_test* and *cmd_set*.
Expects: a pointer to struct _dataIMSI to hold results. After a call with gsmCommandMode *cmd_set* the STRING result will contain the current IMSI, too.
On error it returns:

 - GSMDEVICE_E_INIT (gsmDevice instance is not initialized)
 - GSMDEVICE_E_SUPPORTED (not yet supported e.g. on a stream device)
 - GSMDEVICE_E_CMD_MODE  (invalid command mode)

**Note**: *cmd_set is currently not fully supported.*

----------

**Read and write IMEI (AT+EGMR)**
***INT16 readWriteIMEI( gsmCommandMode cmdMode, struct _dataEGMR \*pData, STRING &result, void \*pParam = NULL)***

Edit and query the international mobile equipment identity.
Supports: gsmCommandMode *cmd_test* and *cmd_set*.
Expects: a pointer to a vaiable of type struct _dataEGMR. In case of *cmd_set* this variable must contain the following fields:
imeiData.mode: imeiReadMode or imeiWriteMode
imeiData.format: don't care, will be set automatically
imeiData.IMEI: EMPTY_STRING or IMEI if imeiData.mode is imeiWriteMode
Other values will cause an error. After a succesful return the current values are returned.
On error it returns:

 - GSMDEVICE_E_INIT (gsmDevice instance is not initialized)
 - GSMDEVICE_E_P_NULL (pData is a null pointer)
 - GSMDEVICE_E_SUPPORTED (not yet supported e.g. on a stream device)
 - GSMDEVICE_E_CMD_MODE  (invalid command mode)

**Note:**cmd_set is currently not completely impemented.*

----------

**Request revision identification (AT+ CGMR)**
***INT16 requestRevisionId( gsmCommandMode cmdMode, void \*pIgnored, STRING &result, void \*pParam = NULL)***

Request revision information from the TA.
Supports: gsmCommandMode *cmd_test* and *cmd_set*.
The void pointer pIgnored is omitted an may be NULL. 
After a call with gsmCommandMode *cmd_set* the STRING result will contain the requested information.
On error it returns:

 - GSMDEVICE_E_INIT (gsmDevice instance is not initialized)
 - GSMDEVICE_E_SUPPORTED (not yet supported e.g. on a stream device)
 - GSMDEVICE_E_CMD_MODE  (invalid command mode)

----------

**Request manufacturer specific information about the TA (ATI)**
***INT16 requestManufacturerData( gsmCommandMode cmdMode, INT16 infoValue, STRING &result, void \*pParam = NULL)***

Request manufacture specific information from the TA.
Supports: gsmCommandMode *cmd_set* only.
Expects: An INT16 value from 0 up to 255. Support of the values 1 to 255 are manufacturer dependend. On a successful return the STRING result will contain the requested information.
On error it returns:

 - GSMDEVICE_E_INIT (gsmDevice instance is not initialized)
 - GSMDEVICE_E_SUPPORTED (not yet supported e.g. on a stream device)
 - GSMDEVICE_E_CMD_MODE  (invalid command mode)

----------


... **t.b.c**

