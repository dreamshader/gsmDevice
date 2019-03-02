
#include "../gsmDevice.h"

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifndef linux

const int pinRx = 8;
const int pinTx = 9;

gsmDevice myDevice;

void setup() 
{
  int retVal;
  int exitCode;
  
  Serial.begin(38400);

  Serial.println("setup entry");

  if( (retVal = myDevice.init(pinRx, pinTx, 9600)) != GSMDEVICE_SUCCESS )
  {
     Serial.print("device init returns: ");
     Serial.println(retVal);
  }
  else
  {
     Serial.println("device init successful.");
     retVal = myDevice.begin(AI_A6);

      
#else
  

int main( int argc, char *argv[] )
{
    gsmDevice myDevice;
    INT16 exitCode = 0;


    if( (exitCode = myDevice.init("/dev/ttyUSB0", 9600)) != GSMDEVICE_SUCCESS )
    {
        printf("device init returns: %d\n", exitCode );
    }
    else
    {


        printf("device init successful.\n");
        exitCode = myDevice.begin(AI_A6);

#endif // linux

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

        if( exitCode == GSMDEVICE_SUCCESS )
        {
            cmdResultCodeFormat resultFormat;
            cmdEcho echoMode;
            struct _dataCMGF msgFormat;
            struct _dataCREG regMode;
            struct _dataCSQ sigQual;
            struct _dataEGMR imeiData;
            struct _dataIMSI imsiData;
            struct _dataCPOL prefOperMode;
            struct _listInString listCPOL;
            struct _listInString listCOPS;
            STRING listResultString;
            struct _dataCPOL readResult;
            struct _dataCOPS copsResult;
            struct _dataCMGS smsData;


#ifdef linux
            printf("device synchronized successful.\n");
#else
            Serial.println("device synchronized successful.");
#endif // linux

            STRING result = GSMDEVICE_EMPTY_STRING;

//            resultFormat = cmdResultText;
//            exitCode = myDevice.resultCodeFormat( cmd_execute, &resultFormat, result );
#ifdef linux
//            printf("select command response format returns: %d\n", exitCode);
//            printf("result: [%s]\n", result.c_str());
#else
//            Serial.print("select command response format returns: ");
//            Serial.println(exitCode);      
//            Serial.print("result: ");
//            Serial.println(result);
#endif // linux

//            result = GSMDEVICE_EMPTY_STRING;
//            exitCode = myDevice.smsMsgFormat( cmd_test, &msgFormat, result );
#ifdef linux
//            printf("select sms message format returns: %d\n", exitCode);
//            printf("result: [%s]\n", result.c_str());
#else
//            Serial.print("select sms message format returns: ");
//            Serial.println(exitCode);      
//            Serial.print("result: ");
//            Serial.println(result);
#endif // linux
 
#ifdef ALL_TESTS
       
            result = GSMDEVICE_EMPTY_STRING;
            exitCode = myDevice.smsMsgFormat( cmd_read, &msgFormat, result );
#ifdef linux
            printf("select sms message format returns: %d\n", exitCode);
            printf("result: [%s]\n", result.c_str());
#else
            Serial.print("select sms message format returns: ");
            Serial.println(exitCode);      
            Serial.print("result: ");
            Serial.println(result);
#endif // linux

            result = GSMDEVICE_EMPTY_STRING;
            msgFormat.current = smsPDUMode;
            exitCode = myDevice.smsMsgFormat( cmd_set, &msgFormat, result );
#ifdef linux
            printf("select sms message format returns: %d\n", exitCode);
            printf("result: [%s]\n", result.c_str());
#else
            Serial.print("select sms message format returns: ");
            Serial.println(exitCode);      
            Serial.print("result: ");
            Serial.println(result);
#endif // linux

            result = GSMDEVICE_EMPTY_STRING;
            msgFormat.current = smsTXTMode;
            exitCode = myDevice.smsMsgFormat( cmd_set, &msgFormat, result );
#ifdef linux
            printf("select sms message format returns: %d\n", exitCode);
            printf("result: [%s]\n", result.c_str());
#else
            Serial.print("select sms message format returns: ");
            Serial.println(exitCode);      
            Serial.print("result: ");
            Serial.println(result);
#endif // linux

//            result = GSMDEVICE_EMPTY_STRING;
//            // echoMode = cmdEchoOn;
//            echoMode = cmdEchoOff;
//            exitCode = myDevice.commandEcho( cmd_execute, &echoMode, result );
#ifdef linux
//            printf("set command echo returns: %d\n", exitCode);
//            printf("result: [%s]\n", result.c_str());
#else
//            Serial.print("set command echo returns: ");
//            Serial.println(exitCode);      
//            Serial.print("result: ");
//            Serial.println(result);
#endif // linux

            result = GSMDEVICE_EMPTY_STRING;
            regMode.mode = enableNetwRegUnsolLoc;
            exitCode = myDevice.networkRegistration( cmd_set, &regMode, result );
#ifdef linux
            printf("network registration returns: %d\n", exitCode);
            printf("result: [%s]\n", result.c_str());
#else
            Serial.print("network registration returns: ");
            Serial.println(exitCode);      
            Serial.print("result: ");
            Serial.println(result);
#endif // linux

            result = GSMDEVICE_EMPTY_STRING;
            regMode.mode = enableNetwRegUnsolLoc;
            exitCode = myDevice.networkRegistration( cmd_read, &regMode, result );
#ifdef linux
            printf("network registration returns: %d\n", exitCode);
            printf("result: [%s]\n", result.c_str());
#else
            Serial.print("network registration returns: ");
            Serial.println(exitCode);      
            Serial.print("result: ");
            Serial.println(result);
#endif // linux

            result = GSMDEVICE_EMPTY_STRING;
            // struct signalQuality sigQual;
            exitCode = myDevice.signalQuality( cmd_execute, &sigQual, result);
#ifdef linux
            printf("signal quality returns: %d\n", exitCode);
            printf("result: [%s]\n", result.c_str());
#else
            Serial.print("signal quality returns: ");
            Serial.println(exitCode);      
            Serial.print("result: ");
            Serial.println(result);
#endif // linux

            result = GSMDEVICE_EMPTY_STRING;
            copsResult.selectMode  = opSelectAuto;
            //copsResult.format;
            //copsResult.oper;
            exitCode = myDevice.operatorSelects( cmd_read, &copsResult, result );
#ifdef linux
            printf("operator selects returns: %d\n", exitCode);
            printf("result: [%s]\n", result.c_str());
            printf("copsResult.selectMode: %d, copsResult.format: %d, copsResult.oper: <%s>\n", 
                    copsResult.selectMode,  
                    copsResult.format, copsResult.oper.c_str() );
#else
            Serial.print("operator selects returns: ");
            Serial.println(exitCode);      
            Serial.print("result: ");
            Serial.println(result);
#endif // linux

            result = GSMDEVICE_EMPTY_STRING;
            copsResult.selectMode  = opSelectAuto;
            //copsResult.format;
            //copsResult.oper;
            exitCode = myDevice.operatorSelects( cmd_test, &copsResult, result );
#ifdef linux
            printf("operator selects returns: %d\n", exitCode);
            printf("result: [%s]\n", result.c_str());
#else
            Serial.print("operator selects returns: ");
            Serial.println(exitCode);      
            Serial.print("result: ");
            Serial.println(result);
#endif // linux


            if( (exitCode = myDevice.listManInitList( listTypeCOPS, &listCOPS, 
                                           copsResult.list)) == GSMDEVICE_SUCCESS )
            {
printf("SUCCESS on COPS list create\n");
                if( (exitCode = myDevice.listMan( &listCOPS, firstItem, NULL, 0, listResultString, 
                                       (void*) &copsResult )) ==  GSMDEVICE_SUCCESS )
                {
printf("SUCCESS on COPS list first item. _stat: %d, _longAlphaOper: <%s>, _shortAlphaOper: <%s>, _numericOper: <%s>\n", copsResult.stat, copsResult.longAlphaOper.c_str(), copsResult.shortAlphaOper.c_str(), copsResult.numericOper.c_str() );

                    while( (exitCode = myDevice.listMan( &listCOPS, nextItem, NULL, 0, listResultString, 
                                           (void*) &copsResult )) ==  GSMDEVICE_SUCCESS )
                    {
printf("SUCCESS on COPS list next item. _stat: %d, _longAlphaOper: <%s>, _shortAlphaOper: <%s>, _numericOper: <%s>\n", copsResult.stat, copsResult.longAlphaOper.c_str(), copsResult.shortAlphaOper.c_str(), copsResult.numericOper.c_str() );
                    }

                    if( (exitCode = myDevice.listMan( &listCOPS, lastItem, NULL, 0, listResultString, 
                                           (void*) &copsResult )) ==  GSMDEVICE_SUCCESS )
                    {
printf("SUCCESS on COPS list last item. _stat: %d, _longAlphaOper: <%s>, _shortAlphaOper: <%s>, _numericOper: <%s>\n", copsResult.stat, copsResult.longAlphaOper.c_str(), copsResult.shortAlphaOper.c_str(), copsResult.numericOper.c_str() );

                    
                        while( (exitCode = myDevice.listMan( &listCOPS, prevItem, NULL, 0, listResultString, 
                                               (void*) &copsResult )) ==  GSMDEVICE_SUCCESS )
                        {
printf("SUCCESS on COPS list prev item. _stat: %d, _longAlphaOper: <%s>, _shortAlphaOper: <%s>, _numericOper: <%s>\n", copsResult.stat, copsResult.longAlphaOper.c_str(), copsResult.shortAlphaOper.c_str(), copsResult.numericOper.c_str() );
                        }
                    }
                    else
                    {
printf("FAIL on COPS last list item, %d\n", exitCode);
                    }


                }
                else
                {
printf("FAIL on COPS first list item, %d\n", exitCode);
                }

            }
            else
            {
printf("FAIL on COPS list create, %d\n", exitCode);
            }





            result = GSMDEVICE_EMPTY_STRING;
            prefOperMode.index = GSMDEVICE_CMD_CPOL_NULL_INDEX;
            prefOperMode.format = prefOperLongAlphaMode;
            prefOperMode.oper = GSMDEVICE_EMPTY_STRING;
            exitCode = myDevice.preferredOperatorList( cmd_test, &prefOperMode, result );
#ifdef linux
            printf("preferred oper list returns: %d\n", exitCode);
            printf("result: [%s]\n", result.c_str());
#else
            Serial.print("preferred operator list returns: ");
            Serial.println(exitCode);      
            Serial.print("result: ");
            Serial.println(result);
#endif // linux

            result = GSMDEVICE_EMPTY_STRING;
            prefOperMode.index = GSMDEVICE_CMD_CPOL_NULL_INDEX;
            prefOperMode.format = prefOperLongAlphaMode;
            prefOperMode.oper = GSMDEVICE_EMPTY_STRING;
            exitCode = myDevice.preferredOperatorList( cmd_read, &prefOperMode, result );
#ifdef linux
            printf("preferred oper list returns: %d\n", exitCode);
//            printf("result: [%s]\n", result.c_str());
            printf("list: [%s]\n", prefOperMode.list.c_str());
#else
            Serial.print("preferred operator list returns: ");
            Serial.println(exitCode);      
//            Serial.print("result: ");
//            Serial.println(result);
            Serial.print("result list: [");
            Serial.print( prefOperMode.list );
            Serial.println("]");
#endif // linux

// for list tests comment out following line and corresponding #endif
// take care that prefOperMode.list contains data
#ifdef THIS_IS_NOT_DEFINED

            listResultString = GSMDEVICE_EMPTY_STRING;
            if( (exitCode = myDevice.listManInitList( listTypeCPOL, &listCPOL, 
                                           prefOperMode.list)) == GSMDEVICE_SUCCESS )
            {
                if( (exitCode = myDevice.listMan( &listCPOL, firstItem, NULL, 0, listResultString, 
                                       (void*) &readResult )) ==  GSMDEVICE_SUCCESS )
                {

#ifdef linux
printf("GOT first item %d\n", listCPOL._current);
printf("             [%s]\n", listResultString.c_str());

printf("readResult.index: %d, readResult.format: %d, readResult.oper: %s\n", readResult.index,  readResult.format,  readResult.oper.c_str() );

#else

Serial.print("GOT first item: ");
Serial.println(listCPOL._current);
Serial.print("             ");
Serial.println(listResultString);
Serial.print("readResult.index: ");
Serial.print(readResult.index);
Serial.print(", readResult.format: ");
Serial.print(readResult.format);
Serial.print(", readResult.oper: ");
Serial.println(readResult.oper);

#endif // linux

                    while( (exitCode = myDevice.listMan( &listCPOL, nextItem, NULL, 0, listResultString, 
                                           (void*) &readResult )) ==  GSMDEVICE_SUCCESS )
                    {
#ifdef linux
printf("GOT next item %d\n", listCPOL._current);
printf("             [%s]\n", listResultString.c_str());
printf("readResult.index: %d, readResult.format: %d, readResult.oper: %s\n", readResult.index,  readResult.format,  readResult.oper.c_str() );

#else

Serial.print("GOT next item: ");
Serial.println(listCPOL._current);
Serial.print("             ");
Serial.println(listResultString);
Serial.print("readResult.index: ");
Serial.print(readResult.index);
Serial.print(", readResult.format: ");
Serial.print(readResult.format);
Serial.print(", readResult.oper: ");
Serial.println(readResult.oper);

#endif // linux
                    }

#ifdef linux
printf("last call to nextItem returned: %d\n", exitCode);
printf("GOT next item %d\n", listCPOL._current);
printf("readResult.index: %d, readResult.format: %d, readResult.oper: %s\n", readResult.index,  readResult.format,  readResult.oper.c_str() );

#else

Serial.print("GOT next item: ");
Serial.println(listCPOL._current);
Serial.print("readResult.index: ");
Serial.print(readResult.index);
Serial.print(", readResult.format: ");
Serial.print(readResult.format);
Serial.print(", readResult.oper: ");
Serial.println(readResult.oper);

#endif // linux

                    while( (exitCode = myDevice.listMan( &listCPOL, prevItem, NULL, 0, listResultString, 
                                           (void*) &readResult )) ==  GSMDEVICE_SUCCESS )
                    {
#ifdef linux
printf("GOT prev item %d\n", listCPOL._current);
printf("             [%s]\n", listResultString.c_str());
printf("readResult.index: %d, readResult.format: %d, readResult.oper: %s\n", readResult.index,  readResult.format,  readResult.oper.c_str() );

#else

Serial.print("GOT prev item: ");
Serial.println(listCPOL._current);
Serial.print("             ");
Serial.println(listResultString);
Serial.print("readResult.index: ");
Serial.print(readResult.index);
Serial.print(", readResult.format: ");
Serial.print(readResult.format);
Serial.print(", readResult.oper: ");
Serial.println(readResult.oper);

#endif // linux
                    }

#ifdef linux
printf("last call to prevItem returned: %d\n", exitCode);
printf("             [%s]\n", listResultString.c_str());
printf("readResult.index: %d, readResult.format: %d, readResult.oper: %s\n", readResult.index,  readResult.format,  readResult.oper.c_str() );

#else

Serial.print("last call to prevItem returned: ");
Serial.println(exitCode);
Serial.print("             ");
Serial.println(listResultString);
Serial.print("readResult.index: ");
Serial.print(readResult.index);
Serial.print(", readResult.format: ");
Serial.print(readResult.format);
Serial.print(", readResult.oper: ");
Serial.println(readResult.oper);

#endif // linux
                    exitCode = myDevice.listMan( &listCPOL, lastItem, NULL, 0, listResultString, 
                                           (void*) &readResult );

#ifdef linux
printf("GOT last item %d\n", listCPOL._current);
printf("             [%s]\n", listResultString.c_str());
printf("readResult.index: %d, readResult.format: %d, readResult.oper: %s\n", readResult.index,  readResult.format,  readResult.oper.c_str() );

#else

Serial.print("GOT last item: ");
Serial.println(listCPOL._current);
Serial.print("             ");
Serial.println(listResultString);
Serial.print("readResult.index: ");
Serial.print(readResult.index);
Serial.print(", readResult.format: ");
Serial.print(readResult.format);
Serial.print(", readResult.oper: ");
Serial.println(readResult.oper);

#endif // linux


                }
                else
                {
#ifdef linux
printf("ERROR firstItem: %d\n", exitCode);
#else
Serial.print("ERROR firstItem: ");
Serial.println(exitCode);      
#endif // linux
                }
            }
            else
            {
#ifdef linux
printf("ERROR init: %d\n", exitCode);
#else
Serial.print("ERROR init: ");
Serial.println(exitCode);      
#endif // linux
            }

#endif // THIS_IS_NOT_DEFINED


            result = GSMDEVICE_EMPTY_STRING;
            //
            exitCode = myDevice.requestIMSI( cmd_test, &imsiData, result);
#ifdef linux
            printf("request IMSI returns: %d\n", exitCode);
            printf("result: [%s]\n", result.c_str());
#else
            Serial.print("request IMSI returns: ");
            Serial.println(exitCode);      
            Serial.print("result: ");
            Serial.println(result);
#endif // linux

            result = GSMDEVICE_EMPTY_STRING;
            //
            exitCode = myDevice.requestIMSI( cmd_set, &imsiData, result);
#ifdef linux
            printf("request IMSI returns: %d\n", exitCode);
            printf("result: [%s]\n", result.c_str());
#else
            Serial.print("request IMSI returns: ");
            Serial.println(exitCode);      
            Serial.print("result: ");
            Serial.println(result);
#endif // linux


            result = GSMDEVICE_EMPTY_STRING;
            imeiData.mode = imeiReadMode;
            imeiData.format = 7;
            imeiData.IMEI = GSMDEVICE_EMPTY_STRING;
            exitCode = myDevice.readWriteIMEI( cmd_set, &imeiData, result);
#ifdef linux
            printf("read/write IMEI returns: %d\n", exitCode);
            printf("result: [%s]\n", result.c_str());
#else
            Serial.print("read/write IMEI returns: ");
            Serial.println(exitCode);      
            Serial.print("result: ");
            Serial.println(result);
#endif // linux

            result = GSMDEVICE_EMPTY_STRING;
            //
            exitCode = myDevice.requestRevisionId( cmd_set, NULL, result);
#ifdef linux
            printf("request revision returns: %d\n", exitCode);
            printf("result: [%s]\n", result.c_str());
#else
            Serial.print("request revision returns: ");
            Serial.println(exitCode);      
            Serial.print("result: ");
            Serial.println(result);
#endif // linux

            result = GSMDEVICE_EMPTY_STRING;
            //
            exitCode = myDevice.requestManufacturerData( cmd_set, 0, result);
#ifdef linux
            printf("request manufacturer data returns: %d\n", exitCode);
            printf("result: [%s]\n", result.c_str());
#else
            Serial.print("request manufacturer data returns: ");
            Serial.println(exitCode);      
            Serial.print("result: ");
            Serial.println(result);
#endif // linux

#endif // ALL_TESTS

            result = GSMDEVICE_EMPTY_STRING;

->   add phone here            smsData.phoneNum = "0049...";
            smsData.message = "Servus Du!";
            //
            exitCode = myDevice.sendSMSMessage( cmd_set, &smsData, result);
#ifdef linux
            printf("send sms message returns: %d\n", exitCode);
            printf("result: [%s]\n", result.c_str());
#else
            Serial.print("send sms message returns: ");
            Serial.println(exitCode);      
            Serial.print("result: ");
            Serial.println(result);
#endif // linux



        }
        else
        {
#ifdef linux
            printf("device begin returns: %d\n", exitCode);
#else
            Serial.print("device begin returns: ");
            Serial.println(exitCode);      
#endif // linux
        }
    }

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifndef linux

  }
    
  Serial.println("setup DONE");

}

void loop() {
}

#else

    printf("DONE!\n");

    return( exitCode );
}

#endif

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
