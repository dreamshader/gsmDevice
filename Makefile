#
CC  = gcc -Wall
CPP = g++ -std=gnu++11
CC_DEBUG = -g -DDEBUG -DDEBUG_ALL
CC_DEFINES =
ADD_INCLDIR = 
ADD_LIBDIR = 
#
ADD_LIBS =
#
#
TESTPGM_SRC = testGSM.c
GSM_SRC = gsmDevice.cpp
GSM_HDR = gsmDevice.h
#
TESTPGM = testGSM
#
PARSER = responseParser
PARSER_SRC = responseParser.cpp
#
#
all:	$(GSM)

$(TESTPGM): $(GSM_SRC) $(GSM_HDR) $(TESTPGM_SRC)
	$(CPP) -o $(TESTPGM) $(CC_DEBUG) $(GSM_SRC) $(TESTPGM_SRC) $(ADD_LIBS)

$(PARSER): $(GSM_SRC) $(GSM_HDR) $(PARSER_SRC)
	$(CPP) -o $(PARSER) $(CC_DEBUG) $(GSM_SRC) $(PARSER_SRC) $(ADD_LIBS)


clean:
	rm -f *.o *~ 


