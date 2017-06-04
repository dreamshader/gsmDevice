#
CC  = gcc -Wall
CPP = g++
CC_DEBUG = -g -DDEBUG -DDEBUG_ALL
CC_DEFINES =
ADD_INCLDIR = 
ADD_LIBDIR = 
#
ADD_LIBS =
#
#
GSM_SRC = testGSM.c gsmDevice.cpp
GSM_HDR = gsmDevice.h
#
GSM = testGSM
#
#
#
all:	$(GSM)

$(GSM): $(GSM_SRC) $(GSM_HDR)
	$(CPP) -o $(GSM) $(GSM_SRC) $(ADD_LIBS)

clean:
	rm -f *.o *~ 


