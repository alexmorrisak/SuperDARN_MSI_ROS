OS:=$(shell uname -s)

CC=gcc
CCpp=g++
CFLAGS=-c
LFLAGS  = -L"../iniparser3.0b" -L"${HOME}/uhd/host/build/lib"
common_libs=-lm -liniparser -lrt -luhd
linux_libs=-lboost_thread
qnx_libs=-lsocket
INCLUDES=-I"../include/" -I"include/" -I"../tsg/include" -I"../iniparser3.0b/src/" \
	 -I"${HOME}/uhd/host/include" -I"${HOME}/uhd/host/include/uhd/types" -I"${HOME}/uhd/host/include/uhd/usrp"
	 #-I"${HOME}/uhd/host/include/uhd/utils" -I"${HOME}/uhd/host/include/uhd/transport" \
	 #-I"${HOME}/uhd/host/lib" -I"${HOME}/uhd/host/lib/types" -I"${HOME}/uhd/host/lib/usrp" \
	 #-I"${HOME}/uhd/host/lib/convert" 
C_SOURCES=_dma.c _open_PLX9080.c _create_DMA_buff.c _decodestate.c _open_ini_file.c ../_tcpsocket.c ../utils.c 
CPP_SOURCES=main.cpp
C_OBJECTS=$(C_SOURCES:.c=.o)
CPP_OBJECTS=$(CPP_SOURCES:.cpp=.o)
EXECUTABLE=usrp_tcp_driver

all: $(EXECUTABLE)

$(EXECUTABLE): $(C_OBJECTS) $(CPP_OBJECTS)
	$(CCpp) -o $@ $(C_OBJECTS) $(CPP_OBJECTS) $(LFLAGS) $(common_libs) $(linux_libs) ${INCLUDES}

.c.o:
	$(CC) $(CFLAGS) $< -o $@ $(LFLAGS) $(INCLUDES)

.cpp.o:
	$(CCpp) $(CFLAGS) $< -o $@ $(LFLAGS) $(INCLUDES)

clean:
	rm -rf *.o usrp_tcp_driver

