OS:=$(shell uname -s)

CC=gcc
CCpp=g++
CFLAGS=-c -Wall -pthread -std=c++0x -g3
LFLAGS  = -L"../iniparser3.0b" -L"${HOME}/uhd/host/build/lib"
common_libs=-lm -liniparser -lrt -luhd -lfftw3
linux_libs=-lboost_thread
qnx_libs=-lsocket
INCLUDES=-I"../include/" -I"include/" -I"../tsg/include" -I"../iniparser3.0b/src/" \
	 -I"${HOME}/uhd/host/include" -I"${HOME}/uhd/host/include/uhd/types" -I"${HOME}/uhd/host/include/uhd/usrp" 

C_SOURCES=_dma.c _open_PLX9080.c _create_DMA_buff.c _decodestate.c _open_ini_file.c ../_tcpsocket.c ../utils.c 
CPP_SOURCES=main.cpp convolve.cpp recv_to_buffer.cpp recv_clr_freq.cpp tx_mix_upsample.cpp recv_and_hold.cpp rx_mix_downsample.cpp
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

