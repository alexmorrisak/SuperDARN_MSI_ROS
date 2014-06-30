OS:=$(shell uname -s)

CC=gcc
NVCC=nvcc
CCpp=g++
CFLAGS=-c -Wall -pthread -std=c++0x -g3
CUFLAGS=-c 
LFLAGS  = -L"../iniparser3.0b" -L"${HOME}/uhd/host/build/lib" -L"/usr/local/cuda/lib64" -L"/usr/local/cuda/lib"
common_libs=-lm -liniparser -lrt -luhd -lfftw3 -lcuda -lcudart
linux_libs=-lboost_thread
qnx_libs=-lsocket
INCLUDES=-I"../include/" -I"include/" -I"../tsg/include" -I"../iniparser3.0b/src/" \
	 -I"${HOME}/uhd/host/include" -I"${HOME}/uhd/host/include/uhd/types" -I"${HOME}/uhd/host/include/uhd/usrp" 

C_SOURCES=_decodestate.c _open_ini_file.c ../_tcpsocket.c ../utils.c 
CUDA_SOURCE0=rx_process_gpu.cu
CUDA_SOURCE1=tx_process_gpu.cu
CPP_SOURCES=main.cpp convolve.cpp recv_clr_freq.cpp tx_mix_upsample.cpp recv_and_hold.cpp tx_worker.cpp \
			tx_data.cpp rx_data.cpp
C_OBJECTS=$(C_SOURCES:.c=.o)
CUDA_OBJECT0=$(CUDA_SOURCE0:.cu=.o)
CUDA_OBJECT1=$(CUDA_SOURCE1:.cu=.o)
CPP_OBJECTS=$(CPP_SOURCES:.cpp=.o)
EXECUTABLE=usrp_tcp_driver

all: $(EXECUTABLE)

$(EXECUTABLE): $(C_OBJECTS) $(CPP_OBJECTS) $(CUDA_OBJECT0) $(CUDA_OBJECT1)
	$(CCpp) -o $@ $(C_OBJECTS) $(CUDA_OBJECT0) $(CUDA_OBJECT1) $(CPP_OBJECTS) $(LFLAGS) $(common_libs) $(linux_libs) ${INCLUDES}

.c.o:
	$(CC) $(CFLAGS) $< -o $@ $(LFLAGS) $(INCLUDES)

$(CUDA_OBJECT0):
	$(NVCC) $(CUFLAGS) $(CUDA_SOURCE0) $< -o $@ $(LFLAGS) $(INCLUDES)

$(CUDA_OBJECT1):
	$(NVCC) $(CUFLAGS) $(CUDA_SOURCE1) $< -o $@ $(LFLAGS) $(INCLUDES)

.cpp.o:
	$(CCpp) $(CFLAGS) $< -o $@ $(LFLAGS) $(INCLUDES)

clean:
	rm -rf *.o usrp_tcp_driver

