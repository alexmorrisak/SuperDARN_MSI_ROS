OS:=$(shell uname -s)

CC=gcc
NVCC=nvcc
CCpp=g++
CFLAGS=-c -Wall -pthread -std=c++0x -g3
CUFLAGS=-c 
LFLAGS  = -L"../iniparser3.0b" -L"${HOME}/uhd/host/build/lib" -L"/usr/local/cuda/lib64" -L"/usr/local/cuda/lib"
common_libs=-lm -liniparser -lrt -luhd -lfftw3 -lcuda -lcudart
linux_libs=-lboost_thread -lboost_system
INCLUDES=-I"../include/" -I"include/" -I"../tsg/include" -I"../iniparser3.0b/src/" \
	 -I"${HOME}/uhd/host/include" -I"${HOME}/uhd/host/include/uhd/types" -I"${HOME}/uhd/host/include/uhd/usrp" 
C_SOURCES=
CUDA_SOURCE0=rx_process_gpu.cu
CUDA_SOURCE1=tx_process_gpu.cu
CPP_SOURCES=main.cpp recv_clr_freq.cpp recv_and_hold.cpp tx_worker.cpp \
			tx_data.cpp rx_data.cpp rx_beamform.cpp socket_utils.cpp
C_OBJECTS=$(C_SOURCES:.c=.o)
CUDA_OBJECT0=$(CUDA_SOURCE0:.cu=.o)
CUDA_OBJECT1=$(CUDA_SOURCE1:.cu=.o)
CPP_OBJECTS=$(CPP_SOURCES:.cpp=.o)
EXECUTABLE=usrp_tcp_server

all: $(EXECUTABLE)

$(EXECUTABLE): $(CPP_OBJECTS) rx_process_gpu.o tx_process_gpu.o
	$(CCpp) -o $@ rx_process_gpu.o tx_process_gpu.o $(CPP_OBJECTS) $(LFLAGS) $(common_libs) $(linux_libs) ${INCLUDES}

rx_process_gpu.o : rx_process_gpu.cu
	nvcc $(CUFLAGS) rx_process_gpu.cu -o rx_process_gpu.o

tx_process_gpu.o : tx_process_gpu.cu
	nvcc $(CUFLAGS) tx_process_gpu.cu -o tx_process_gpu.o 

.cpp.o:
	$(CCpp) $(CFLAGS) $< -o $@ $(LFLAGS) $(INCLUDES)

clean:
	rm -rf *.o $(EXECUTABLE)

