#include <pthread.h>
#include <stdint.h>
#include <sys/time.h>
#include "tsg.h"
#include "rtypes.h"

#ifndef TCPSOCKET_H
#define TCPSOCKET_H

#ifdef __cplusplus
	extern "C" {
#endif

int tcpsocket(int port);

#ifdef __cplusplus
	}
#endif

#endif
