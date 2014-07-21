#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <netdb.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <time.h>

#define X_BIT 0x04 //Transmit bit
#define TR_BIT 0x02 //TR bit
#define S_BIT 0x80 //Scope sync
#define P_BIT 0x10 //Phase bit.  0 for 0 degrees, 1 for 180 degrees

int decodestate(int r,int c,char state){

	int code;
	code=0;

	if( (state & X_BIT) == X_BIT){
		if ((state & P_BIT) == P_BIT){ code |= 0x00040000; // 180 degress
		}
		else {code |= 0x00020000; // 0 degrees
		}
	}
	if( (state & TR_BIT) == TR_BIT) code |= 0x00010000; //LSB of in-phase component
	if( (state & S_BIT) == S_BIT ) code |= 0x00000001; //LSB of q-phase component

	return code;
}

