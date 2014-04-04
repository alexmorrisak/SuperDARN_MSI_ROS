#define MIN(a,b)  ((a) < (b) ? (a) : (b))
#define MAX(a,b)  ((a) > (b) ? (a) : (b))

#ifdef __cplusplus
	extern "C" {
#endif

int tcpsocket(int port);
int send_data(int fd,void  *buf,size_t buflen);
int recv_data(int fd,void *buf,size_t buflen);

#ifdef __cplusplus
	}
#endif
