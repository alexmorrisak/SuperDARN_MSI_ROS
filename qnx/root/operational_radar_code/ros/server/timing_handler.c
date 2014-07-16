#include <stdlib.h>
#include <pthread.h>
#include <stdio.h>
#include <sys/time.h>
#include "control_program.h"
#include "global_server_variables.h"
#include <errno.h>
#include <string.h>

extern int timingsock;
extern pthread_mutex_t timing_comm_lock;

extern int verbose;
extern struct TRTimes bad_transmit_times;
extern uint32_t* start_usec;
void *timing_ready_controlprogram(struct ControlProgram *arg)
{
  struct DriverMsg msg;
  memset(&msg, 0, sizeof(msg));
  pthread_mutex_lock(&timing_comm_lock);
   if (arg!=NULL) {
     if (arg->state->pulseseqs[arg->parameters->current_pulseseq_index]!=NULL) {
       msg.type=TIMING_CtrlProg_READY;
       msg.status=1;
       send_data(timingsock, &msg, sizeof(struct DriverMsg));
       send_data(timingsock, arg->parameters, sizeof(struct ControlPRM));
       recv_data(timingsock, &msg, sizeof(struct DriverMsg));
     } 
   }
  pthread_mutex_unlock(&timing_comm_lock);
   pthread_exit(NULL);
};

void *timing_end_controlprogram(struct ControlProgram *control_program)
{
  struct DriverMsg msg;
  pthread_mutex_lock(&timing_comm_lock);
  msg.type=TIMING_CtrlProg_END;
  msg.status=1;
  send_data(timingsock, &msg, sizeof(struct DriverMsg));
  send_data(timingsock, control_program->parameters, sizeof(struct ControlPRM));
  recv_data(timingsock, &msg, sizeof(struct DriverMsg));
  pthread_mutex_unlock(&timing_comm_lock);
   pthread_exit(NULL);
};

void *timing_register_seq(struct ControlProgram *control_program)
{
  struct DriverMsg msg;
  memset(&msg,0,sizeof(msg));
  int index;
  pthread_mutex_lock(&timing_comm_lock);
  msg.type=TIMING_REGISTER_SEQ;
  msg.status=1;
  send_data(timingsock, &msg, sizeof(struct DriverMsg));
  send_data(timingsock, control_program->parameters, sizeof(struct ControlPRM));
  index=control_program->parameters->current_pulseseq_index;
  send_data(timingsock, &index, sizeof(index)); //requested index
  send_data(timingsock,control_program->state->pulseseqs[index], sizeof(struct TSGbuf)); // requested pulseseq
  send_data(timingsock,control_program->state->pulseseqs[index]->rep, 
    sizeof(unsigned char)*control_program->state->pulseseqs[index]->len); // requested pulseseq
  send_data(timingsock,control_program->state->pulseseqs[index]->code, 
    sizeof(unsigned char)*control_program->state->pulseseqs[index]->len); // requested pulseseq
  recv_data(timingsock, &msg, sizeof(struct DriverMsg));
  pthread_mutex_unlock(&timing_comm_lock);
  pthread_exit(NULL);
}
void *timing_pretrigger(void *arg)
{
  int rval,i;
  //uint32_t* start_usec;
  rval=0;
  struct DriverMsg msg;
  memset(&msg,0,sizeof(msg));
  pthread_mutex_lock(&timing_comm_lock);
  msg.type=TIMING_PRETRIGGER;
  msg.status=1;
  printf("TIMING: PRETRIGGER: Entering pretrigger\nSend msg\n");
  send_data(timingsock, &msg, sizeof(struct DriverMsg));
  //printf("TIMING: PRETRIGGER: free %p %p\n",bad_transmit_times.start_usec,bad_transmit_times.duration_usec);
  if(bad_transmit_times.start_usec!=NULL) free(bad_transmit_times.start_usec);
  if(bad_transmit_times.duration_usec!=NULL) free(bad_transmit_times.duration_usec);
  bad_transmit_times.start_usec=NULL;
  bad_transmit_times.duration_usec=NULL;
  //printf("TIMING: PRETRIGGER: free end\n");
  //printf("TIMING: PRETRIGGER: recv bad_transmit times object\n");
  rval=recv_data(timingsock, &bad_transmit_times.length, sizeof(bad_transmit_times.length));
  if (rval<=0) {printf("\nERROR in recv_data()!!%s\n\n",strerror(errno));}
  printf("TIMING: PRETRIGGER: length %d %i\n",bad_transmit_times.length, rval);
  if (bad_transmit_times.length>0) {
    //printf("TIMING: PRETRIGGER: Mallocs start\n");
    bad_transmit_times.start_usec=(uint32_t*) malloc(sizeof(uint32_t)*bad_transmit_times.length);
    memset(bad_transmit_times.start_usec, 0, sizeof(uint32_t)*bad_transmit_times.length);
    if (bad_transmit_times.start_usec==NULL) printf("ERROR in malloc!!\n");
    bad_transmit_times.duration_usec=(uint32_t*) malloc(sizeof(uint32_t)*bad_transmit_times.length);
    memset(bad_transmit_times.duration_usec, 0, sizeof(uint32_t)*bad_transmit_times.length);
    if (bad_transmit_times.duration_usec==NULL) printf("ERROR in malloc!!\n");
    //printf("TIMING: PRETRIGGER: Mallocs end\n");
  } else {
    bad_transmit_times.start_usec=NULL;
    bad_transmit_times.duration_usec=NULL;
  }
  //printf("TIMING: PRETRIGGER: recv start usec object %i\n",rval);
  rval=recv_data(timingsock, (uint32_t*)bad_transmit_times.start_usec, sizeof(uint32_t)*bad_transmit_times.length);
  if (rval<=0) {printf("\nERROR in recv_data()\n%s!!\n\n",strerror(errno));}

  //printf("TIMING: PRETRIGGER: recv duration usec object %i\n", rval);
  rval=recv_data(timingsock, (uint32_t*)bad_transmit_times.duration_usec, sizeof(uint32_t)*bad_transmit_times.length);
  //send_data(timingsock, &rval,sizeof(int));
  //printf("TIMING: PRETRIGGER: recv msg %i\n",rval);
  rval=recv_data(timingsock, &msg, sizeof(struct DriverMsg));
  if (rval<=0) {printf("\nERROR in recv_data()!!%s\n\n",strerror(errno));}

  printf("TIMING: PRETRIGGER: done recv msg %i\n",rval);
  pthread_mutex_unlock(&timing_comm_lock);
  printf("TIMING: PRETRIGGER: exit\n");
  pthread_exit(NULL);
};

void *timing_trigger(int trigger_type)
{
  struct DriverMsg msg;
  pthread_mutex_lock(&timing_comm_lock);
  switch(trigger_type) {
    case 0:
      msg.type=TIMING_TRIGGER;
      break;
    case 1:
      msg.type=TIMING_TRIGGER;
      break;
    case 2:
      msg.type=TIMING_GPS_TRIGGER;
      break;
  }
  msg.status=1;
  send_data(timingsock, &msg, sizeof(struct DriverMsg));
  recv_data(timingsock, &msg, sizeof(struct DriverMsg));
  pthread_mutex_unlock(&timing_comm_lock);
  pthread_exit(NULL);
};

void *timing_wait(void *arg)
{
  struct DriverMsg msg;
  memset(&msg, 0, sizeof(msg));
  pthread_mutex_lock(&timing_comm_lock);
  msg.type=TIMING_WAIT;
  msg.status=1;
  send_data(timingsock, &msg, sizeof(struct DriverMsg));
  recv_data(timingsock, &msg, sizeof(struct DriverMsg));
  pthread_mutex_unlock(&timing_comm_lock);
  pthread_exit(NULL);
};
void *timing_posttrigger(void *arg)
{
  struct DriverMsg msg;
  pthread_mutex_lock(&timing_comm_lock);

   msg.type=TIMING_POSTTRIGGER;
   msg.status=1;
   send_data(timingsock, &msg, sizeof(struct DriverMsg));
   recv_data(timingsock, &msg, sizeof(struct DriverMsg));
   pthread_mutex_unlock(&timing_comm_lock);
   pthread_exit(NULL);
};

/*
void *timing_handler(void *arg)
{

  pthread_mutex_lock(&timing_comm_lock);
   if (verbose>1) fprintf(stderr,"Inside the timing handler\n");
   if (verbose>1) fprintf(stderr,"Timing: Do some work\n");
   if (verbose>1) fprintf(stderr,"Leaving timing handler\n");
  pthread_mutex_unlock(&timing_comm_lock);
   pthread_exit(NULL);
};
*/

