#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/un.h>
#include <sys/socket.h>
//#ifdef __QNX__
//  #include <hw/inout.h>
//  #include <sys/socket.h>
//  #include <sys/neutrino.h>
//  #include <netinet/in.h>
//  #include <netinet/tcp.h>
//#endif
#include <signal.h>
#include <netdb.h>
#include <stdio.h>
#include <string>
#include <time.h>
#include <iostream>
#include <complex>
#include <cstdlib>

//Added by Alex for usrp
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/utils/thread_priority.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/utils/static.hpp>
#include <uhd/exception.hpp>

#include <boost/thread.hpp>
#include <boost/format.hpp>

//Added by Alex for signal processing
//#include <alex_filters.hpp>

#ifdef __cplusplus
	extern "C" {
#endif
	#include "control_program.h"
	#include "global_server_variables.h"
	#include "utils.h"
	#include "timing_defs.h"
	#include "_regs_PLX9080.h"
	#include "iniparser.h"
	//#include "tcpsocket.h"
	#include "decodestate.h"
#ifdef __cplusplus
	}
#endif

#define MAX_TSG 16
#define	MAX_TIME_SEQ_LEN 1048576
#define MAX_PULSES 100

dictionary *Site_INI;
int sock,msgsock;
int verbose=10;
int configured=1;
int		writingFIFO=0, dma_count=0, under_flag=0,empty_flag=0,IRQ, intid;
int		max_seq_count, xfercount, totransfer;
uintptr_t	mmap_io_ptr_dio;
unsigned int	virtual_addr[1], physical_addr[1];
struct sigevent interruptevent;
int tr_event=0, scope_event=0;

pthread_t int_thread;
void graceful_cleanup(int signum)
{
  int temp;
  char path[256];
  sprintf(path, "%s", "rostiming");

//#ifdef __QNX__
//  // disable interrupts
//  temp=in32( mmap_io_ptr_dio+0x0c);
//  out32(mmap_io_ptr_dio+0x0c, temp & 0xffffff00);
//  //clear interrupt status
//  temp=in32(mmap_io_ptr_dio+0x0c);
//  temp|=0x04;
//  out32(mmap_io_ptr_dio+0x0c, temp);
////  InterruptDetach(intid);
//#endif
  close(msgsock);
  close(sock);
  std::cout << "Unlinking Unix Socket: " << path << "\n";
  unlink(path);

  exit(0);
}

//const struct sigevent* isr_handler(void *arg, int id){
//	int temp;
//#ifdef __QNX__
	//if interrupt created by timing card, then clear it
//	temp=in32(mmap_io_ptr_dio+0x0c);
//	if( (temp & 0x04) == 0x04 ){
//                temp|=0x04;
//                out32(mmap_io_ptr_dio+0x0c, temp);
//		return(&interruptevent);
//	}
//	else{
//		return(NULL);
//	}
//#else
//		return(NULL);
//#endif	
//}

//void * int_handler(void *arg){
//  int temp;
//  unsigned long elapsed;
//  struct timeval t0,t1,t2,t3,t4,t5,t6;
////#ifdef __QNX__
////        setprio(0,20);
////        memset(&interruptevent,0,sizeof(interruptevent));
////        interruptevent.sigev_notify=SIGEV_INTR;
////        ThreadCtl(_NTO_TCTL_IO, NULL);
////        intid=InterruptAttach(IRQ, isr_handler, NULL, NULL, NULL);
//        while(1){
//              if(writingFIFO) {
////                printf("Waiting for DMA transfer to complete\n");
//                if(xfercount<max_seq_count){
////                  InterruptWait(NULL,NULL);
//                  setprio(0,100);
//                  while(! (in32(mmap_io_ptr_dio+0x0c) & 0x04)) usleep(10);
//                  //clear interrupt flag
//                  temp=in32(mmap_io_ptr_dio+0x0c);
//                  temp|=0x04;
//                  out32(mmap_io_ptr_dio+0x0c, temp);
//                  setprio(0,20);
//                  if( (max_seq_count-xfercount) > FIFOLVL ){
//                        totransfer=FIFOLVL;
//                  }
//                  else{
//                        totransfer=max_seq_count-xfercount;
//                  }
//                  if(xfercount<max_seq_count){
////                      printf("DMA transfer\n"); 
//                      //usleep(100000); 
//	              empty_flag=in32(mmap_io_ptr_dio+0x04) & 0x1000 ; 
//	              under_flag=in32(mmap_io_ptr_dio+0x04) & 0x400 ; 
//                      xfercount+=totransfer;
//                      dma_count++; 
//                  }
//                  if(xfercount>=max_seq_count) {
////                    printf("END DMA transfers 1\n");
//                    writingFIFO=0;
//                  }
//                  if (empty_flag || under_flag) {
//                    std::cout << "DMA Error\n";
//                    writingFIFO=0;
//                  }
//                } else {
////                  printf("END DMA transfers 2\n");
//                  writingFIFO=0;
//                }
//              } else {
//                usleep(100);
//              }
//        }
//#endif
//        pthread_exit(NULL);
//}
int filter(std::vector<std::complex<float> >& signal, const std::vector<std::complex<float> >& taps){
        std::vector<std::complex<float> > output;
        std::complex<float>  temp;
        int i,j;
	output.clear();

        for (i=0; i<taps.size(); i++){
                temp = std::complex<float>(0,0);
                for(j=0; j<(i+1); j++){
                        temp += signal[i+j] * taps[taps.size()-j];
                }
                output.push_back(temp);
        }

        for (i=0; i<(signal.size()-taps.size()); i++){
                temp = std::complex<float>(0,0);
                for(j=0; j<taps.size(); j++){
                        temp += signal[i+j] * taps[j];
                }
                output.push_back(temp);
        }

        for (i=0; i<signal.size(); i++){
                signal[i] = output[i] / std::complex<float> (30,0);
        }
        return 1;
}


int main(){
    // DECLARE AND INITIALIZE ANY NECESSARY VARIABLES
        int     maxclients=MAX_RADARS*MAX_CHANNELS+1;
        struct  ControlPRM  clients[maxclients],client ;
        struct  TSGbuf *pulseseqs[MAX_RADARS][MAX_CHANNELS][MAX_SEQS];
        struct  TSGprm *tsgparams[MAX_RADARS][MAX_CHANNELS][MAX_SEQS];
	unsigned int	*seq_buf[MAX_RADARS][MAX_CHANNELS];
        int seq_count[MAX_RADARS][MAX_CHANNELS];
        int old_pulse_index[MAX_RADARS][MAX_CHANNELS];
        int ready_index[MAX_RADARS][MAX_CHANNELS];
	unsigned int	*master_buf;
        int old_seq_id=-10;
        int new_seq_id=-1;
	int old_beam=-1;
	int new_beam=-1;
        struct TRTimes bad_transmit_times, bad_transmit_temp;
        unsigned int bad_transmit_counter=0;


	// socket and message passing variables
	int	data;
	char	datacode;
	int	rval;
        fd_set rfds,efds;

	// counter and temporary variables
	int	i,j,k,r,c,buf,index,offset_pad;
	int	dds_offset,rx_offset,tx_offset;
        int     scope_start,dds_trigger,rx_trigger;
	int 	temp;
	int	tempint;
	char	tempchar;
	int	status,dead_flag,step;
	int	tempcode;
        struct timeval t0,t1,t2,t3,t4,t5,t6;
        unsigned long elapsed;

	// function specific message variables
        int     numclients=0;
        struct  DriverMsg msg;

	// timing related variables
        struct timeval tv;
	struct	timespec	start, stop, sleep, now;
	float	ftime;
	int	clockresolution;
	time_t	tod;

	// usrp-related variables
	typedef std::complex<int16_t> sc16;
	std::vector<std::complex<float> > usrp_complex_buf;
	std::vector<std::complex<float> > usrp_complex_buf1;
	std::vector<sc16> usrp_buf(4*MAX_TIME_SEQ_LEN);
	std::vector<sc16> usrp_buf1(4*MAX_TIME_SEQ_LEN);
	std::vector<sc16 *> buffs(1, &usrp_buf.front());
	std::vector<std::complex<float> > filter_table (30, std::complex<float> (0,0));
	//std::comstd::complex<float> filter_table[60] = std::complex<float>(1,0);
	int ifilter = 0;
	int filter_table_len = 30;
	buffs.push_back(&usrp_buf1.front());
	std::vector<int16_t> usrp_real;
	std::vector<int16_t> usrp_imag;
	float td, pd, pdreal, pdimag;

	//variables to be set by po
	std::string args, ant, subdev, ref, otw;
	size_t spb;
	double rate, freq, gain, wave_freq, bw;
	float ampl;

	// Here are the hard-wired values, NOT set by the po...:
	args = "addr0=192.168.10.2, addr1=192.168.10.3";
	subdev = "A:A";
	ref = "internal";
	otw = "sc16";

	// Set the center frequency
	freq = 10750000;

	//create a usrp device
	std::cout << std::endl;
	std::cout << boost::format("Creating the usrp device with: %s...") % args << std::endl;
	uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(args);

	//Lock mboard clocks
	//usrp->set_clock_source("internal", 0); //master
	std::cout << "hello 1" << std::endl;
	UHD_ASSERT_THROW(usrp->get_num_mboards() == 2);
	
	uhd::time_spec_t time0 = usrp->get_time_now(0);
	uhd::time_spec_t time1 = usrp->get_time_now(1);
	std::cout << boost::format("Time 0: %d") % time0.get_real_secs() << std::endl;
	std::cout << boost::format("Time 1: %d") % time1.get_real_secs() << std::endl;
	
	//make mboard 1 a slave over the MIMO Cable
	usrp->set_clock_source("mimo", 1);
	usrp->set_time_source("mimo", 1);
	
	//set time on the master (mboard 0)
	usrp->set_time_now(uhd::time_spec_t(0.0), 0);
	
	//sleep a bit while the slave locks its time to the master
	boost::this_thread::sleep(boost::posix_time::milliseconds(100));
	//t0 = usrp->get_time_now(0);
	//t1 = usrp->get_time_now(1);
	//std::cout << boost::format("Time 0: %d") % t0.get_real_secs() << std::endl;
	//std::cout << boost::format("Time 1: %d") % t1.get_real_secs() << std::endl;

	//Set clock reference
	//usrp->set_clock_source(ref);

	//Specify daughterboard routing
	usrp->set_tx_subdev_spec(subdev);

	//Set sample rate
	std::cout << boost::format("Setting TX Rate: %f kHz...") % (1e3/(STATE_TIME)) << std::endl;
	usrp->set_tx_rate(1e6 / STATE_TIME);
	std::cout << boost::format("Actual TX Rate: %f kHz...") % (usrp->get_tx_rate()/1e3) << std::endl << std::endl;


	//create a transmit streamer
	//linearly map channels (index0 = channel0, index1 = channel1, ...)
	uhd::stream_args_t stream_args("sc16", otw);
	for (size_t chan = 0; chan < usrp->get_tx_num_channels(); chan++)
	    stream_args.channels.push_back(chan); //linear mapping
	uhd::tx_streamer::sptr tx_stream = usrp->get_tx_stream(stream_args);

	//Initialize motherboard clock to arbitrary time
	//std::cout << boost::format("Setting device timestamp to 0...") << std::endl;
	//usrp->set_time_now(uhd::time_spec_t(0.0));

	std::cout << boost::format("Using Device: %s") % usrp->get_pp_string() << std::endl;

//#ifdef __QNX__
//	struct	 _clockperiod 	new, old;
//
//#endif
	// pci, io, and memory variables
	// Vestigial variables from timing driver; probably unnecessary for usrp
	unsigned int	*mmap_io_ptr;
	int		pci_handle;
	int		pci_device=0;

// PCI-7300A variables
	int		pci_handle_dio, IRQ_dio, mmap_io_dio;
        
//	int		 pseq[7]={0, 3, 4, 6}, scope_sync[16384], TR[16384], TX[16384], TX_array[16384], 
//                         trigger[16384], FIFOlevel[16384];
//	int		 tau=2400, tperiod=1, tlength=300, time_array[10], intt=200, loopcount=0, fifocnt=0;
        int delay_count;
        unsigned long counter;

        signal(SIGINT, graceful_cleanup);
        signal(SIGTERM, graceful_cleanup);

	Site_INI=NULL;
	//temp=_open_ini_file();
        //if(temp < 0 ) {
        //        std::cerr << "Error opening Site ini file, exiting driver\n";
        //        exit(temp);
        //}
        tx_offset=iniparser_getint(Site_INI,"timing:tx_offset",TX_OFFSET);
        dds_offset=iniparser_getint(Site_INI,"timing:dds_trigger_offset",DDS_OFFSET);
        rx_offset=iniparser_getint(Site_INI,"timing:rx_trigger_offset",RX_OFFSET);
        if (verbose > -1 ) std::cerr << "DDS Offset: " << dds_offset << " RX Offset: " << rx_offset << "\n";

        max_seq_count=0;
	if (verbose > 1) std::cout << "Zeroing arrays\n";

	for (r=0;r<MAX_RADARS;r++){
	  for (c=0;c<MAX_CHANNELS;c++){
	    if (verbose > 1) std::cout << r << " " << c << "\n";
	    for (i=0;i<MAX_SEQS;i++) pulseseqs[r][c][i]=NULL;
            ready_index[r][c]=-1; 
            old_pulse_index[r][c]=-1; 
            seq_buf[r][c]=(unsigned int*) malloc(4*MAX_TIME_SEQ_LEN);
          } 
        }
        bad_transmit_times.length=0;
        bad_transmit_times.start_usec=(uint32_t*) malloc(sizeof(unsigned int)*MAX_PULSES);
        bad_transmit_times.duration_usec=(uint32_t*) malloc(sizeof(unsigned int)*MAX_PULSES);

	for (i=0; i<filter_table_len; i++){
		filter_table[i] = pow(2.7183, -(0.5)*pow(((float)i-(filter_table_len-1)/2)/filter_table_len/0.25,2));
	}
       
//#ifdef __QNX__
//    // SET THE SYSTEM CLOCK RESOLUTION AND GET THE START TIME OF THIS PROCESS 
//	// set the system clock resolution to 10 us
//	new.nsec=10000;
//	new.fract=0;
//	temp=ClockPeriod(CLOCK_REALTIME,&new,0,0);
//	if(temp==-1) 	perror("Unable to change system clock resolution");
//	temp=ClockPeriod(CLOCK_REALTIME,0,&old,0);
//	if(temp==-1) 	perror("Unable to read sytem time");
//	clockresolution=old.nsec;
//    /* OPEN THE PLX 9080 AND GET LOCAL BASE ADDRESSES */
//	clock_gettime(CLOCK_REALTIME, &start);
//	IRQ=IRQ_dio;
//	std::cout << "PLX9080 configuration IRQ: " << IRQ << "\n";
//	clock_gettime(CLOCK_REALTIME, &stop);
//	if(temp==-1) {
//	 std::cerr << "PLX9080 configuration failed\n";
//         configured=0;
//        }
//	std::cerr << " EXECUTION TIME: " << stop.tv_nsec-start.tv_nsec << " nsec\n";
//    /* CREATE DMA BUFFERS FOR ALL RECIEVER CHANNELS */
//	clock_gettime(CLOCK_REALTIME, &start);
//#endif

//#ifdef __QNX__
//	temp=_create_DMA_buff(&virtual_addr[0], &physical_addr[0], 4*MAX_TIME_SEQ_LEN);
//	master_buf=(unsigned int*)virtual_addr[0];
//	if (temp==-1){
//	  std::cerr << "ERROR MAKING DMA BUFFERS!\n";
//        }
//#else
        master_buf=(unsigned int*) malloc(4*MAX_TIME_SEQ_LEN);
//#endif

	clock_gettime(CLOCK_REALTIME, &stop);
	if (temp == 1)	std::cerr << "DMA buffers created sucessfully!\n";
	std::cerr << " EXECUTION TIME: " << stop.tv_nsec-start.tv_nsec << " nsec\n";

    // OPEN TCP SOCKET AND START ACCEPTING CONNECTIONS 
	//printf("timing host port: %i \n", TIMING_HOST_PORT);
	sock=tcpsocket(TIMING_HOST_PORT);
	//bind(sock, (struct sockaddr *) TIMING_HOST_IP, sizeof(struct sockaddr_un));
	//printf("Done binding socket: %i \n", sock);
        //sock=server_unixsocket("/tmp/rostiming",0);
	listen(sock, 5);
	while(1){
                rval=1;
		//printf("sock: %i \n", sock);
		msgsock=accept(sock, 0, 0);
		//printf("msgsock: %i \n", msgsock);
		if (verbose > 0) std::cout << "accepting socket!!!!!\n";
		if( (msgsock==-1) ){
			perror("accept FAILED!");
			return EXIT_FAILURE;
		}
		else while (rval>=0){
                  /* Look for messages from external client process */
                  FD_ZERO(&rfds);
                  FD_SET(msgsock, &rfds); //Add msgsock to the read watch
                  FD_ZERO(&efds);
                  FD_SET(msgsock, &efds);  //Add msgsock to the exception watch
                  /* Wait up to five seconds. */
                  tv.tv_sec = 5;
                  tv.tv_usec = 0;
		  if (verbose > 1) std::cout << msgsock << " Entering Select\n";
                  rval = select(msgsock+1, &rfds, NULL, &efds, NULL);
                  //rval = select(msgsock+1, &rfds, NULL, &efds, &tv); //Actually implement the timeout (AFM)
		  if (verbose > 1) std::cout << msgsock << " Leaving Select " << rval << "\n";
                  /* tv value might be modified -- Don’t rely on the value of tv now! */
                  if (FD_ISSET(msgsock,&efds)){
                    if (verbose > 1) std::cout << "Exception on msgsock " << msgsock << "  ...closing\n";
                    break;
                  }
                  if (rval == -1) perror("select()");
                  rval=recv(msgsock, &buf, sizeof(int), MSG_PEEK); 
                  if (verbose>1) std::cout << msgsock << " PEEK Recv Msg " << rval << "\n";
		  if (rval==0) {
                    if (verbose > 1) std::cout << "Remote Msgsock " << msgsock << " client disconnected ...closing\n";
                    break;
                  } 
		  if (rval<0) {
                    if (verbose > 0) std::cout << "Msgsock " << " Error ...closing\n";
                    break;
                  } 
                  if ( FD_ISSET(msgsock,&rfds) && rval>0 ) {
                    if (verbose>1) std::cout << "Data is ready to be read\n";
		    if (verbose > 1) std::cout << msgsock << " Recv Msg\n";
                    rval=recv_data(msgsock,&msg,sizeof(struct DriverMsg));
                    datacode=msg.type;
		    if (verbose > 1) std::cout << "\nmsg code is " <<  datacode << "\n";
		    switch( datacode ){
		      case TIMING_REGISTER_SEQ:
		        if (verbose > 0) std::cout << "\nRegister new timing sequence for timing card\n";
		        msg.status=0;
                        rval=recv_data(msgsock,&client,sizeof(struct ControlPRM));
                        r=client.radar-1; 
                        c=client.channel-1; 
			if (verbose > 1) std::cout << "Radar: " << client.radar <<
				" Channel: " << client.channel << " Beamnum: " << client.tbeam <<
				" Status: " << msg.status << "\n";
		        rval=recv_data(msgsock,&index,sizeof(index));
		        if (verbose > 1) std::cout << "Requested index: " << r << " " << c << " " << index << "\n";
		        if (verbose > 1) std::cout << "Attempting Free on pulseseq: " << pulseseqs[r][c][index];
                        if (pulseseqs[r][c][index]!=NULL) {
                          if (pulseseqs[r][c][index]->rep!=NULL)  free(pulseseqs[r][c][index]->rep);
                          if (pulseseqs[r][c][index]->code!=NULL) free(pulseseqs[r][c][index]->code);
                          free(pulseseqs[r][c][index]);
                        }
		        if (verbose > 1) std::cout << "Done Free - Attempting Malloc\n";	
                        pulseseqs[r][c][index]=(TSGbuf*) malloc(sizeof(struct TSGbuf));
		        if (verbose > 1) std::cout << "Finished malloc\n";
                        rval=recv_data(msgsock,pulseseqs[r][c][index], sizeof(struct TSGbuf)); // requested pulseseq
                        pulseseqs[r][c][index]->rep=
                          (unsigned char*) malloc(sizeof(unsigned char)*pulseseqs[r][c][index]->len);
                        pulseseqs[r][c][index]->code=
                          (unsigned char*) malloc(sizeof(unsigned char)*pulseseqs[r][c][index]->len);
                        rval=recv_data(msgsock,pulseseqs[r][c][index]->rep, 
                          sizeof(unsigned char)*pulseseqs[r][c][index]->len);
                        rval=recv_data(msgsock,pulseseqs[r][c][index]->code, 
                          sizeof(unsigned char)*pulseseqs[r][c][index]->len);
			if (verbose > 1) std::cout << "Pulseseq length: " << pulseseqs[r][c][index]->len << "\n";
                        old_seq_id=-10;
                        old_pulse_index[r][c]=-1;
                        new_seq_id=-1;
                        rval=send_data(msgsock, &msg, sizeof(struct DriverMsg));
                        break;
		      case TIMING_CtrlProg_END:
		        if (verbose > 0) std::cout << "\nA client is done\n";	
                        msg.status=0;
                        old_seq_id=-10;
                        new_seq_id=-1;
                        break;

		      case TIMING_CtrlProg_READY:
		        if (verbose > 0) std::cout << "\nAsking to set up timing info for client that is ready " << numclients << "\n";
                        msg.status=0;
		        rval=recv_data(msgsock,&client,sizeof(struct ControlPRM));
                        r=client.radar-1; 
                        c=client.channel-1; 
                        if ((ready_index[r][c]>=0) && (ready_index[r][c] <maxclients) ) {
                          clients[ready_index[r][c]]=client;
                        } else {
                          clients[numclients]=client;
                          ready_index[r][c]=numclients;
                          numclients=(numclients+1);
                        }
			if (verbose > 1) std::cout << "Radar: " << client.radar << " Channel: " << client.channel <<
					" Beamnum: " << client.tbeam << " Status: " << msg.status << "\n";

			// Calculate frequency-dependent phase shift based on time-delay
			td = 10 * (16/2-client.tbeam);
			pd = fmod((td*freq*1e-9), 1.0) * 6.28;
			pdreal = cos(pd);
			pdimag = sin(pd);
			std::cout << "Time delay: " << td << " ns.  Phase delay: " << 360*pd/6.28 << " degrees.\n";
			std::cout << "Multiplication factor: " << pdreal << " + j*" << pdimag << std::endl;
			std::cout << "Transmit frequency: " << client.tfreq << std::endl;
			std::cout << "Trise: " << client.trise << std::endl;

                        index=client.current_pulseseq_index; 
                        if (index!=old_pulse_index[r][c]) {
                        //if (1==1) {
			  if (verbose > -1) std::cerr << "Need to unpack pulseseq " << r << " " << c << " " << index << "\n";
			  if (verbose > -1) std::cerr << "Pulseseq length: " << pulseseqs[r][c][index]->len << "\n";
			// unpack the timing sequence
			  seq_count[r][c]=0;
                          step=(int)((double)pulseseqs[r][c][index]->step/(double)STATE_TIME+0.5);
                            
                        //If DDS or RX Offset is negative pad the seq_buf iwith the maximum negative offset
                          offset_pad=(int)((double)MIN(dds_offset,rx_offset)/((double)STATE_TIME+0.5))-2;
			  if (verbose > -1) std::cout << "offset pad: " << offset_pad << "\n";	
                          for(i=0;i>offset_pad;i--) {
                            seq_buf[r][c][seq_count[r][c]]=0;
                            seq_count[r][c]++;
                          }
			  for(i=0;i<pulseseqs[r][c][index]->len;i++){
			    tempcode=_decodestate(r,c,(pulseseqs[r][c][index]->code)[i]);	
			    for( j=0;j<step*(pulseseqs[r][c][index]->rep)[i];j++){
			      seq_buf[r][c][seq_count[r][c]]=tempcode;
			      seq_count[r][c]++;
			    }
			  }
                        }
	                if (verbose > 1) std::cout << "Timing Card seq length: " << seq_count[r][c] << " state step: " <<
					STATE_TIME*1e-6 << " time: " << STATE_TIME*1e-6*seq_count[r][c] << "\n";

                        if (numclients >= maxclients) msg.status=-2;
		        if (verbose > 1) std::cout << "\nclient ready done\n";
                        numclients=numclients % maxclients;
                        old_pulse_index[r][c]=index;
                        rval=send_data(msgsock, &msg, sizeof(struct DriverMsg));
                        break; 

		      case TIMING_PRETRIGGER:
                        gettimeofday(&t0,NULL);
			if(verbose > 1 ) std::cout << "Setup Timing Card for next trigger\n";
                          msg.status=0;
		          if (verbose > 1) std::cout << "Max Seq length: " << max_seq_count <<
					 " Num clients: " << numclients << "\n";	
                          new_seq_id=-1;
			  new_beam=client.tbeam;
	                  for( i=0; i<numclients; i++) {
                            r=clients[i].radar-1;
                            c=clients[i].channel-1;
                            new_seq_id+=r*1000 +
                              c*100 +
                              clients[i].current_pulseseq_index+1;
                            if (verbose > 1) std::cout << i << " " << new_seq_id << " " << clients[i].current_pulseseq_index << "\n";
                          }
                          if (verbose > 1) std::cout << "Timing Driver: " << new_seq_id << " " << old_seq_id << "\n";

                          if (new_seq_id!=old_seq_id | new_beam != old_beam) { 
			    if (new_seq_id != old_seq_id) usrp_complex_buf.clear();
			    //Set the center frequency
			    for(size_t chan = 0; chan < usrp->get_tx_num_channels(); chan++) {
			            std::cout << boost::format("Setting TX Freq: %f MHz...") % (freq/1e6) << std::endl;
			            usrp->set_tx_freq(1000*client.tfreq, chan);
			            std::cout << boost::format("Actual TX Freq: %f MHz...") % (usrp->get_tx_freq(chan)/1e6) << std::endl << std::endl;
			    }
                          //if () { 
                            if (verbose > -1) std::cout << "Calculating Master sequence " << old_seq_id << " " << new_seq_id << "\n";
                        //if (1==1) { 
                            max_seq_count=0;
                            for (i=0;i<numclients;i++) {
                              r=clients[i].radar-1;
                              c=clients[i].channel-1;
                              if (seq_count[r][c]>=max_seq_count) max_seq_count=seq_count[r][c];
		              if (verbose > 1) std::cout << "Max Seq length: " << max_seq_count << "\n";
                              counter=0;
			      if (verbose > 1) std::cout << "Merging Client Seq " <<  i << 
						"into Master Seq " << r << " " << c << 
						"length: " << seq_count[r][c] << "\n";
                              if (i==0) {
				for (j=0;j<seq_count[r][c];j++) {
                                  master_buf[j]=seq_buf[r][c][j];
				  }
                                  counter++;
                              }
                              else {
				for (j=0;j<seq_count[r][c];j++) {
				  master_buf[j]|=seq_buf[r][c][j];
				  }
				  counter++;
			      }
                              
                              if (verbose > 1 ) std::cout << "Total Tr: " << counter << "\n";
			    }


			    if (new_seq_id != old_seq_id) {
			    	usrp_complex_buf.clear();
			    	usrp_complex_buf1.clear();
			    	usrp_real.resize(max_seq_count);
			    	usrp_imag.resize(max_seq_count);
			    	usrp_complex_buf.resize(max_seq_count);
			    	usrp_complex_buf1.resize(max_seq_count);
			    	usrp_buf.resize(max_seq_count);
			    	usrp_buf1.resize(max_seq_count);
			    	// Do a second-stage decode.. Put the tr and sync logic bits into the LSB's of
			    	// of the real and imaginary components
			    	// [TODO] this logic should probably be in the first decodestate() function
			    	for (j=0;j<max_seq_count;j++){
			    	    usrp_real[j] = (0x0001 & (master_buf[j] >> 16));
			    	    usrp_imag[j] = (0x0001 & (master_buf[j]));
			    	}

			    	// Build the complex-float-valued vector of baseband rf values
			    	// [TODO] this should be some kind of lookup table
			    	// i.e. if the tx bit is on, then look up the next value in the filtered-wave table
			    	// and for imaging configuration multiply by some phase shift.  
			    	// For imaging configuration, the output should
			    	// be a two-dimensional vector of values,i.e usrp_buf[16][master_buf_len]
			    	for (j=0;j<max_seq_count;j++){
			    	      if ((master_buf[j] & 0x00020000) == 0x00020000){
			    	    	//usrp_complex_buf[j] = filter_table[ifilter] * (std::complex<float>(1,0));
			    	    	//usrp_complex_buf1[j] = filter_table[ifilter] * std::complex<float>(pdreal,pdimag) * (std::complex<float>(1,0));
			    	    	usrp_complex_buf[j] = (std::complex<float>(1,0));
			    	    	//usrp_complex_buf1[j] = std::complex<float>(pdreal,pdimag) * (std::complex<float>(1,0));
			    	    	//if (ifilter >= filter_table_len) ifilter %= filter_table_len;
			    	      }
			    	}
			    	int filtrval;
			    	filter(usrp_complex_buf, filter_table);
			    }
			    for (j=0;j<max_seq_count;j++){
			          if (usrp_complex_buf[j] != std::complex<float>(0,0)){
			        	usrp_complex_buf1[j] = std::complex<float>(pdreal,pdimag) * usrp_complex_buf[j];
			          }
			    }

			    // Merge the logic bits with the baseband rf complex values
			    // The output needs to be sc16 type to preserve the bit information
			    int16_t tempreal, tempimag;
			    std::complex<float> chan0temp, chan1temp;
			    for (j=0;j<max_seq_count;j++){
			          chan0temp = usrp_complex_buf[j] * std::complex<float>(0.95*pow(2,14),0);
			          chan1temp = usrp_complex_buf1[j] * std::complex<float>(0.95*pow(2,14),0);

				  tempreal = usrp_real[j] | (((int16_t) chan0temp.real()) & 0xfffe);
				  tempimag = usrp_imag[j] | (((int16_t) chan0temp.imag()) & 0xfffe);
				  usrp_buf[j] = std::complex<int16_t>(tempreal, tempimag);

				  tempreal = (int16_t) chan1temp.real();
				  tempimag = (int16_t) chan1temp.imag();
				  usrp_buf1[j] = std::complex<int16_t>(tempreal, tempimag);
			    }

			    //std::cout << "sizeof usrp_buf:" << usrp_buf.size() << "\n";

			    // add the FIFO level bits
                            bad_transmit_times.length=0;
                            tr_event=0; 
                            scope_event=0; 
                            scope_start=-1;
                            dds_trigger=0;
                            rx_trigger=0;
	                    //if (verbose > 1) std::cout << "Fifo stamping Master Seq using FIFOLVL " << FIFOLVL << "\n";

			    for(i=0;i<max_seq_count;i++){
                              //if( ( i!=0 ) && ( i <= (max_seq_count-FIFOLVL) ) ) {
                                //if( ((i%FIFOLVL)==0) ) {
				//  for(j=0;j<FIFOWIDTH;j++){
                            	//	master_buf[i+j]|=0x80;
				//  }
                                //}
                              //}
                              if ((master_buf[i] & 0x02)==0x02) {
                                /* JDS: use tx as AM gate for mimic recv sample for external freq gen */
                                //if(tx_offset > 0) {
                                //  temp=tx_offset/STATE_TIME;
                                //  master_buf[i+temp]|= 0x04 ; 
                                //}
                                if (tr_event==0) { 
                                  if (verbose > 1 ) std::cout << "Master TR sample start: " << i << " " << master_buf[i] << "\n";
                                  bad_transmit_times.length++;
                                  if(bad_transmit_times.length > 0){ 
                                    if(bad_transmit_times.length < MAX_PULSES) { 
                                      (bad_transmit_times.start_usec)[bad_transmit_times.length-1]=i*STATE_TIME;
                                      (bad_transmit_times.duration_usec)[bad_transmit_times.length-1]=STATE_TIME;
                                    } else {
                                      std::cout << "Too many transmit pulses\n";
                                    } 
                                  }
                                } else {
                                    (bad_transmit_times.duration_usec)[bad_transmit_times.length-1]+=STATE_TIME;
                                }
                                tr_event=1;
                              } else {
                                if(tr_event==1) 
                                  if (verbose > 1 ) std::cout << "Master TR sample end: " << i << " " << master_buf[i] << "\n";
                                tr_event=0;
                              }
                              if ((master_buf[i] & 0x01)==0x01) {
                                if (scope_event==0) { 
                                  if (verbose > 1 ) std::cout << "Scope Sync sample start: " << i << " " << master_buf[i] << "\n";
                                  scope_start=i;
                                }
                                scope_event=1;
                              } else {
                                if (scope_event==1) 
                                  if (verbose > 1 ) std::cout << "Scope Sync sample end: " << i << " " << master_buf[i] << "\n";
                                scope_event=0;
                              }
                            }
                            if (scope_start>-1) { 
                              //dds_trigger=scope_start+(int)((double)dds_offset/((double)STATE_TIME+0.5));
                              //rx_trigger=scope_start+(int)((double)rx_offset/((double)STATE_TIME+0.5));
                              //if (verbose > 1 ) {
                              //  std::cout << "---> Scope Sync in Master " << max_seq_count << " at " << scope_start << "\n";
                              //  std::cout << "---> DDS Trigger in Master " << max_seq_count << " at " << dds_trigger << "\n"; 
                              //  std::cout << "---> Rx Trigger in Master " << max_seq_count << " at " << rx_trigger << "\n"; 
                              //} 
                            } else {
                              //if (verbose > 1 ) std::cout << "XXX> Scope Sync not in Master " << max_seq_count << "\n";
                              //dds_trigger=0;
                              //rx_trigger=0;
                            }
                            if((dds_trigger>=0) && (dds_trigger<max_seq_count)) {
                              //master_buf[dds_trigger]|=0x4000;                            
                            }
                            if((rx_trigger>=0) && (rx_trigger<max_seq_count)) {
                              //master_buf[rx_trigger]|=0x8000;                            
                            }
                          }
                        

	                  if (verbose > 1) std::cout << "seq length: " << max_seq_count << " state step: " <<
						STATE_TIME*1e-6 << " time: " << (STATE_TIME*1E-6*max_seq_count) << "\n";

			  if (verbose > 1) std::cout << "Max Seq Count: " << max_seq_count << "\n";
                          if (verbose > 1) std::cout << "END FIFO Stamps\n";

                        if (new_seq_id < 0 ) {
                          old_seq_id=-10;
                        }  else {
                          old_seq_id=new_seq_id;
                        }
                        new_seq_id=-1;
			old_beam = new_beam;

                        send_data(msgsock, &bad_transmit_times.length, sizeof(bad_transmit_times.length));
                        send_data(msgsock, bad_transmit_times.start_usec, 
                                  sizeof(unsigned int)*bad_transmit_times.length);
                        send_data(msgsock, bad_transmit_times.duration_usec, 
                                  sizeof(unsigned int)*bad_transmit_times.length);

			msg.status=0;
                        if (verbose > 1)  std::cout << "Ending Pretrigger Setup\n";
                        rval=send_data(msgsock, &msg, sizeof(struct DriverMsg));
                        gettimeofday(&t6,NULL);
                        elapsed=(t6.tv_sec-t0.tv_sec)*1E6;
                        elapsed+=(t6.tv_usec-t0.tv_usec);
                        if (verbose > 1) {
                          std::cout << "Timing Pretrigger Elapsed Microseconds: " << elapsed << "\n";
                        }
                        break; 

		      case TIMING_TRIGGER:

			if (verbose > 1 ) std::cout << "Setup Timing Card trigger\n";	
                        msg.status=0;
			if (verbose > 1) std::cout << "Read msg struct from tcp socket!\n";	

                        if(configured) {

			//std::cout << ("Here is the head of the master buffer:\n");
			//std::cout << "Length: " << MAX_TIME_SEQ_LEN << "\n";
			//for (i=0; i < max_seq_count; i++){
			//	if(master_buf[i] != 0) {
			//		std::cout << i << "\t";
			//		std::cout << usrp_buf[i] << " " << usrp_buf[i] << std::endl;
			//	}
			//};

			//setup the metadata flags
			uhd::tx_metadata_t md;
			md.start_of_burst = true;
			md.end_of_burst   = false;
			md.has_time_spec  = true;
			time0 = usrp->get_time_now(0);
			md.time_spec = uhd::time_spec_t(time0 + 0.001);
			
			tx_stream->send(buffs, max_seq_count, md);
        		//md.start_of_burst = false;
        		//md.has_time_spec = false;
			//md.end_of_burst = true;
			//tx_stream->send("", 0, md);
                        }

                        gettimeofday(&t1,NULL);
			if (verbose > 1 ) std::cout << " Trigger Time: " << t1.tv_sec << " " << t1.tv_usec << "\n";
			if (verbose > 1 ) std::cout << "End Timing Card trigger\n";
                        rval=send_data(msgsock, &msg, sizeof(struct DriverMsg));

                        break;

                      case TIMING_GPS_TRIGGER:
                        if (verbose > 1 ) std::cout << "Setup Timing Card GPS trigger\n";
                        msg.status=0;
                        if (verbose > 1) std::cout << "Read msg struct from tcp socket!\n";

                        if(configured) {
//#ifdef __QNX__
//                          //clear interrupt status
//                          temp=in32(mmap_io_ptr_dio+0x0c);
//                          temp|=0x04;
//                          out32(mmap_io_ptr_dio+0x0c, temp);
//                          writingFIFO=1;
//                          //enable outputs and wait for external trigger
//                          out32(mmap_io_ptr_dio+0x04,0x0161);
//#endif
                        }                   
                        if (verbose > 1 ) std::cout << "End Timing Card GPS trigger\n";
                        rval=send_data(msgsock, &msg, sizeof(struct DriverMsg));
                        break;
		      case TIMING_WAIT:
			if (verbose > 1 ) std::cout << "Timing Card: Wait\n";	
                        msg.status=0;
			if (verbose > 1) std::cout << "Read msg struct from tcp socket!\n";	
                        dead_flag=0;
                        if(configured) {
//#ifdef __QNX__
//			  if (verbose > 1 ) std::cout << "Timing Card: Wait : Inside configured\n";	
//                          delay_count=0;
//                          gettimeofday(&t0,NULL);
//	                  while(writingFIFO==1 && dead_flag==0) {
//                            gettimeofday(&t1,NULL);
//                            delay(1); //wait to finish writing FIFO
//                            delay_count++;
//                            if((t1.tv_sec-t0.tv_sec) > 1) dead_flag=1;
//                          }
//                          if (delay_count > 0) if (verbose > -1) std::cout << "writingFIFO wait " << delay_count << " ms\n";
//                          if(dead_flag==0) { 
//			    while( ( in32(mmap_io_ptr_dio+0x04) & 0x00001000 ) != 0x00001000) delay(1); //wait for FIFO empty 
//                          } else {
//                            msg.status+=-1;
//                            //printf("Wait timeout!!!!!! %d\n",dma_count);
//                          } 
//
//  			  //disable outputs, wait for trigger, terminations off
//	               	  out32(mmap_io_ptr_dio+0x04, 0x00000041); 
//
//                          if(xfercount<max_seq_count) {
//                            gettimeofday(&t0,NULL);
//                            msg.status+=-2;
//                            printf("Wait:  %8d %8d FIFO Underflow 0x%x Empty: 0x%x time :: sec: %8d usec:%8d\n",xfercount,max_seq_count,under_flag,empty_flag,t0.tv_sec,t0.tv_usec);
//                          }
//	                  out32(mmap_io_ptr_dio+0x04, 0x00000641); //clear fifo, and clear under-run status bit
//#endif
                        }
                        if (verbose > 1)  std::cout << "Ending Wait \n";
                        rval=send_data(msgsock, &msg, sizeof(struct DriverMsg));
			break;
		      case TIMING_POSTTRIGGER:
                        numclients=0;
                        for (r=0;r<MAX_RADARS;r++){
                          for (c=0;c<MAX_CHANNELS;c++){
                            ready_index[r][c]=-1;
                          }
                        }
                        rval=send_data(msgsock, &msg, sizeof(struct DriverMsg));
                        break;
		      default:
			if (verbose > -10) std::cerr << "BAD CODE: " << datacode << "\n"; //%c : %d\n",datacode,datacode);
			break;
		    }
		  }	
		} 
		if (verbose > 0 ) std::cerr << "Closing socket\n";
		close(msgsock);
	};

        return 1;
}
