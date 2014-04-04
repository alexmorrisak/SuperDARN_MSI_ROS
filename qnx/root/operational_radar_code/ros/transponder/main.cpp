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
#include <sys/mman.h>
#include <fcntl.h>
#include <exception>

//Added by Alex for usrp
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/utils/thread_priority.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/utils/static.hpp>
#include <uhd/exception.hpp>

#include <boost/thread.hpp>
#include <boost/format.hpp>

//Added by Alex for signal processing
#include <alex_custom.hpp>

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
#define MAX_INPUTS 2
#define MAX_CARDS 1
#define MAIN_INPUT 0
#define BACK_INPUT 1 
#define IMAGING 0 
#define MAX_SAMPLES 262144 
#define MIMO 1


dictionary *Site_INI;
int sock=0,msgsock=0;
int verbose=10;
int configured=1;
int		writingFIFO=0, dma_count=0, under_flag=0,empty_flag=0,IRQ, intid;
int		max_seq_count=0, xfercount=0, totransfer=0;
uintptr_t	mmap_io_ptr_dio;
unsigned int	virtual_addr[1], physical_addr[1];
struct sigevent interruptevent;
int tr_event=0, scope_event=0;

pthread_t int_thread;
void graceful_cleanup(int signum)
{
  char path[256];
  sprintf(path, "%s", "rostiming");
  close(msgsock);
  close(sock);
  std::cout << "Unlinking Unix Socket: " << path << "\n";
  unlink(path);

  exit(0);
}

const struct sigevent* isr_handler(void *arg, int id){
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
		return(NULL);
//#endif	
}

/***********************************************************************
 * transmit_worker function
 * A function to be used as a boost::thread_group thread for transmitting
 **********************************************************************/
void transmit_worker(
    uhd::tx_streamer::sptr tx_stream,
    std::vector<std::complex<short> *> pulse_sequences,
    int sequence_length,
    uhd::time_spec_t start_time
){
    //setup the metadata flags to send this single burst of data
    uhd::tx_metadata_t md;
    md.start_of_burst = true;
    md.end_of_burst   = true;
    md.has_time_spec  = true;
    md.time_spec = start_time;

    std::cout << "about to send..\n";
    //Now go for it!
    tx_stream->send(pulse_sequences,sequence_length, md);
}

float alpha = 0;
int rx_thread_status=0;
int main(){
    // DECLARE AND INITIALIZE ANY NECESSARY VARIABLES
        int     maxclients=MAX_RADARS*MAX_CHANNELS+1;
        struct  ControlPRM  clients[maxclients],client ;
        struct  TSGbuf *pulseseqs[MAX_RADARS][MAX_CHANNELS][MAX_SEQS];
        //struct  TSGprm *tsgparams[MAX_RADARS][MAX_CHANNELS][MAX_SEQS];
	unsigned int	*seq_buf[MAX_RADARS][MAX_CHANNELS];
        int seq_count[MAX_RADARS][MAX_CHANNELS];
        int old_pulse_index[MAX_RADARS][MAX_CHANNELS];
        int ready_index[MAX_RADARS][MAX_CHANNELS];
	unsigned int	*master_buf;
        int old_seq_id=-10;
        int new_seq_id=-1;
	int old_beam=-1;
	int new_beam=-1;
        struct TRTimes bad_transmit_times;

	unsigned int *shared_main_addresses[MAX_RADARS][MAX_CHANNELS][MAX_INPUTS]; //Only a single channel buffer for now.. (AFM 10 March 2014)
	unsigned int *shared_back_addresses[MAX_RADARS][MAX_CHANNELS][MAX_INPUTS]; 
	
	//uint64_t main_address=0,back_address=0;
	char shm_device[80];
	int shm_fd=0;

	// socket and message passing variables
	char	datacode;
	int	rval;
        fd_set rfds,efds;

	// counter and temporary variables
	int	i,j,r,c,buf,index,offset_pad;
	int	dds_offset,rx_offset,tx_offset;
        int     scope_start,dds_trigger,rx_trigger;
	int	step;
	int	tempcode;
        struct timeval t0,t6;
        unsigned long elapsed;

	// function specific message variables
        int     numclients=0;
        struct  DriverMsg msg;

	// timing related variables
	struct	timespec start, stop;

	// usrp-timing related variables
	uhd::time_spec_t get_data_t0;
	uhd::time_spec_t get_data_t1;

	// usrp-related variables
	typedef std::complex<int16_t> sc16;
	typedef std::complex<float> fc32;
	const int filter_table_len = 30;
	std::vector<fc32> filter_table(filter_table_len, std::complex<float>(0,0));
	std::vector<fc32> usrp_complex_buf;
	std::vector<fc32> usrp_complex_buf1;
	std::vector<sc16> tx_buf0;
	std::vector<sc16> tx_buf1;
	std::vector<sc16 *> tx_buffs;//(1, &usrp_short_buf.front());
	//buffs.push_back(&usrp_short_buf1.front());
	std::vector<sc16> rx_buf0;
	std::vector<sc16> rx_buf1;
	std::vector<sc16 *> rx_buffs;
	std::vector<int16_t> usrp_real;
	std::vector<int16_t> usrp_imag;
	float td, pd, pdreal, pdimag;

	//variables to be used as arguments to setup the usrp device(s)
	std::string args, subdev;
	args = "addr0=192.168.10.2, addr1=192.168.10.3";
	subdev = "A:A";

	clock_gettime(CLOCK_REALTIME, &start);

	//create a usrp device
	std::cout << std::endl;
	std::cout << boost::format("Creating the usrp device with: %s...") % args << std::endl;
	uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(args);

	//Lock mboard clocks
	UHD_ASSERT_THROW(usrp->get_num_mboards() == 2);
	
	uhd::time_spec_t time0 = usrp->get_time_now(0);
	uhd::time_spec_t time1 = usrp->get_time_now(1);
	std::cout << boost::format("Time 0: %d") % time0.get_real_secs() << std::endl;
	std::cout << boost::format("Time 1: %d") % time1.get_real_secs() << std::endl;
	
	if (MIMO){
	std::cout << "Using MIMO configuration.. The cable is plugged in right?!!\n";
	//make mboard 1 a slave over the MIMO Cable
	usrp->set_clock_source("mimo", 1);
	usrp->set_time_source("mimo", 1);
	
	//set time on the master (mboard 0)
	usrp->set_time_now(uhd::time_spec_t(0.0), 0);
	
	//sleep a bit while the slave locks its time to the master
	boost::this_thread::sleep(boost::posix_time::milliseconds(100));
	}
	if (!MIMO){
	std::cout << "Using external clock reference..\n";
	usrp->set_clock_source("external", 0);
	usrp->set_clock_source("external", 1);
	
	//set time on the master (mboard 0)
	usrp->set_time_next_pps(uhd::time_spec_t(0.0), 0);
	usrp->set_time_next_pps(uhd::time_spec_t(0.0), 1);
	
	//sleep a bit while the slave locks its time to the master
	boost::this_thread::sleep(boost::posix_time::milliseconds(1100));
	}

	//Specify daughterboard routing
	usrp->set_tx_subdev_spec(subdev);

	//Set Tx sample rate
	std::cout << boost::format("Setting TX Rate: %f kHz...") % (1e3/STATE_TIME) << std::endl;
	usrp->set_tx_rate(1e6 / STATE_TIME);
	std::cout << boost::format("Actual TX Rate: %f kHz...") % (usrp->get_tx_rate()/1e3) << std::endl << std::endl;
	//Set Rx sample rate
	std::cout << boost::format("Setting RX Rate: %f kHz...") % (1e3/STATE_TIME) << std::endl;
	usrp->set_rx_rate(1e6 / STATE_TIME);
	std::cout << boost::format("Actual RX Rate: %f kHz...") % (usrp->get_rx_rate()/1e3) << std::endl << std::endl;

	//create a transmit streamer
	//linearly map channels (index0 = channel0, index1 = channel1, ...)
	uhd::stream_args_t tx_stream_args("sc16", "sc16");
	for (size_t tx_chan = 0; tx_chan < usrp->get_tx_num_channels(); tx_chan++)
	    tx_stream_args.channels.push_back(tx_chan); //linear mapping
	uhd::tx_streamer::sptr tx_stream = usrp->get_tx_stream(tx_stream_args);

	//create a receive streamer
	//linearly map channels (index0 = channel0, index1 = channel1, ...)
	uhd::stream_args_t rx_stream_args("fc64", "sc16"); //use doubles for receiving
	for (size_t rx_chan = 0; rx_chan < usrp->get_rx_num_channels(); rx_chan++){
	    std::cout << "rx channel: " << rx_chan << std::endl;
	    rx_stream_args.channels.push_back(rx_chan); //linear mapping
	}
	uhd::rx_streamer::sptr rx_stream = usrp->get_rx_stream(rx_stream_args);

	std::cout << boost::format("Using Device: %s") % usrp->get_pp_string() << std::endl;

	for(r=0;r<MAX_RADARS;r++){
              for(c=0;c<MAX_CHANNELS;c++){
                  sprintf(shm_device,"/receiver_main_%d_%d_%d",r,c,0);
                  shm_unlink(shm_device);
                  shm_fd=shm_open(shm_device,O_RDWR|O_CREAT,S_IRUSR | S_IWUSR);
		  int rval = ftruncate(shm_fd,MAX_SAMPLES*4);
		  if (rval==-1) std::cerr << "ftruncate error!!\n";
                  shared_main_addresses[r][c][0]=(uint *)mmap(0,MAX_SAMPLES*4,PROT_READ|PROT_WRITE,MAP_SHARED,shm_fd,0);
                  close(shm_fd);
                  sprintf(shm_device,"/receiver_back_%d_%d_%d",r,c,0);
                  shm_unlink(shm_device);
                  shm_fd=shm_open(shm_device,O_RDWR|O_CREAT,S_IRUSR | S_IWUSR);
                  rval = ftruncate(shm_fd, MAX_SAMPLES*4);
		  if (rval==-1) std::cerr << "ftruncate error!!\n";
                  shared_back_addresses[r][c][0]=(uint *)mmap(0,MAX_SAMPLES*4,PROT_READ|PROT_WRITE,MAP_SHARED,shm_fd,0);
                  close(shm_fd);
                  for (i=0;i<MAX_SAMPLES;i++) {
                    shared_main_addresses[r][c][0][i]=i;
                    shared_back_addresses[r][c][0][i]=i;
                  }
              }
            }


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
        if (verbose > -1 ) std::cout << "Tx Offset: " << tx_offset <<
		"DDS Offset: " << dds_offset << " RX Offset: " << rx_offset << "\n";

        max_seq_count=0;
	if (verbose > 1) std::cout << "Zeroing arrays\n";
	for (r=0;r<MAX_RADARS;r++){
	  for (int c=0;c<MAX_CHANNELS;c++){
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

        master_buf=(unsigned int*) malloc(4*MAX_TIME_SEQ_LEN);

	clock_gettime(CLOCK_REALTIME, &stop);
	std::cout << "Elapsed setup time: " << 1e-9*(stop.tv_nsec-start.tv_nsec) << " sec\n";

    // OPEN TCP SOCKET AND START ACCEPTING CONNECTIONS 
	//printf("timing host port: %i \n", TIMING_HOST_PORT);
	sock=tcpsocket(TIMING_HOST_PORT);
	//bind(sock, (struct sockaddr *) TIMING_HOST_IP, sizeof(struct sockaddr_un));
	//printf("Done binding socket: %i \n", sock);
        //sock=server_unixsocket("/tmp/rostiming",0);
	listen(sock, 5);
	uhd::time_spec_t tstart;
	//std::vector<boost::thread *> receive_threads;
	boost::thread_group receive_threads;
	receive_threads.join_all();
	int32_t rx_status_flag;
	int32_t frame_offset;
	int32_t dma_buffer;
	int32_t nrx_samples;
	int32_t shm_memory;
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
                  //tv.tv_sec = 5;
                  //tv.tv_usec = 0;
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
			alpha = 32*(9.86/(2e-8*client.trise)) / (0.8328*usrp->get_rx_rate());
			std::cout << "alpha: " << alpha << std::endl;
			for (i=0; i<filter_table_len; i++){
				filter_table[i] = pow(alpha/3.14,0.5)*pow(2.7183, 
					-1*(alpha)*pow((((float)i-(filter_table_len-1)/2)/filter_table_len),2))/filter_table_len;
			}
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
			pd = fmod((td*client.tfreq*1e-6), 1.0) * 6.28;
			pdreal = cos(pd);
			pdimag = sin(pd);
			std::cout << "Time delay: " << td << " ns.  Phase delay: " << 360*pd/6.28 << " degrees.\n";
			std::cout << "Multiplication factor: " << pdreal << " + j*" << pdimag << std::endl;
			std::cout << "Transmit frequency: " << client.tfreq << std::endl;
			std::cout << "Trise: " << client.trise << std::endl;

                        index=client.current_pulseseq_index; 
                        if (index!=old_pulse_index[r][c]) {
                        //if (1==1) {
			  if (verbose > -1) std::cout << "Need to unpack pulseseq " << r << " " << c << " " << index << "\n";
			  if (verbose > -1) std::cout << "Pulseseq length: " << pulseseqs[r][c][index]->len << "\n";
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
	                  for( i=0; i<numclients; i++){
                            r=clients[i].radar-1;
                            c=clients[i].channel-1;
                            new_seq_id+=r*1000 +
                              c*100 +
                              clients[i].current_pulseseq_index+1;
                            if (verbose > 1) std::cout << i << " " << new_seq_id << " " << clients[i].current_pulseseq_index << "\n";
                          }
                          if (verbose > 1) std::cout << "Timing Driver: " << new_seq_id << " " << old_seq_id << "\n";

                          if ((new_seq_id!=old_seq_id) | (new_beam != old_beam)) { 
			    if (new_seq_id != old_seq_id){
				std::vector<fc32>().swap(usrp_complex_buf);
			    }
			    //Set the rx center frequency
			    for(size_t chan = 0; chan < usrp->get_tx_num_channels(); chan++) {
			            std::cout << boost::format("Setting RX Freq: %f MHz...") % (client.tfreq/1e3) << std::endl;
			            usrp->set_rx_freq(1000*client.tfreq, chan);
			            std::cout << boost::format("Actual RX Freq: %f MHz...") % (usrp->get_rx_freq(chan)/1e6) << std::endl << std::endl;
			    }
			    //Set the tx center frequency
			    for(size_t chan = 0; chan < usrp->get_tx_num_channels(); chan++) {
			            std::cout << boost::format("Setting TX Freq: %f MHz...") % (client.tfreq/1e3) << std::endl;
			            usrp->set_tx_freq(1000*client.tfreq, chan);
			            std::cout << boost::format("Actual TX Freq: %f MHz...") % (usrp->get_tx_freq(chan)/1e6) << std::endl << std::endl;
			    }
                          //if () { 
                            if (verbose > -1) std::cout << "Calculating Master sequence " << old_seq_id << " " << new_seq_id << "\n";
                        //if (1==1) { 
                            max_seq_count=0;
			      printf("okay here-1\n");
			      printf("numclients: %i\n",numclients);
                            for (i=0;i<numclients;i++) {
			
			      printf("okay here0\n");
                              r=clients[i].radar-1;
                              c=clients[i].channel-1;
			      printf("okay here\n");
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
			    	std::vector<fc32>().swap(usrp_complex_buf); // Used to free memory
			    	std::vector<fc32>().swap(usrp_complex_buf1);
			    	usrp_complex_buf.resize(max_seq_count);
			    	usrp_complex_buf1.resize(max_seq_count);

			    	std::vector<sc16>().swap(tx_buf0);
			    	std::vector<sc16>().swap(tx_buf1);
			    	tx_buf0.resize(max_seq_count);
			    	tx_buf1.resize(max_seq_count);
			    	//std::vector<sc16 *>().swap(buffs);
				tx_buffs.clear();
				tx_buffs.push_back(&tx_buf0.front());
				tx_buffs.push_back(&tx_buf1.front());

			    	std::vector<int16_t>().swap(usrp_real);
			    	std::vector<int16_t>().swap(usrp_imag);
			    	usrp_real.resize(max_seq_count);
			    	usrp_imag.resize(max_seq_count);
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
				//int txon_flag = 0;
			    	for (j=0;j<max_seq_count;j++){
			    	      if ((master_buf[j] & 0x00020000) == 0x00020000){
					//if (txon_flag == 0){
			    	    	usrp_complex_buf[j] = (std::complex<float>(1,0));
			    	      }
			    	      if ((master_buf[j] & 0x00040000) == 0x00040000){
				    //if (((master_buf[j] & 0x00020000) == 0x00000000) && (txon_flag==1)){
				    //    txon_flag=0;
			    	    	usrp_complex_buf[j] = (std::complex<float>(-1,0));
					std::cout << usrp_complex_buf[j] << std::endl;
				      }
			    	}
			    	_convolve(usrp_complex_buf, filter_table);
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
				  tx_buf0[j] = std::complex<int16_t>(tempreal, tempimag);

				  tempreal = (int16_t) chan1temp.real();
				  tempimag = (int16_t) chan1temp.imag();
				  tx_buf1[j] = std::complex<int16_t>(tempreal, tempimag);
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

                        if(configured) {
			  uhd::time_spec_t start_time = usrp->get_time_now() + 0.02;
			  tstart = usrp->get_time_now();

			  //First wait for any existing receiver threads..
			  std::cout << "Waiting on thread.." << std::endl;
			  receive_threads.join_all();
			  
		       	  //Adjust the number of samples to receive to account for sample rate conversion
			  int oversamplerate = (int) (200000 / (client.baseband_samplerate/2));
			  std::cout << "client baseband sample rate: " << client.baseband_samplerate << std::endl;;
			  //std::cout << "pulse width (us): " << .txpl << std::endl;;
			  std::cout << "Usrp sample rate: " << 200000 << std::endl;;
			  std::cout << "Oversample rate: " << oversamplerate << std::endl;;

			  std::cout << "Clearing Rx Buffer memory ..\n";
			  std::vector<sc16>().swap(rx_buf0); // Used to free memory
			  std::vector<sc16>().swap(rx_buf0);
			  rx_buf0.resize(client.number_of_samples);
			  rx_buf1.resize(client.number_of_samples); 
			  std::vector<sc16 *>().swap(rx_buffs);
			  rx_buffs.push_back(&rx_buf0.front());
			  rx_buffs.push_back(&rx_buf1.front());

			  //Start the receive stream thread
		       	  receive_threads.create_thread(boost::bind(recv_to_buffer,
			  	usrp,
			  	rx_stream,
			  	rx_buffs,
			  	client.number_of_samples,
			  	client.baseband_samplerate,
			  	start_time,
			  	&rx_thread_status));

			  uhd::time_spec_t now_time = usrp->get_time_now();
			  //call function to start tx stream simultaneously with rx stream
			  std::cout << "time: " << now_time.get_real_secs() <<
				" start time: " << start_time.get_real_secs() << std::endl;
		       	  transmit_worker(tx_stream, tx_buffs, max_seq_count, start_time);
                        }
                        rval=send_data(msgsock, &msg, sizeof(struct DriverMsg));

                        break;

                      case TIMING_GPS_TRIGGER:
                        if (verbose > 1 ) std::cout << "Setup Timing Card GPS trigger\n";
                        msg.status=0;
                        if (verbose > 1) std::cout << "Read msg struct from tcp socket!\n";
                        if (verbose > 1 ) std::cout << "End Timing Card GPS trigger\n";
                        rval=send_data(msgsock, &msg, sizeof(struct DriverMsg));
                        break;
		      case TIMING_WAIT:
			std::cout << "USRP_TCP_DRIVER_WAIT: Waiting on receiver thread.." << std::endl;
			receive_threads.join_all();
			std::cout << "Joined thread" << std::endl;

			if (verbose > 1){
			uhd::time_spec_t t1 = usrp->get_time_now();
			std::cout << "Rx samples elapsed milliseconds: " << 
				1e3*(t1.get_real_secs()-tstart.get_real_secs()) << std::endl;
			}
			if (verbose > 1 ) std::cout << "Timing Card: Wait\n";	
                        msg.status=0;
			if (verbose > 1) std::cout << "Read msg struct from tcp socket!\n";	
                        //dead_flag=0;
                        if (verbose > 1)  std::cout << "Ending Wait \n";
                        rval=send_data(msgsock, &msg, sizeof(struct DriverMsg));
			break;

		      case RECV_GET_DATA:
			if (rx_thread_status==-1){
				printf("Shit, bad status!!\n");
				rx_status_flag=-1;
			}
			if (rx_thread_status==0) printf("Status okay!!\n");
			std::cout << "Client asking for rx samples (Radar,Channel): " <<
				client.radar << " " << client.channel << std::endl;
			get_data_t0 = usrp->get_time_now();
			r = client.radar-1; c = client.channel-1;

			rx_status_flag = 0;
		        rval=recv_data(msgsock,&client,sizeof(struct ControlPRM));
                        rval=send_data(msgsock,&rx_status_flag, sizeof(int));

			if(rx_status_flag == 0){
			  std::cout << "Repackaging RX data.." << std::endl;
			  int32_t i_temp;
			  int32_t q_temp;
			  int32_t iq_temp;
			  for(int i=0; i < client.number_of_samples; i++){
			  	i_temp=(int32_t) rx_buf0[i].real();
			  	q_temp=(int32_t) rx_buf0[i].imag();
			  	iq_temp=(i_temp << 16) | (q_temp & 0x0000ffff); // Assuming i-phase component comes first ..?
				//std::cout << i << " " << i_temp << " " << q_temp << "\n";
			  	shared_main_addresses[r][c][0][i] = iq_temp;
			  	shared_back_addresses[r][c][0][i] = iq_temp;

			        //printf("Rx: %x\t",shared_main_addresses[r][c][0][i]);
			        //printf("I: %hi\t",((shared_main_addresses[r][c][0][i] >> 16) & 0x0000ffff));
			        //printf("Q: %hi\n",(shared_main_addresses[r][c][0][i] & 0x0000ffff));
			  }
			  
			  r=client.radar-1;
			  c=client.channel-1;
                          if(verbose > 1 ){
			  	std::cout << "Radar: " << client.radar << 
			         		"\tChannel: " << client.channel << std::endl;
			  	std::cout << "r: " << r << "\tc: " << c << std::endl;
			  }
			  shm_memory=1; // Flag used to indicate to client if shared memory (mmap()) is used. 1 for yes.
			  rval=send_data(msgsock,&shm_memory,sizeof(shm_memory));
			  frame_offset=0;  // The GC316 cards normally produce rx data w/ a header of length 2 samples. 0 for usrp.
			  rval=send_data(msgsock,&frame_offset,sizeof(frame_offset));
			  dma_buffer=0; // Flag used to indicate to client if DMA tranfer is used. 1 for yes.
			  rval=send_data(msgsock,&dma_buffer,sizeof(dma_buffer));
			  nrx_samples=client.number_of_samples;
			  std::cout << "Number of samples to receive: " << nrx_samples << std::endl;
			  rval=send_data(msgsock,&nrx_samples,sizeof(nrx_samples));
			  
			  std::cout << "About to send data to client" << std::endl;
			  if(IMAGING==0){
                          	if(verbose > 1 ) std::cout << "Sending shared addresses..: " << 
			  		shared_main_addresses[r][c][0] << "\t" << shared_back_addresses[r][c][0] << std::endl;
			  	
			  	rval=send_data(msgsock,&shared_main_addresses[r][c][0],sizeof(unsigned int));
			  	rval=send_data(msgsock,&shared_back_addresses[r][c][0],sizeof(unsigned int));
			  }
			  std::cout << "Send data to client successful" << std::endl;
			  msg.status = rx_status_flag;
			  rval=send_data(msgsock,&msg,sizeof(DriverMsg));
			}
			get_data_t1 = usrp->get_time_now();
			std::cout << "Ending RECV_GET_DATA. Elapsed time: " << 
				get_data_t1.get_real_secs() - get_data_t0.get_real_secs() << std::endl;
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
