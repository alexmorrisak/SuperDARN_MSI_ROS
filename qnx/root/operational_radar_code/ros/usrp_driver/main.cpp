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
#include <txrx_data.hpp>
#include <math.h>
#include <cuda_prog.h>

#ifdef __cplusplus
	extern "C" {
#endif
    //#include "cuda_prog.h"
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

#define TXRATE 2.5e6
#define TXFREQ 12e6
#define RXRATE 2.5e6
#define RXFREQ 12e6

#define MIMO 1
#define NUNITS 2

#define NRADARS 2

dictionary *Site_INI;
int sock=0,msgsock=0;
int verbose=-10;
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
  //unlink(path);

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


float alpha = 0;
int rx_thread_status=0;
int rx_clrfreq_rval=0;
int usable_bandwidth,N,start,end;
double search_bandwidth,unusable_sideband;
double *pwr=NULL,*pwr2=NULL;
FILE *clr_fd;
int main(){
    // DECLARE AND INITIALIZE ANY NECESSARY VARIABLES
    int     maxclients=MAX_RADARS*MAX_CHANNELS+1;
    struct  ControlPRM  clients[maxclients], client;
    struct  TSGbuf *pulseseqs[MAX_RADARS][MAX_CHANNELS][MAX_SEQS];
    struct  CLRFreqPRM clrfreq_parameters;
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
	int nave=0;
	int center=0;
	int usable_bandwidth=0;
    struct TRTimes bad_transmit_times;

	//unsigned int *shared_main_addresses[MAX_RADARS][MAX_CHANNELS][MAX_INPUTS]; //Only a single channel buffer for now.. (AFM 10 March 2014)
	uint32_t *shared_main_addresses[MAX_RADARS][MAX_CHANNELS][MAX_INPUTS]; //Only a single channel buffer for now.. (AFM 10 March 2014)
	//uint64_t *shared_main_addresses[MAX_RADARS][MAX_CHANNELS][MAX_INPUTS]; //Only a single channel buffer for now.. (AFM 10 March 2014)
	//unsigned int *shared_back_addresses[MAX_RADARS][MAX_CHANNELS][MAX_INPUTS]; 
	uint32_t *shared_back_addresses[MAX_RADARS][MAX_CHANNELS][MAX_INPUTS]; 
	//uint64_t *shared_back_addresses[MAX_RADARS][MAX_CHANNELS][MAX_INPUTS]; 
	
	//uint64_t main_address=0,back_address=0;
	char shm_device[80];
	int shm_fd=0;

	//socket and message passing variables
	char	datacode;
	int	rval;
        fd_set rfds,efds;

	//counter and temporary variables
	int	i,j,r,c,buf,index,offset_pad;
	int	dds_offset,rx_offset,tx_offset;
        int     scope_start,dds_trigger,rx_trigger;
	int	step;
	int	tempcode;
    struct timeval t0,t6;
    struct timeval t1,t2;
    unsigned long elapsed;

	// function-specific message variables
    int     numclients=0,nactiveclients=0;
    struct  DriverMsg msg;

	// timing related variables
	struct	timespec cpu_start, cpu_stop;
	struct	timespec tx0,tx1,tx2;
    float elapsed_t1;

	// usrp-timing related variables
	uhd::time_spec_t get_data_t0;
	uhd::time_spec_t get_data_t1;

	// usrp-data variables
	typedef std::complex<int16_t> sc16;
	typedef std::complex<float> fc32;
	std::vector<fc32> filter_taps;//filter taps for baseband filtering of tx signal
    std::vector<std::vector<fc32> > tx_bb_vecs;
	std::vector<sc16 *> tx_vec_ptrs;

	std::vector<sc16 *> rx_vec_ptrs;
    std::vector<std::complex<float>* > bb_vec_ptrs;

	int tx_osr;

    tx_data tx(NUNITS,NUNITS);
    rx_data rx(NUNITS,2*NUNITS,RXFREQ, RXRATE);

	// Swing buffered.
	unsigned int iseq=0;

    std::vector<float> time_delays;//,pd;

	//variables to be used as arguments to setup the usrp device(s)
	std::string args, txsubdev, rxsubdev;
	args = "addr0=192.168.10.2, addr1=192.168.10.3";
	if(NUNITS==1)
	args = "addr0=192.168.10.2";
	txsubdev = "A:A";
	rxsubdev = "A:A A:B";

	clock_gettime(CLOCK_REALTIME, &cpu_start);

	//create a usrp device
	std::cout << std::endl;
	std::cout << boost::format("Creating the usrp device with: %s...") % args << std::endl;
	uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(args);
    
	//Specify daughterboard routing
	usrp->set_tx_subdev_spec(txsubdev);
	usrp->set_rx_subdev_spec(rxsubdev);

    int nrx_antennas = usrp->get_rx_num_channels();
    std::cout << "nrx_antennas: " << nrx_antennas << std::endl;
    int ntx_antennas = usrp->get_tx_num_channels();
    std::cout << "ntx_antennas: " << ntx_antennas << std::endl;
    //ntx_antennas = 1;

	//Lock mboard clocks
	std::cout << usrp->get_num_mboards() << std::endl;
	UHD_ASSERT_THROW(usrp->get_num_mboards() == NUNITS);
	
	if (NUNITS==1){
		std::cout << "using single usrp unit..\n";
		usrp->set_clock_source("internal");
		usrp->set_time_now(uhd::time_spec_t(0.0));
	}
	else{
		if (MIMO==1){
			std::cout << "Using MIMO configuration.. The cable is plugged in right?!!\n";
			//make mboard 1 a slave over the MIMO Cable
			usrp->set_clock_source("mimo", 1);
			usrp->set_time_source("mimo", 1);
			
			//set time on the master (mboard 0)
			usrp->set_time_now(uhd::time_spec_t(0.0), 0);
			
			//sleep a bit while the slave locks its time to the master
			boost::this_thread::sleep(boost::posix_time::milliseconds(100));
		}
		if (MIMO==0){
			std::cout << "Using external clock reference..\n";
			usrp->set_clock_source("external", 0);
			usrp->set_clock_source("external", 1);
			
			//set time on the master (mboard 0)
			usrp->set_time_next_pps(uhd::time_spec_t(0.0), 0);
			usrp->set_time_next_pps(uhd::time_spec_t(0.0), 1);
			
			//sleep a bit while the slave locks its time to the master
			boost::this_thread::sleep(boost::posix_time::milliseconds(1100));
		}
	}


	//Set Tx and Rx sample rates
	std::cout << boost::format("Setting TX Rate: %f kHz...") % (1e3/STATE_TIME) << std::endl;
	float txrate=TXRATE;
	usrp->set_tx_rate(txrate);
    usrp->set_tx_freq(TXFREQ);
	std::cout << boost::format("Actual TX Rate: %f kHz...") % (usrp->get_tx_rate()/1e3) << std::endl << std::endl;
	txrate = usrp->get_tx_rate();
	//Set Rx sample rate
	float rxrate=RXRATE;
	std::cout << boost::format("Setting RX Rate: %f kHz...") % (rxrate) << std::endl;
	usrp->set_rx_rate(rxrate);
    usrp->set_rx_freq(RXFREQ);
	std::cout << boost::format("Actual RX Rate: %f kHz...") % (usrp->get_rx_rate()/1e3) << std::endl << std::endl;
	rxrate = usrp->get_rx_rate();

	//create a transmit streamer
	//linearly map channels (index0 = channel0, index1 = channel1, ...)
	uhd::stream_args_t tx_stream_args("sc16", "sc16");
	for (size_t tx_chan = 0; tx_chan < usrp->get_tx_num_channels(); tx_chan++)
	    tx_stream_args.channels.push_back(tx_chan); //linear mapping
	uhd::tx_streamer::sptr tx_stream = usrp->get_tx_stream(tx_stream_args);

	//create a receive streamer
	//linearly map channels (index0 = channel0, index1 = channel1, ...)
	uhd::stream_args_t rx_stream_args("sc16", "sc16"); //use int16_t for receiving
	for (size_t rx_chan = 0; rx_chan < usrp->get_rx_num_channels(); rx_chan++){
	    std::cout << "rx channel: " << rx_chan << std::endl;
	    rx_stream_args.channels.push_back(rx_chan); //linear mapping
	}
	uhd::rx_streamer::sptr rx_stream = usrp->get_rx_stream(rx_stream_args);

	std::cout << boost::format("Using Device: %s") % usrp->get_pp_string() << std::endl;

    //Create shared memory location(s) for baseband rx samples
    //This memory is shared with the radarshell process for transferring rx data
	for(r=0;r<MAX_RADARS;r++){
        for(c=0;c<MAX_CHANNELS;c++){
            sprintf(shm_device,"/receiver_main_%d_%d_%d",r,c,0);
            shm_unlink(shm_device);
            shm_fd=shm_open(shm_device,O_RDWR|O_CREAT,S_IRUSR | S_IWUSR);
		    int rval = ftruncate(shm_fd,MAX_SAMPLES*4);
		    if (rval==-1) std::cerr << "ftruncate error!!\n";
            //shared_main_addresses[r][c][0]=(uint *)mmap(0,MAX_SAMPLES*4,PROT_READ|PROT_WRITE,MAP_SHARED,shm_fd,0);
            shared_main_addresses[r][c][0]=(uint32_t *)mmap(0,MAX_SAMPLES*4,PROT_READ|PROT_WRITE,MAP_SHARED,shm_fd,0);
            //shared_main_addresses[r][c][0]=(uint64_t *)mmap(0,MAX_SAMPLES*4,PROT_READ|PROT_WRITE,MAP_SHARED,shm_fd,0);
            close(shm_fd);
            sprintf(shm_device,"/receiver_back_%d_%d_%d",r,c,0);
            shm_unlink(shm_device);
            shm_fd=shm_open(shm_device,O_RDWR|O_CREAT,S_IRUSR | S_IWUSR);
            rval = ftruncate(shm_fd, MAX_SAMPLES*4);
		    if (rval==-1) std::cerr << "ftruncate error!!\n";
            //shared_back_addresses[r][c][0]=(uint *)mmap(0,MAX_SAMPLES*4,PROT_READ|PROT_WRITE,MAP_SHARED,shm_fd,0);
            shared_back_addresses[r][c][0]=(uint32_t *)mmap(0,MAX_SAMPLES*4,PROT_READ|PROT_WRITE,MAP_SHARED,shm_fd,0);
            //shared_back_addresses[r][c][0]=(uint64_t *)mmap(0,MAX_SAMPLES*4,PROT_READ|PROT_WRITE,MAP_SHARED,shm_fd,0);
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
   //tx_offset=iniparser_getint(Site_INI,"timing:tx_offset",TX_OFFSET);
   //dds_offset=iniparser_getint(Site_INI,"timing:dds_trigger_offset",DDS_OFFSET);
   //rx_offset=iniparser_getint(Site_INI,"timing:rx_trigger_offset",RX_OFFSET);
   //if (verbose > 1 ) std::cout << "Tx Offset: " << tx_offset <<
   //"DDS Offset: " << dds_offset << " RX Offset: " << rx_offset << "\n";

    max_seq_count=0;
	if (verbose > 1) std::cout << "Zeroing arrays\n";
	for (r=0;r<MAX_RADARS;r++){
	    for (c=0;c<MAX_CHANNELS;c++){
	        if (verbose > 1) std::cout << r << " " << c << "\n";
            ready_index[r][c]=-1; 
            old_pulse_index[r][c]=-1; 
            seq_buf[r][c]=(unsigned int*) malloc(4*MAX_TIME_SEQ_LEN);
	        for (i=0;i<MAX_SEQS;i++) {
                pulseseqs[r][c][i]=NULL;
            }
        } 
    }
    bad_transmit_times.length=0;
    bad_transmit_times.start_usec=(uint32_t*) malloc(sizeof(unsigned int)*MAX_PULSES);
    bad_transmit_times.duration_usec=(uint32_t*) malloc(sizeof(unsigned int)*MAX_PULSES);

    master_buf=(unsigned int*) malloc(MAX_TIME_SEQ_LEN*sizeof(int32_t));

	clock_gettime(CLOCK_REALTIME, &cpu_stop);
	if(verbose>1)std::cout << "Elapsed setup time: " << 1e-9*(cpu_stop.tv_nsec-cpu_start.tv_nsec) << " sec\n";

    // OPEN TCP SOCKET AND START ACCEPTING CONNECTIONS 
	if (verbose > 1) printf("timing host port: %i \n", TIMING_HOST_PORT);
	sock=tcpsocket(TIMING_HOST_PORT);
	//bind(sock, (struct sockaddr *) TIMING_HOST_IP, sizeof(struct sockaddr_un));
	//printf("Done binding socket: %i \n", sock);
        //sock=server_unixsocket("/tmp/rostiming",0);
	listen(sock, 5);
	uhd::time_spec_t tstart;

    //Create thread groups
	boost::thread_group receive_threads;
	boost::thread_group rx_process_threads;
	boost::thread_group tx_threads;

	int32_t rx_status_flag;
	int32_t frame_offset;
	int32_t dma_buffer;
	int32_t nrx_samples;
	int32_t shm_memory;
	while(true){
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
            if (verbose>1) std::cout << "Socket data is ready to be read\n";
		    if (verbose > 1) std::cout << msgsock << " Recv Msg\n";
            rval=recv_data(msgsock,&msg,sizeof(struct DriverMsg));
            datacode=msg.type;
		    if (verbose > 1) std::cout << "\nmsg code is " <<  datacode << "\n";
		   switch( datacode ){
		      case TIMING_REGISTER_SEQ:
		        if (verbose > 0) std::cout << "\nRegister new sequence for usrp driver\n";
		        msg.status=0;
                rval=recv_data(msgsock,&client,sizeof(struct ControlPRM));
                r=client.radar-1; 
                c=client.channel-1; 

                rx.register_client(client);
                tx.register_client(client);

			    if (verbose > 1) std::cout << "Radar: " << client.radar <<
				    " Channel: " << client.channel << " Beamnum: " << client.tbeam <<
				    " Status: " << msg.status << "\n";
		        rval=recv_data(msgsock,&index,sizeof(index));
		        if (verbose > 1) std::cout << "Requested index: " << r << " " << c << " " << index << "\n";
		        if (verbose > 1) std::cout << "Attempting Free on pulseseq: " << pulseseqs[r][c][index];
                if (pulseseqs[r][c][index]!=NULL) {
                    std::cout << "Doing this..\n";
                    if (pulseseqs[r][c][index]->rep!=NULL){
				        free(pulseseqs[r][c][index]->rep);
                        pulseseqs[r][c][index]->rep=NULL;
                    }
                    if (pulseseqs[r][c][index]->code!=NULL){
                        free(pulseseqs[r][c][index]->code);
                        pulseseqs[r][c][index]->code=NULL;
                    }
                    free(pulseseqs[r][c][index]);
                    pulseseqs[r][c][index]=NULL;
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
		        rval=recv_data(msgsock,&client,sizeof(struct ControlPRM));
		        if (verbose > 0) printf("A client is done. Radar: %i, Channel: %i\n", client.radar-1, client.channel-1);
                r=client.radar-1;
                c=client.channel-1;
                tx.unregister_client(r,c);
                rx.unregister_client(r,c);
                msg.status=0;
                rval=send_data(msgsock, &msg, sizeof(struct DriverMsg));
                old_seq_id=-10;
                new_seq_id=-1;
                break;

		      case TIMING_CtrlProg_READY:
                //numclients = 0;
		        if (verbose > 1) printf("\nAsking to set up timing info for client that is ready %i\n",numclients);
                msg.status=0;
		        rval=recv_data(msgsock,&client,sizeof(struct ControlPRM));
                r=client.radar-1; 
                c=client.channel-1; 

                std::cout << "baseband samplerate: " << client.baseband_samplerate << std::endl;
                std::cout << "baseband samplerate: " << client.number_of_samples << std::endl;
                rx.ready_client(
                    client.radar-1,
                    client.channel-1,
                    client.number_of_samples, 
                    client.rfreq, 
                    client.baseband_samplerate);

                
                if ((ready_index[r][c]>=0) && (ready_index[r][c] <maxclients) ) {
                    clients[ready_index[r][c]]=client;
                } 
                else {
                    //std::cout << "Shoot! There should only be one client!!\n";
                    clients[numclients]=client;
                    ready_index[r][c]=numclients;
                    numclients=(numclients+1);
                }

			    if (verbose > 1) std::cout << "Radar: " << client.radar << " Channel: " << client.channel <<
					" Beamnum: " << client.tbeam << " Status: " << msg.status << "\n";

			    // Calculate time delay for beamforming
                //tx_freqs.clear();
                //tx_freqs.push_back(1e3*client.tfreq);
                //tx_freqs[0] = (1000*client.tfreq);
                //for (size_t i=0; i<tx_freqs.size(); i++)
                //    std::cout << "tx_freqs: " << tx_freqs[i] << std::endl;
                //client_freqs.resize(1);
                //client_freqs.resize(1);
                //client_freqs[0]= (1000*client.tfreq-RXFREQ);
                //std::cout << "rx_freqs: " << client_freqs[0] << std::endl;
                time_delays.resize(tx.get_num_clients());
                for (int i=0; i< tx.get_num_clients(); i++)
			        time_delays[i] = 10 * (16/2-client.tbeam); // 10 ns per antenna per beam

                index=client.current_pulseseq_index; 

                if (index!=old_pulse_index[r][c]){
			        if (verbose > -1) std::cout << "Need to unpack pulseseq " << r << " " << c << " " << index << "\n";
			        if (verbose > -1) std::cout << "Pulseseq length: " << pulseseqs[r][c][index]->len << "\n";
                
			    // unpack the timing sequence
			        seq_count[r][c]=0;
                    step=(int)((double)pulseseqs[r][c][index]->step/(double)STATE_TIME+0.5);
                            
                    //If DDS or RX Offset is negative pad the seq_buf iwith the maximum negative offset
                    offset_pad=(int)((double)MIN(dds_offset,rx_offset)/((double)STATE_TIME+0.5))-2;
			        if (verbose > -1) std::cout << "offset pad: " << offset_pad << "\n";	
                    for(int i=0;i>offset_pad;i--) {
                      seq_buf[r][c][seq_count[r][c]]=0;
                      seq_count[r][c]++;
                    }
			        for(int i=0;i<pulseseqs[r][c][index]->len;i++){
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
		        if (verbose > 1) std::cout << "\nclient ready\n";
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
	            for(int i=0; i<numclients; i++){
                    r=clients[i].radar-1;
                    c=clients[i].channel-1;
                    new_seq_id += r*1000 + c*100 + clients[i].current_pulseseq_index+1;
                    if (verbose > 1) std::cout << i << " " <<
                        new_seq_id << " " << clients[i].current_pulseseq_index << "\n";
                }
                if (verbose > 1) std::cout << "Timing Driver: " << new_seq_id << " " << old_seq_id << "\n";
                if ((new_seq_id!=old_seq_id) | (new_beam != old_beam)) // A new integration period is happening so set iseq to zero
			        iseq=0;
			    if (new_seq_id!=old_seq_id){ // Calculate new baseband pulse sequence if needed.  Otherwise just use the last pulse sequence
			        //Set the rx center frequency
			        for(int chan = 0; chan < nrx_antennas; chan++) {
			            usrp->set_rx_freq(RXFREQ,chan);
			            if(verbose>1)
				            std::cout << boost::format("RX freq set to: %f MHz...") % 
				    	    (usrp->get_rx_freq(chan)/1e6) << 
				    	    std::endl << std::endl;
			        }
			        //Set the tx center frequency
			        for(int chan = 0; chan < ntx_antennas; chan++) {
			            usrp->set_tx_freq(TXFREQ, chan);
			            if(verbose>1)
					         std::cout << boost::format("TX freq set to: %f MHz...") % 
							    (usrp->get_tx_freq(chan)/1e6) << 
							    std::endl << std::endl;
			        }
                    if (verbose > -1) std::cout << "Calculating Master sequence " << old_seq_id << " " << new_seq_id << "\n";
                    max_seq_count=0;
			        printf("numclients: %i\n",numclients);

				    tx_osr = round(txrate * ((float)STATE_TIME*1e-6));

                    //if (verbose > -1){
				    //    std::cout << "tx rate: " << usrp->get_tx_rate() << "\n";
				    //    std::cout << "bb rate: " << 1/(STATE_TIME*1e-6) << "\n";
				    //    std::cout << "creating tx vecs.\n osr: " << tx_osr << "\n";
                    //}

					//tx_rf_vecs.push_back(std::vector<sc16>(tx_osr*max_seq_count,0));

                    tx_bb_vecs.resize(tx.get_num_radars());
                    //for (int iclient=0; iclient<numclients; iclient++) {
                    if (numclients > tx.get_num_radars()){
                        numclients = tx.get_num_radars();
                    }
                    //for (int iradar=0; iradar<tx.get_num_radars(); iradar++) {
                    for (int iradar=0; iradar<1; iradar++) {
                        //int iclient = iradar;
                        //if (iradar+1 > numclients) iclient = numclients-1;
                        //tx_bb_vecs[i].resize(ntx_antennas);
                        //tx_rf_vecs[i].resize(ntx_antennas);
                        //for (int iant=0; iant<ntx_antennas; iant++){
                        //    tx_rf_vecs[i][iant].resize(tx_osr*seq_count[r][c]);
                        //    //std::cout << "tx rf vec size: " << tx_rf_vecs[0][i].size() << std::endl;
                        //}
                        int iclient=0;
                        r=clients[iclient].radar-1;
                        c=clients[iclient].channel-1;
                        if (seq_count[r][c]>=max_seq_count) max_seq_count=seq_count[r][c];
		        	    if (verbose > 1) std::cout << "Max Seq length: " << max_seq_count << "\n";
                        counter=0;
			            if (verbose > 1) std::cout << "Merging Client Seq " <<  iclient << 
			        		"into Master Seq " << r << " " << c << 
			        		"length: " << seq_count[r][c] << "\n";
                        tx_bb_vecs[iclient].clear();
                        tx_bb_vecs[iclient].resize(max_seq_count,0);
                        std::cout << "tx bb vecs size: " << tx_bb_vecs[iclient].size() << std::endl;
                        if (iclient==0) {
                            printf("Initializing master buffer\n");
			                for (j=0;j<seq_count[r][c];j++) {
			                    if ((seq_buf[r][c][j] & 0x00020000) == 0x00020000){
                                    tx_bb_vecs[iclient][j] = std::complex<float>(1,0);
                                }
			                    if ((seq_buf[r][c][j] & 0x00040000) == 0x00040000){
			                      tx_bb_vecs[iclient][j] = (std::complex<float>(-1,0));
				                }
                       	        master_buf[j]=seq_buf[r][c][j];
			                }
                            //counter++;
                        }
                        else {
                            printf("Appending master buffer\n");
			                for (j=0;j<seq_count[r][c];j++) {
			                    if ((seq_buf[r][c][j] & 0x00020000) == 0x00020000){
                                    tx_bb_vecs[iclient][j] = std::complex<float>(1,0);
                                }
			                    if ((seq_buf[r][c][j] & 0x00040000) == 0x00040000){
			                      tx_bb_vecs[iclient][j] = (std::complex<float>(-1,0));
                                }
			                    master_buf[j]|=seq_buf[r][c][j];
                            }
			                //counter++;
                        }
                    }
                        	  
                    if (verbose > 1 ) std::cout << "Total Tr: " << counter << "\n";

                    bad_transmit_times.length=0;
                    tr_event=0; 
                    scope_event=0; 
                    scope_start=-1;
                    dds_trigger=0;
                    rx_trigger=0;

			        for(int i=0;i<max_seq_count;i++){
                        if ((master_buf[i] & 0x00010000)==0x00010000) {
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
                    
	                  if (verbose > 1) std::cout << "seq length: " << max_seq_count << " state step: " <<
			    		STATE_TIME*1e-6 << " time: " << (STATE_TIME*1E-6*max_seq_count) << "\n";

			          if (verbose > 1) std::cout << "Max Seq Count: " << max_seq_count << "\n";
                      if (verbose > 1) std::cout << "END FIFO Stamps\n";

                    //std::cout << client.trise << std::endl;
                    /*Calculate taps for Gaussian filter*/
			        //alpha = 32*(9.86/(2e-8*client.trise)) / (0.8328*usrp->get_rx_rate());
			        //std::cout << "alpha: " << alpha << std::endl;
			        //for (i=0; i<filter_table_len; i++){
			        //	filter_table[i] = pow(alpha/3.14,0.5)*pow(M_E, 
			        //		-1*(alpha)*pow((((float)i-(filter_table_len-1)/2)/filter_table_len),2))/filter_table_len;
			        //}
                    //for (int iclient=0; iclient<numclients; iclient++){
                    for (int iclient=0; iclient<1; iclient++){
                        filter_taps.resize(50e3/clients[iclient].trise,
                            std::complex<float>(clients[iclient].trise/50.e3/2./(float)tx.get_num_clients(),0));
                        std::cout << clients[iclient].trise << std::endl;
                        std::cout << filter_taps.size() << std::endl;
                        std::cout << "signal length: " << tx_bb_vecs[iclient].size() << std::endl;
                        for (int j=0; j<tx_bb_vecs[iclient].size(); j++){
                            if (tx_bb_vecs[iclient][j] != std::complex<float>(0,0) && j%10 == 0) 
                                std::cout << iclient << " " << j << " " << tx_bb_vecs[iclient][j] << std::endl;
                        }
                        std::cout << filter_taps.size() << std::endl;
			            _convolve(&tx_bb_vecs[iclient].front(), 
                            tx_bb_vecs[iclient].size(), 
                            &filter_taps.front(),
                            filter_taps.size());
                    }
                    // Add the same pulse sequence for each client.  Not ideal, but works
                    // if you assume every client's got the same pulse sequence
                    for (int iclient=0; iclient<numclients; iclient++){ 
                        tx.add_pulse_seq(
                            clients[iclient].radar-1,
                            clients[iclient].channel-1, 
                            (float) 1e3*clients[iclient].tfreq, 
                            tx_bb_vecs[0]);
                    }
			    }

			    if (new_beam != old_beam) { //Re-calculate the RF sample vectors for new beam direction.  BB samples are the same.
			    	for(size_t chan = 0; chan < usrp->get_rx_num_channels(); chan++) {
			    	        usrp->set_rx_rate(rxrate, chan);
			                usrp->set_rx_freq(RXFREQ, chan);
			    	        if(verbose>1)
						std::cout << boost::format("RX rate set to: %f MHz...") % 
								(usrp->get_rx_rate(chan)/1e6) << std::endl;
						std::cout << boost::format("RX freq set to: %f MHz...") % 
								(usrp->get_rx_freq(chan)/1e6) << std::endl;
			    	}
			    	for(size_t chan=0; chan<ntx_antennas; chan++) {
			    	        usrp->set_tx_rate(txrate, chan);
			                usrp->set_tx_freq(TXFREQ, chan);
			    	        if(verbose>1)
						std::cout << boost::format("TX rate set to: %f MHz...") % 
								(usrp->get_rx_rate(chan)/1e6) << std::endl;
						std::cout << boost::format("TX freq set to: %f MHz...") % 
								(usrp->get_tx_freq(chan)/1e6) << std::endl;
			    	}

                    rx_process_threads.join_all(); //Make sure rx processing is done so that we have exclusive access to the GPU

                    clock_gettime(CLOCK_MONOTONIC, &tx0);
			        //tx_bb_vecs.clear();
                    //tx_bb_vecs.resize(ntx_antennas);
			        //tx_rf_vecs.clear();
					//tx_rf_vecs.resize(ntx_antennas);
                    //tx_rf_vec_ptrs.resize(ntx_antennas);
                    //for (int iant=0; iant<ntx_antennas; iant++){
                    //    tx_rf_vecs[iant].resize(tx_osr*max_seq_count);
                    //    //tx_rf_vec_ptrs[iant] = &tx_rf_vecs[iant].front();
                    //}


                    clock_gettime(CLOCK_MONOTONIC, &tx0);
                    for (int iradar=0; iradar<NRADARS; iradar++){ //Deal with each antenna channel for each radar
                        /*This handles the mapping from client number to antenna number(s).
                         * Make adjustments based on single-site vs. dual-site radars, IP
                         * address mapping, etc. */

                        if (numclients == 1){
                            for (int i=1; i<NRADARS; i++){
                                tx_bb_vecs[i].clear();
                                tx_bb_vecs[i].resize(tx_bb_vecs[0].size(), 0);
                                //tx_freqs.push_back(0);
                            }
                        }
                        //if (tx.get_num_clients(iradar) != 0){
                        //    float* bb_ptr = tx.get_bb_vec_ptr(iradar);
                        //    for (int i=0; i<tx.get_num_bb_samples(); i+=10){
                        //        std::cout << i << " " << bb_ptr[i] << std::endl;
                        //    }
                        //}

                        
                        tx.set_rf_vec_size(tx_osr*max_seq_count);
                        std::cout << "num bb samps: " << tx.get_num_bb_samples() << std::endl;
                        std::cout << "num rf samps: " << tx.get_num_rf_samples() << std::endl;
                        std::cout << "rf vec ptrs: " << tx.get_rf_vec_ptrs(iradar) << std::endl;
                        std::cout << "num rf samps: " << tx.get_num_rf_samples() << std::endl;
                        for (int i=0; i<tx.get_num_clients(iradar); i++){
                            std::cout << "tx freqs: " << tx.get_freqs(iradar)[i] << std::endl;
                        }
                        //for (size_t i=0; i<tx.get_num_rf_samples(); i++){
                        //    int16_t** rf_ptr = tx.get_rf_vec_ptrs(iradar);
                        //    //std::cout << rf_ptr << std::endl;
                        //    //std::cout << i << " " << rf_ptr[0] << std::endl;
                        //}
                        std::cout << "num clients: " << tx.get_num_clients(iradar) << std::endl;
                        //std::cout << "tx freq: " << tx_freqs[iradar] << std::endl;
                        if (tx.get_num_clients(iradar) == 0){
                            std::cout << "zeroing rf vector\n";
                            tx.zero_rf_vec(iradar);
                            continue;
                        }
                        std::cout << "rf vec ptrs: " << tx.get_rf_vec_ptrs(iradar) << std::endl;
                        //std::cout << (tx.get_rf_vec_ptrs(iradar))[0][0] << std::endl;
                        std::cout << "about to enter tx_process_gpu\n";
				        tx_process_gpu(
                            tx.get_bb_vec_ptr(iradar),
                            tx.get_rf_vec_ptrs(iradar),
				        	tx.get_num_bb_samples(),
				        	tx.get_num_rf_samples(),
				        	usrp->get_tx_freq(),
                            txrate,
                            tx.get_freqs(iradar), // List of center frequencies for this radar
				        	&time_delays.front(), // Vector of beam-forming-related time delays
                            tx.get_num_clients(iradar), // Number of channels for this radar [TODO] Numclients handles nradars but not nchannels yet
                            tx.get_num_ants_per_radar()); //number of antennas per radar
                    }

                    clock_gettime(CLOCK_MONOTONIC, &tx1);
                    elapsed_t1 = (1e9*tx1.tv_sec+tx1.tv_nsec) - (1e9*tx0.tv_sec+tx0.tv_nsec);
                    printf("\n\ntx mix upsample elapsed time: %.2f ms\n", elapsed_t1/1e6);
			        //tx_vec_ptrs.resize(ntx_antennas);
                    std::cout << "num clients: " << numclients << std::endl;
                    //std::cout << "txrfvecs size: " << tx_rf_vecs.size() << std::endl;
                    //for (size_t iant=0; iant<ntx_antennas; iant++){
                    //    tx_vec_ptrs[iant] = &tx_rf_vecs[iant].front();
                    //}
                        
			        //for(unsigned int iant=0;iant<ntx_antennas;iant++){
			        //    for(unsigned int iclient=0;iclient<numclients;iclient++){
                    //        std::cout << "index: " << iclient << " " << iclient << std::endl;
			    	//        tx_vec_ptrs[iclient]=&tx_rf_vecs[iclient][0].front();
                    //        std::cout << "tx_vec_ptr size: " <<  tx_vec_ptrs.size() << std::endl;
                    //        std::cout << "tx_vec_ptr: " <<  tx_vec_ptrs[iclient] << std::endl;
                    //    }
			        //    for(unsigned int iclient=numclients;iclient<ntx_antennas;iclient++){
                    //        std::cout << "index: " << iclient << " " << iclient%2 << std::endl;
                    //        std::cout << "tx_rf_vecs size: " <<  tx_rf_vecs.size() << std::endl;
                    //        std::cout << "iclient mod 2: " <<  iclient%2 << std::endl;
                    //        std::cout << "tx_vec_ptr: " <<  tx_vec_ptrs[iclient] << std::endl;
			    	//        tx_vec_ptrs[iclient]=&tx_rf_vecs[iclient%2][0].front();
                    //    }
                    //}
			    }

                if (new_seq_id < 0 ) {
                  old_seq_id=-10;
                }
                else {
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

                gettimeofday(&t0,NULL);
			    if (verbose > 1 ) std::cout << "Setup for trigger\n";	
			    if (verbose>1) std::cout << std::endl;
                msg.status=0;

                if(configured) {
			        //First wait for any existing receiver threads..
			        if (verbose > 1) std::cout << "Waiting on thread.." << std::endl;
			        receive_threads.join_all();
			  
		       	    //Adjust the number of samples to receive to account for sample rate conversion
			        if (verbose > 1){
			  	        std::cout << "client baseband sample rate: " << client.baseband_samplerate << std::endl;
			  	        std::cout << "nsamples: " << rx.get_num_rf_samples() << std::endl;
			  	        std::cout << "Usrp sample rate: " << rxrate << std::endl;
			        }

			        //Start the receive stream thread
			        if(verbose>1) std::cout << "About to start rx thread..\n";
			        rx_thread_status=0;
			        gettimeofday(&t0,NULL);
			        tstart = usrp->get_time_now();
	
                    rx.set_rf_vec_ptrs(&rx_vec_ptrs);
                    tx.set_rf_vec_ptrs(&tx_vec_ptrs);

			        uhd::time_spec_t start_time = usrp->get_time_now() + 0.02;
                    gettimeofday(&t1,NULL);
		            receive_threads.create_thread(boost::bind(recv_and_hold,
                        bad_transmit_times.start_usec,
                        bad_transmit_times.duration_usec,
                        bad_transmit_times.length,
			         	usrp,
			         	rx_stream,
			         	rx_vec_ptrs,
			         	rx.get_num_rf_samples()/2,
			         	start_time,
			         	&rx_thread_status));

			         //call function to start tx stream simultaneously with rx stream
		             tx_threads.create_thread(boost::bind(transmit_worker,
                        tx_stream,
                        tx_vec_ptrs,
                        tx.get_num_rf_samples(),
                        start_time));
                }
                tx_threads.join_all();
                //receive_threads.join_all();

                rval=send_data(msgsock, &msg, sizeof(struct DriverMsg));

                gettimeofday(&t6,NULL);
                elapsed=(t6.tv_sec-t0.tv_sec)*1E6;
                elapsed+=(t6.tv_usec-t0.tv_usec);
                if (verbose > 1) {
                  std::cout << "Timing Trigger Elapsed Microseconds: " << elapsed << "\n";
                }

                break;

            case TIMING_GPS_TRIGGER:
                if (verbose > 1 ) std::cout << "Setup Timing Card GPS trigger\n";
                msg.status=0;
                if (verbose > 1) std::cout << "Read msg struct from tcp socket!\n";
                if (verbose > 1 ) std::cout << "End Timing Card GPS trigger\n";
                rval=send_data(msgsock, &msg, sizeof(struct DriverMsg));
                break;

		    case TIMING_WAIT:
			    if(verbose > 1) std::cout << "USRP_TCP_DRIVER_WAIT: Nothing happens here.." << std::endl;
                msg.status=0;

			    if (verbose > 1) std::cout << "Read msg struct from tcp socket!\n";	
                //dead_flag=0;
                
                if (verbose > 1)  std::cout << "Ending Wait \n\n";
                rval=send_data(msgsock, &msg, sizeof(struct DriverMsg));

			    break;

            case TIMING_POSTTRIGGER:
                //receive_threads.join_all();
                nactiveclients=numclients;
                numclients=0;
                std::cout << "Post trigger.  Un-readying all clients\n\n";
                for (r=0;r<MAX_RADARS;r++){
                    for (c=0;c<MAX_CHANNELS;c++){
                        ready_index[r][c]=-1;
                    }
                }
                rval=send_data(msgsock, &msg, sizeof(struct DriverMsg));
                break;

            case RECV_GET_DATA:
			    if (verbose>1) std::cout << "RECV_GET_DATA: Waiting on all threads.." << std::endl;

                //tx_threads.join_all();
			    //receive_threads.join_all();
			    rx_process_threads.join_all();
                gettimeofday(&t0,NULL);

			    gettimeofday(&t6,NULL);
                elapsed=(t6.tv_sec-t1.tv_sec)*1E6;
                elapsed+=(t6.tv_usec-t1.tv_usec);
			    if (verbose>1)std::cout << "Rx thread elapsed: " << 
			    	elapsed << " usec" << std::endl;
                elapsed=(t6.tv_sec-t0.tv_sec)*1E6;

			    rx_status_flag=-1;
			    if (rx_thread_status==-1){
			    	std::cerr << "Error, bad status from Rx thread!!\n";
			    	rx_status_flag=-1;
			    }
			    else {
                    rx_status_flag=0;
			    	if (verbose>1) printf("Status okay!!\n");
			    }

		        rval=recv_data(msgsock,&client,sizeof(struct ControlPRM));
                rval=send_data(msgsock,&rx_status_flag, sizeof(int));

			    if (verbose>1)std::cout << "Client asking for rx samples (Radar,Channel): " <<
			    	client.radar << " " << client.channel << std::endl;
			    get_data_t0 = usrp->get_time_now();
			    r = client.radar-1; c = client.channel-1;
	
                std::cout << "numclients: " << rx.get_num_clients() << std::endl;

                std::cout << r << " " << c << std::endl;
                std::cout << rx.get_rf_dptr(r) << std::endl;
                std::cout << rx.get_bb_dptr(r) << std::endl;
                std::cout << rx.get_num_rf_samples() << std::endl;
                std::cout << rx.get_num_bb_samples() << std::endl;
                std::cout << rx.get_num_clients(r) << std::endl;
                std::cout << rx.get_num_ants_per_radar() << std::endl;
                std::cout << (float) rxrate << std::endl;
                std::cout << (float) client.baseband_samplerate << std::endl;
                std::cout << rx.get_freqs(r) << std::endl;
                if (verbose) std::cout << "about to enter rx_process_gpu()\n";
			    rx_process_threads.create_thread(boost::bind(rx_process_gpu,
                    rx.get_rf_dptr(r),
                    rx.get_bb_dptr(r),
                    rx.get_num_rf_samples(),
                    rx.get_num_bb_samples(),
			    	rx.get_num_clients(r),
			    	rx.get_num_ants_per_radar(),
			    	(float) rxrate,
			    	(float) client.baseband_samplerate,
                    rx.get_freqs(r)));
                

                //rx_process_threads.join_all();
                printf("iseq: %i\n", iseq);
                std::cout << "set_bb_vec_ptrs: " << r << " " << c << std::endl;
                rx.set_bb_vec_ptrs(r,c, &bb_vec_ptrs);
			    if(rx_status_flag == 0){
			      if(verbose>1)std::cout << "Repackaging RX data.." << std::endl;
                  std::cout << "nactiveclients: " << nactiveclients << std::endl;

                  gettimeofday(&t1,NULL);
			      for(int i=0;i<client.number_of_samples;i++){
			      	// Assuming i-phase component comes first ..?
			      	shared_main_addresses[r][c][0][i] = (int32_t)
			    		( ((int32_t) (bb_vec_ptrs[MAIN_INPUT][i].real()) << 16) & 0xffff0000)| 
			    		( ((int32_t) (bb_vec_ptrs[MAIN_INPUT][i].imag()) ) & 0x0000ffff);
			      	shared_back_addresses[r][c][0][i] = (int32_t)
			    		( ((int32_t) (bb_vec_ptrs[BACK_INPUT][i].real()) << 16) & 0xffff0000)| 
			    		( ((int32_t) (bb_vec_ptrs[BACK_INPUT][i].imag()) ) & 0x0000ffff);

                    //Rx:
			    	if(verbose>100){
                            printf("%i\t",i);
                            for (size_t ifreq=0; ifreq<rx.get_num_clients(); ifreq++){
                                printf("%5.0f, ",std::abs(shared_main_addresses[r][c][0][i]));
                                printf("%5.0f \t ",360. / M_PI * std::arg(shared_main_addresses[r][c][0][i]));
                                //printf("%5.0f, ",std::abs(bb_vec_ptrs[MAIN_INPUT][i]));
                                //printf("%5.0f \t ",360. / M_PI * std::arg(bb_vec_ptrs[MAIN_INPUT][i]));
                            }
                            printf("\n");
			    	}
			    	//if(verbose>100){
                    //        printf("%i\t\t",i);
			        //    	printf("(%hi,",((shared_main_addresses[r][c][0][i] >> 16) & 0x0000ffff));
			        //    	printf("%hi)\t",(shared_main_addresses[r][c][0][i] & 0x0000ffff));

			        //    	printf("(%hi,",((shared_back_addresses[r][c][0][i] >> 16) & 0x0000ffff));
			        //    	printf("%hi)\n",(shared_back_addresses[r][c][0][i] & 0x0000ffff));
			    	//}
			      }
                  gettimeofday(&t6,NULL);
                  elapsed=(t6.tv_sec-t1.tv_sec)*1E6;
                  elapsed+=(t6.tv_usec-t1.tv_usec);
                  if (verbose > 1) {
                    std::cout << "Data sent: Elapsed Microseconds: " << elapsed << "\n";
                  }

			      //iseq++;
			      
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
			      rval=send_data(msgsock,&nrx_samples,sizeof(nrx_samples));
			      
			      if(IMAGING==0){
                    if(verbose > 1 ) std::cout << "Sending shared addresses..: " << 
			        shared_main_addresses[r][c][0] << "\t" << shared_back_addresses[r][c][0] << std::endl;
			      	rval=send_data(msgsock,&shared_main_addresses[r][c][0],sizeof(unsigned int));
			      	//rval=send_data(msgsock,&shared_main_addresses[r][c][0],sizeof(uint32_t**));
			      	rval=send_data(msgsock,&shared_back_addresses[r][c][0],sizeof(unsigned int));
			      	//rval=send_data(msgsock,&shared_back_addresses[r][c][0],sizeof(uint32_t**));
			      }
			      if (verbose>1)std::cout << "Send data to client successful" << std::endl;
			      msg.status = rx_status_flag;
			      rval=send_data(msgsock,&msg,sizeof(DriverMsg));
			    }
                rx_status_flag=0;
                gettimeofday(&t6,NULL);
                elapsed=(t6.tv_sec-t0.tv_sec)*1E6;
                elapsed+=(t6.tv_usec-t0.tv_usec);
                if (verbose > 1) {
                  std::cout << "Data sent: Elapsed Microseconds: " << elapsed << "\n";
                }

			    if(verbose>1)std::cout << "Ending RECV_GET_DATA. Elapsed time: " << std::endl;
                gettimeofday(&t6,NULL);
                elapsed=(t6.tv_sec-t0.tv_sec)*1E6;
                elapsed+=(t6.tv_usec-t0.tv_usec);
                if (verbose > 1) {
                  std::cout << "RECV_GET_DATA Elapsed Microseconds: " << elapsed << "\n";
                }
                usleep(10000);
			    break;

            case RECV_CLRFREQ:
                tx_threads.join_all();
			    receive_threads.join_all();
			    rx_process_threads.join_all();

			    if(verbose > 1) std::cout << "Doing clear frequency search!!!" << std::endl;
			    rval=recv_data(msgsock,&clrfreq_parameters,sizeof(struct CLRFreqPRM));
			    rval=recv_data(msgsock,&client,sizeof(struct ControlPRM));
                //printf("RECV_CLRFREQ for radar %i channel %i",client.radar, client.channel);
			    if (verbose) printf("Doing clear frequency search for radar %d, channel %d\n",client.radar,client.channel);
			    nave=0;
			    usable_bandwidth=clrfreq_parameters.end-clrfreq_parameters.start;
			    center=(clrfreq_parameters.start+clrfreq_parameters.end)/2;
			    if(verbose > -1 ){
			    	printf("  requested values\n");
                    printf("    start: %d\n",clrfreq_parameters.start);
                    printf("    end: %d\n",clrfreq_parameters.end);
                    printf("    center: %d\n",center);
                    printf("    bandwidth: %lf in Khz\n",(float)usable_bandwidth);
                    printf("    nave:  %d %d\n",nave,clrfreq_parameters.nave);
			    }
                            usable_bandwidth=floor(usable_bandwidth/2)*2;
			    /*
			    *  Set up fft variables
			    */
                N=(int)pow(2,ceil(log10(1.25*(float)usable_bandwidth)/log10(2)));
                if(N>1024){
                  N=512;
                  usable_bandwidth=300;
                  start=(int)(center-usable_bandwidth/2+0.49999);
                  end=(int)(center+usable_bandwidth/2+0.49999);
                }
			    /* 1 kHz fft bins*/

			    //Set the rx center frequency
			    //Set the rx sample rate
			    for(int chan = 0; chan < nrx_antennas; chan++) {
			            usrp->set_rx_freq(center, chan);
			            usrp->set_rx_rate(1000*N, chan);
			    	    //N = (int) (usrp->get_rx_rate() / 1000);
			    if(verbose>-1) std::cout << "Actual RX rate for clr freq search: " << N << " kHz\n";
			    }

                /* set up search parameters search_bandwidth > usable_bandwidth */
                search_bandwidth=N;
                //search_bandwidth=800;            
                start=(int)(center-search_bandwidth/2.0+0.49999);
                end=(int)(center+search_bandwidth/2+0.49999);
                unusable_sideband=(search_bandwidth-usable_bandwidth)/2;
                clrfreq_parameters.start=start;
                clrfreq_parameters.end=end;
                if(verbose > 1 ){
			    	printf("  search values\n");
                    printf("  start: %d %d\n",start,clrfreq_parameters.start);
                    printf("  end: %d %d\n",end,clrfreq_parameters.end);
                    printf("  center: %d\n",center);
                    printf("  search_bandwidth: %lf in Khz\n",search_bandwidth);
                    printf("  usable_bandwidth: %d in Khz\n",usable_bandwidth);
                    printf("  unusable_sideband: %lf in Khz\n",unusable_sideband);
                    printf("  N: %d\n",N);
                    printf("Malloc fftw_complex arrays %d\n",N);
			    }

                if(pwr!=NULL) {free(pwr);pwr=NULL;}
                pwr = (double*) malloc(N*sizeof(double));
			    for(int i=0;i<N;i++)
			    	pwr[i]=0.;
                //if(pwr2!=NULL) {free(pwr2);pwr2=NULL;}
                //pwr2 = (double*) malloc(N*sizeof(double));

			    if(verbose>1)std::cout << "starting clr freq search\n";

                //usrp->set_rx_freq(1e3*center);
                //usrp->set_rx_rate(1e3*center);
			    rx_clrfreq_rval= recv_clr_freq(usrp,rx_stream,search_bandwidth,clrfreq_parameters.nave,pwr);
	
			    pwr2 = &pwr[(int)unusable_sideband];

			    if(verbose > 0 ) printf("Send clrfreq data back\n");
                            rval=send_data(msgsock, &clrfreq_parameters, sizeof(struct CLRFreqPRM));
                            rval=send_data(msgsock, &usable_bandwidth, sizeof(int));
                            if(verbose > 1 ) {
			    	printf("  final values\n");
                            	printf("  start: %d\n",clrfreq_parameters.start);
                            	printf("  end: %d\n",clrfreq_parameters.end);
                            	printf("  nave: %d\n",clrfreq_parameters.nave);
                            	printf("  usable_bandwidth: %d\n",usable_bandwidth);
			    }
                //for (int i=0; i<usable_bandwidth; i++){
                //    std::cout << pwr2[i] << std::endl;
                //}
                rval=send_data(msgsock, pwr2, sizeof(double)*usable_bandwidth);  //freq order power
                rval=send_data(msgsock, &msg, sizeof(struct DriverMsg));

			    //clr_fd = fopen("/tmp/clr_data.txt","a+");
			    //for(int i=0;i<usable_bandwidth;i++){
                            //    printf("%4d: %8d %8.3lf\n",i,(int)(unusable_sideband+start+i),pwr2[i]);
                            //    fprintf(clr_fd,"%4d %8d %8.3lf\n",i,(int)(unusable_sideband+start+i),pwr2[i]);
                            //  }
                            //fclose(clr_fd);

                if(pwr!=NULL) {free(pwr);pwr=NULL;}
			    /* The following free causes crash because pwr2 is in use by the arby_server.
			    Does arby_server free() this pointer?  Or is this a memory leak? (AFM)*/
                            //if(pwr2!=NULL) {free(pwr2);pwr2=NULL;}
			    
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
