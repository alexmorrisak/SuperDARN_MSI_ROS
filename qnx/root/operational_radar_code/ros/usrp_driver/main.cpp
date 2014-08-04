#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/un.h>
#include <sys/socket.h>
#include <string.h>
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
#include "alex_custom.hpp"
#include "txrx_data.hpp"
#include <math.h>
#include "cuda_prog.h"
#include "socket_utils.hpp"

#ifdef __cplusplus
	extern "C" {
#endif
	#include "control_program.h"
	#include "global_server_variables.h"
	#include "timing_defs.h"
	//#include "iniparser.h"
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

#define X_BIT 0x04 //Transmit bit
#define TR_BIT 0x02 //TR bit
#define S_BIT 0x80 //Scope sync
#define P_BIT 0x10 //Phase bit.  0 for 0 degrees, 1 for 180 degrees

#define TXRATE 5e6
#define TXFREQ 12e6
#define RXRATE 5e6
#define RXFREQ 12e6

#define MIMO 1
#define NUNITS 2

#define NRADARS 2

//dictionary *Site_INI;
int sock=0,msgsock=0,tmp=0;
int verbose=10;
int double_buf=1;
int configured=1;
int		writingFIFO=0, dma_count=0, under_flag=0,empty_flag=0,IRQ, intid;
//int		max_seq_count=0, xfercount=0, totransfer=0;
uintptr_t	mmap_io_ptr_dio;
unsigned int	virtual_addr[1], physical_addr[1];
struct sigevent interruptevent;
int tr_event=0, scope_event=0;

pthread_t int_thread;
void graceful_cleanup(int signum)
{
  //char path[256];
  //sprintf(path, "%s", "rostiming");
  //std::cout << "Unlinking Unix Socket: " << path << "\n";
  std::cout << "Closing msgsock and sock\n";
  close(msgsock);
  close(sock);
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
//double *pwr=NULL,*pwr2=NULL;
std::vector<double> pwr;
std::vector<double> pwr2;
FILE *clr_fd;
int main(){
    // DECLARE AND INITIALIZE ANY NECESSARY VARIABLES
    int     maxclients=MAX_RADARS*MAX_CHANNELS+1;
    struct  ControlPRM  clients[maxclients], client;
    struct  CLRFreqPRM clrfreq_parameters;
    int ready_index[MAX_RADARS][MAX_CHANNELS];
    int old_seq_id=-10;
    int new_seq_id=-1;
	int old_beam=-1;
	int new_beam=-1;
	int nave=0;
	int center=0;
	int usable_bandwidth=0;
    struct TRTimes bad_transmit_times;

	uint32_t *shared_main_addresses[MAX_RADARS][MAX_CHANNELS][MAX_INPUTS];
	uint32_t *shared_back_addresses[MAX_RADARS][MAX_CHANNELS][MAX_INPUTS]; 
	
	char shm_device[80];
	int shm_fd=0;

	//socket and message passing variables
	char	datacode;
	int	rval;
    fd_set rfds,efds;

	//counter and temporary variables
    //int offset_pad, rx_offset, dds_offset, tx_offset;
    //int scope_start, dds_trigger, rx_trigger;
	int	i,r,c,buf,index;
    struct timeval t0,t6;
    struct timeval t1;
    unsigned long elapsed;

	// function-specific message variables
    int     numclients=0;
    struct  DriverMsg msg;

	// timing related variables
	struct	timespec cpu_start, cpu_stop;
	struct	timespec tx0,tx1;
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

    std::vector<std::complex<float> > beamform_main;
    std::vector<std::complex<float> > beamform_back;

    /* Example single-site radar configuration, imaging*/
    //tx_data tx(1, 16, TXFREQ, TXRATE);
    //rx_data rx(1,20,RXFREQ, RXRATE);
    /* Example dual-site radar configuration, imaging*/
    //tx_data tx(2, 32, TXFREQ, TXRATE);
    //rx_data rx(2,40,RXFREQ, RXRATE);
    /* Example single-site radar configuration, non-imaging*/
    //tx_data tx(1, 1, TXFREQ, TXRATE);
    //rx_data rx(1,2,RXFREQ, RXRATE);
    /* Example dual-site radar configuration, non-imaging*/
    //tx_data tx(2, 2, TXFREQ, TXRATE);
    //rx_data rx(2,4,RXFREQ, RXRATE);
    /* For testing*/
    tx_data tx(1, NUNITS, TXFREQ, TXRATE);
    rx_data rx(1, 2*NUNITS, RXFREQ, RXRATE);

	// Swing buffered.
	unsigned int iseq=0;

    double time_delay;

	//variables to be used as arguments to setup the usrp device(s)
	std::string args, txsubdev, rxsubdev;
	args = "addr0=192.168.10.2, addr1=192.168.10.3";
	if(NUNITS==1)
	args = "addr0=192.168.10.2";
	txsubdev = "A:A";
	rxsubdev = "A:A A:B"; //Use two rx channels per daughterboard
	//rxsubdev = "A:A"; //Use one rx channel per daughterboard

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

            /*Poll the usrp(s), waiting for the a PPS signal to occur */
            const uhd::time_spec_t last_pps_time = usrp->get_time_last_pps();
            while (last_pps_time == usrp->get_time_last_pps()) {
                usleep(5e4);
            }
            /* Now that a PPS has occured, we have 
             * almost one full second to sync all the devices */
			usrp->set_time_next_pps(uhd::time_spec_t(0.0), 0);
			usrp->set_time_next_pps(uhd::time_spec_t(0.0), 1);
			
			//Sleep a bit while the USRP's lock to the external reference
			boost::this_thread::sleep(boost::posix_time::milliseconds(1100));
		}
	}


	//Set Tx sample rate and center frequency
	usrp->set_tx_rate(TXRATE);
    usrp->set_tx_freq(TXFREQ);
	if (verbose) std::cout << boost::format("TX Rate: %f kHz...") % (usrp->get_tx_rate()/1e3) << std::endl;
	if (verbose) std::cout << boost::format("TX Freq: %f Hz...") % (usrp->get_tx_freq()/1e3) << std::endl;
    float txrate = usrp->get_tx_rate();
    if ((int)txrate != (int)TXRATE) {
        std::cerr << "Requested Tx sample rate not accepted by USRP.  Exiting..." << std::endl; 
        exit(EXIT_FAILURE);
    }
    if ((int)usrp->get_tx_freq() != (int)TXFREQ) {
        std::cerr << "Requested Tx center freq not accepted by USRP.  Exiting..." << std::endl; 
        exit(EXIT_FAILURE);
    }
	//Set Rx sample rate and center frequency
	usrp->set_rx_rate(RXRATE);
    usrp->set_rx_freq(RXFREQ);
	if (verbose) std::cout << boost::format("RX Rate: %f kHz...") % (usrp->get_rx_rate()/1e3) << std::endl;
	if (verbose) std::cout << boost::format("RX Freq: %f kHz...") % (usrp->get_rx_freq()/1e3) << std::endl;
    if ((int)usrp->get_rx_rate() != (int)RXRATE) {
        std::cerr << "Requested Rx sample rate not accepted by USRP.  Exiting..." << std::endl; 
        exit(EXIT_FAILURE);
    }
    if ((int)usrp->get_rx_freq() != (int)RXFREQ) {
        std::cerr << "Requested Rx center freq not accepted by USRP.  Exiting..." << std::endl; 
        exit(EXIT_FAILURE);
    }

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
		    if (ftruncate(shm_fd,MAX_SAMPLES*4) != 0){
		        std::cerr << "ftruncate error!!\n";
            }
            shared_main_addresses[r][c][0]=(uint32_t *)mmap(0,MAX_SAMPLES*4,PROT_READ|PROT_WRITE,MAP_SHARED,shm_fd,0);
            close(shm_fd);

            sprintf(shm_device,"/receiver_back_%d_%d_%d",r,c,0);
            shm_unlink(shm_device);
            shm_fd=shm_open(shm_device,O_RDWR|O_CREAT,S_IRUSR | S_IWUSR);
		    if (ftruncate(shm_fd,MAX_SAMPLES*4) != 0){
		        std::cerr << "ftruncate error!!\n";
            }
            shared_back_addresses[r][c][0]=(uint32_t *)mmap(0,MAX_SAMPLES*4,PROT_READ|PROT_WRITE,MAP_SHARED,shm_fd,0);
            close(shm_fd);
            
            // For testing..
            for (i=0;i<MAX_SAMPLES;i++) {
                shared_main_addresses[r][c][0][i]=i;
                shared_back_addresses[r][c][0][i]=i;
            }
        }
    }



   signal(SIGINT, graceful_cleanup);
   signal(SIGTERM, graceful_cleanup);

    //Site_INI=NULL;
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

    //max_seq_count=0;
	if (verbose > 1) std::cout << "Zeroing arrays\n";
    bad_transmit_times.length=0;
    bad_transmit_times.start_usec=(uint32_t*) malloc(sizeof(unsigned int)*MAX_PULSES);
    bad_transmit_times.duration_usec=(uint32_t*) malloc(sizeof(unsigned int)*MAX_PULSES);

    //master_buf=(char*) malloc(MAX_TIME_SEQ_LEN*sizeof(int32_t));

	clock_gettime(CLOCK_REALTIME, &cpu_stop);
	float elapsed_setup = 1e-9*(cpu_stop.tv_nsec-cpu_start.tv_nsec) + 
        cpu_stop.tv_sec - cpu_stop.tv_sec;
	if(verbose>1)std::cout << "Elapsed setup time: " << elapsed_setup << " sec\n";

    // OPEN TCP SOCKET AND START ACCEPTING CONNECTIONS 
	if (verbose > 1) printf("timing host port: %i \n", TIMING_HOST_PORT);
	sock=tcpsocket(TIMING_HOST_PORT);
	//bind(sock, (struct sockaddr *) TIMING_HOST_IP, sizeof(struct sockaddr_un));
	//printf("Done binding socket: %i \n", sock);
        //sock=server_unixsocket("/tmp/rostiming",0);
	listen(sock, 5);

    //Create thread groups
	boost::thread_group receive_threads;
	boost::thread_group rx_process_threads;
	boost::thread_group tx_threads;

	int32_t rx_status_flag;
	int32_t frame_offset;
	int32_t dma_buffer;
	//int32_t nrx_samples;
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
           //rval = select(msgsock+1, &rfds, NULL, &efds, NULL);
           if (select(msgsock+1, &rfds, NULL, &efds, NULL) < 0){
               std::cerr << "Error in select. " << strerror(errno) << std::endl;
           }
           if (recv(msgsock, &buf, sizeof(int), MSG_PEEK) <= 0){
               std::cerr << "Error in recv data. " << strerror(errno) << std::endl;
           }
		   if (verbose > 1) std::cout << msgsock << " Leaving Select " << rval << "\n";
           /* tv value might be modified -- Don’t rely on the value of tv now! */
           if (FD_ISSET(msgsock,&efds)){
            if (verbose > 1) std::cout << "Exception on msgsock " << msgsock << "  ...closing\n";
            break;
           }
           if (rval == -1) perror("select()");
           //rval=recv(msgsock, &buf, sizeof(int), MSG_PEEK); 
           if (recv(msgsock, &buf, sizeof(int), MSG_PEEK) <= 0){
               std::cerr << "Error in recv data. " << strerror(errno) << std::endl;
           }
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
            recv_data(msgsock, &msg, sizeof(struct DriverMsg));
            datacode=msg.type;
		    if (verbose > 1) std::cout << "\nmsg code is " <<  datacode << "\n";
		   switch( datacode ){
		      case TIMING_REGISTER_SEQ:
		        if (verbose > 0) std::cout << "\nRegister new sequence for usrp driver\n";
		        msg.status=0;
                recv_data(msgsock, &client, sizeof(struct ControlPRM));
                r=client.radar-1; 
                c=client.channel-1; 

                if (verbose>1) std::cout << "Registering rx client radar " << r << " channel " << c << std::endl;
                //rx.register_client(client);
                if (verbose>1) std::cout << "Registering tx client\n";
                //tx.register_client(client);

			    if (verbose > 1) std::cout << "Radar: " << client.radar <<
				    " Channel: " << client.channel << " Beamnum: " << client.tbeam <<
				    " Status: " << msg.status << "\n";
                recv_data(msgsock, &index, sizeof(index));
		        if (verbose > 1) std::cout << "Requested index: " << r << " " << c << " " << index << "\n";
                recv_data(msgsock, tx.get_tsg_ptr(index), sizeof(struct TSGbuf));
                tx.allocate_pulseseq_mem(index);

                recv_data(msgsock, tx.get_tsg_ptr(index)->rep,
                    sizeof(unsigned char)*tx.get_tsg_ptr(index)->len);
                recv_data(msgsock, tx.get_tsg_ptr(index)->code,
                    sizeof(unsigned char)*tx.get_tsg_ptr(index)->len);
			    //if (verbose > 1) std::cout << "Pulseseq length: " << pulseseqs[r][c][index]->len << "\n";
                old_seq_id=-10;
                //old_pulse_index[r][c]=-1;
                new_seq_id=-1;
                send_data(msgsock, &msg, sizeof(struct DriverMsg));
                break;

		      case TIMING_CtrlProg_END:
                recv_data(msgsock, &client,sizeof(struct ControlPRM));
		        if (verbose > 0) printf("A client is done. Radar: %i, Channel: %i\n", client.radar-1, client.channel-1);
                r=client.radar-1;
                c=client.channel-1;
                //tx.unregister_client(r,c);
                //rx.unregister_client(r,c);
                msg.status=0;
                send_data(msgsock, &msg, sizeof(struct DriverMsg));
                old_seq_id=-10;
                new_seq_id=-1;
                break;

		      case TIMING_CtrlProg_READY:
                //numclients = 0;
		        if (verbose > 1) printf("\nAsking to set up timing info for client that is ready %i\n",numclients);
                msg.status=0;
                recv_data(msgsock, &client,sizeof(struct ControlPRM));
                r=client.radar-1; 
                c=client.channel-1; 

                //rx.ready_client(cient);
                rx.ready_client(&client);
                tx.ready_client(&client);
                
                if ((ready_index[r][c]>=0) && (ready_index[r][c] <maxclients) ) {
                    clients[ready_index[r][c]]=client;
                } 
                else {
                    clients[numclients]=client;
                    ready_index[r][c]=numclients;
                    numclients=(numclients+1);
                }

			    if (verbose > 1) std::cout << "Radar: " << client.radar << " Channel: " << client.channel <<
					" Beamnum: " << client.tbeam << " Status: " << msg.status << "\n";

                //time_delays.resize(tx.get_num_channels());
                //for (size_t i=0; i< tx.get_num_channels(); i++)
			    //    time_delays[i] = 10 * (16/2-client.tbeam); // 10 ns per antenna per beam

                index=client.current_pulseseq_index; 

                tx.unpack_pulseseq(index);

                if (numclients >= maxclients) msg.status=-2;
		        if (verbose > 1) std::cout << "\nclient ready\n";
                numclients=numclients % maxclients;
                send_data(msgsock, &msg, sizeof(struct DriverMsg));
                break; 

            case TIMING_PRETRIGGER:
                gettimeofday(&t0,NULL);
			    if(verbose > 1 ) std::cout << "Setup Timing Card for next trigger\n";
                msg.status=0;
                new_seq_id=-1;
			    new_beam=client.tbeam;

	            for(int i=0; i<numclients; i++){
                    r=clients[i].radar-1;
                    c=clients[i].channel-1;
                    new_seq_id += r*1000 + c*100 + clients[i].current_pulseseq_index+1;
                    if (verbose > 1) std::cout << i << " " <<
                        new_seq_id << " " << clients[i].current_pulseseq_index << "\n";
                }

                if (verbose > 1) std::cout << "new, old sequence id's: " << new_seq_id << " " << old_seq_id << "\n";

                tx.make_bb_vecs(clients[0].trise);
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
                    //max_seq_count=0;

                    rx.reset_swing_buf();
                    tx.make_tr_times(&bad_transmit_times);

                    //    /* Handle Scope Sync Signal logic */
                    //    if ((master_buf[i] & S_BIT)==S_BIT) { // Scope signal logic on
                    //        if (scope_event==0) { 
                    //            if (verbose > 1 ) std::cout << "Scope Sync sample start: " << i << " " << master_buf[i] << "\n";
                    //            scope_start=i;
                    //        }
                    //        scope_event=1;
                    //    }
                    //    else {
                    //        if (scope_event==1 && verbose > 1){
                    //            std::cout << "Scope Sync sample end: " << i << " " << master_buf[i] << "\n";
                    //        }
                    //        scope_event=0;
                    //    }
                    //}

                    /* The following is leftover from the old timing card code.  Might be useful reference
                     * if we want to add other logic signals besides scope-sync and tr gate */
                    /*
                    if (scope_start>-1) { 
                      //dds_trigger=scope_start+(int)((double)dds_offset/((double)STATE_TIME+0.5));
                      //rx_trigger=scope_start+(int)((double)rx_offset/((double)STATE_TIME+0.5));
                      //if (verbose > 1 ) {
                      //  std::cout << "---> Scope Sync in Master " << max_seq_count << " at " << scope_start << "\n";
                      //  std::cout << "---> DDS Trigger in Master " << max_seq_count << " at " << dds_trigger << "\n"; 
                      //  std::cout << "---> Rx Trigger in Master " << max_seq_count << " at " << rx_trigger << "\n"; 
                      //} 
                    }
                    else {
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
                    
                    */
			    }

			    if (new_beam != old_beam) { //Re-calculate the RF sample vectors for new beam direction.  BB samples are the same.
			    	for(size_t chan = 0; chan < usrp->get_rx_num_channels(); chan++) {
			    	        usrp->set_rx_rate(RXRATE, chan);
			                usrp->set_rx_freq(RXFREQ, chan);
			    	}
			    	for(size_t chan=0; chan < usrp->get_tx_num_channels(); chan++) {
			    	        usrp->set_tx_rate(TXRATE, chan);
			                usrp->set_tx_freq(TXFREQ, chan);
			    	}

                    rx_process_threads.join_all(); //Make sure rx processing is done so that we have exclusive access to the GPU

                    clock_gettime(CLOCK_MONOTONIC, &tx0);
                    tx.allocate_rf_vec_mem();
                    /* Process the waveforms for each radar*/
                    for (size_t iradar=0; iradar<tx.get_num_radars(); iradar++){

                        if (tx.get_num_channels(iradar) == 0){ 
                            tx.zero_rf_vec(iradar);
                            std::cout << "zeroing radar number " << iradar << std::endl;
                            continue;
                        }
                        if(verbose>1)std::cout << "about to enter tx_process_gpu\n";
				        tx_process_gpu(
                            tx.get_bb_vec_ptr(iradar),
                            tx.get_rf_vec_ptrs(iradar),
				        	tx.get_num_bb_samples(),
				        	tx.get_num_rf_samples(),
				        	usrp->get_tx_freq(),
                            TXRATE,
                            tx.get_freqs(iradar), // List of center frequencies for this radar
                            tx.get_time_delays(iradar),
                            tx.get_num_channels(iradar), // Number of channels for this radar
                            tx.get_num_ants_per_radar()); //number of antennas per radar
                    }

                    clock_gettime(CLOCK_MONOTONIC, &tx1);
                    elapsed_t1 = (1e9*tx1.tv_sec+tx1.tv_nsec) - (1e9*tx0.tv_sec+tx0.tv_nsec);
                    if (verbose) printf("\n\ntx mix upsample elapsed time: %.2f ms\n", elapsed_t1/1e6);
			    }

                if (new_seq_id < 0 ) {
                  old_seq_id=-10;
                }
                else {
                  old_seq_id=new_seq_id;
                }
                new_seq_id=-1;
			    old_beam = new_beam;

                if (verbose > 2){
                    std::cout << "bad_transmit_times.length: " <<  bad_transmit_times.length << std::endl;
                }
                send_data(msgsock, &bad_transmit_times.length, sizeof(bad_transmit_times.length));
                send_data(msgsock, bad_transmit_times.start_usec, sizeof(uint32_t)*bad_transmit_times.length);
                send_data(msgsock, bad_transmit_times.duration_usec, sizeof(uint32_t)*bad_transmit_times.length);
			    msg.status=0;
                if (verbose > 1)  std::cout << "Ending Pretrigger Setup\n";
                send_data(msgsock, &msg, sizeof(struct DriverMsg));

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
			        //First make sure sure we're done receiving the last pulse sequence
			        if (verbose > 1) std::cout << "Waiting on thread.." << std::endl;
			        receive_threads.join_all();
			  
			        if (verbose > 1){
			  	        std::cout << "client baseband sample rate: " << client.baseband_samplerate << std::endl;
			  	        std::cout << "nsamples: " << rx.get_num_rf_samples() << std::endl;
			  	        std::cout << "Usrp sample rate: " << RXRATE << std::endl;
			        }

			        //Start the receive stream thread
			        if(verbose>1) std::cout << "About to start rx thread..\n";
			        rx_thread_status=0;
			        gettimeofday(&t0,NULL);
	
                    tx.set_rf_vec_ptrs(&tx_vec_ptrs);
                    rx.set_rf_vec_ptrs(&rx_vec_ptrs);
                    /* Toggle the swing buffer for the NEXT pulse sequence
                     * The rx_data::get_num_samples(), get_rf_dptr(), get_bb_dptr(), get_num_clients(), etc
                     * commands still point to the CURRENT pulse sequence */
                    rx.toggle_swing_buffer(); 

			        uhd::time_spec_t start_time = usrp->get_time_now() + 0.005;

                    gettimeofday(&t1,NULL);
		            receive_threads.create_thread(boost::bind(recv_and_hold,
                        &bad_transmit_times,
			         	usrp,
			         	rx_stream,
			         	rx_vec_ptrs,
			         	rx.get_num_rf_samples(),
			         	start_time,
			         	&rx_thread_status));

			         //call function to start tx stream simultaneously with rx stream
		             tx_threads.create_thread(boost::bind(tx_worker,
                        tx_stream,
                        tx_vec_ptrs,
                        tx.get_num_rf_samples(),
                        start_time+20e-6));
                }
                //tx_threads.join_all();
                //receive_threads.join_all();

                send_data(msgsock, &msg, sizeof(struct DriverMsg));

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
                send_data(msgsock, &msg, sizeof(struct DriverMsg));
                break;

		    case TIMING_WAIT:
			    if(verbose > 1) std::cout << "USRP_TCP_DRIVER_WAIT: Nothing happens here.." << std::endl;
                msg.status=0;

			    if (verbose > 1) std::cout << "Read msg struct from tcp socket!\n";	
                //dead_flag=0;
                
                if (verbose > 1)  std::cout << "Ending Wait \n\n";
                send_data(msgsock, &msg, sizeof(struct DriverMsg));

			    break;

            case TIMING_POSTTRIGGER:
                tx.clear_channel_list();
                rx.clear_channel_list();
                numclients=0;
                if (verbose > 1) std::cout << "Post trigger.  Un-readying all clients\n\n";
                for (r=0;r<MAX_RADARS;r++){
                    for (c=0;c<MAX_CHANNELS;c++){
                        ready_index[r][c]=-1;
                    }
                }
                send_data(msgsock, &msg, sizeof(struct DriverMsg));
                break;

            case RECV_GET_DATA:
			    if (verbose>1) std::cout << "RECV_GET_DATA: Waiting on all threads.." << std::endl;

                tx_threads.join_all();
			    receive_threads.join_all();
			    rx_process_threads.join_all();
                gettimeofday(&t0,NULL);
			    //rx_thread_status=-1;

			    gettimeofday(&t6,NULL);
                elapsed=(t6.tv_sec-t1.tv_sec)*1E6;
                elapsed+=(t6.tv_usec-t1.tv_usec);
			    if (verbose>1)std::cout << "Rx thread elapsed: " << 
			    	elapsed << " usec" << std::endl;
                elapsed=(t6.tv_sec-t0.tv_sec)*1E6;

			    if (rx_thread_status==-1){
			    	std::cerr << "Error, bad status from Rx thread!!\n";
			    	rx_status_flag=-1;
			    }
			    else {
                    rx_status_flag=0;
			    	if (verbose>1) printf("Status okay!!\n");
			    }

                recv_data(msgsock, &client,sizeof(struct ControlPRM));
			    r=client.radar-1;
			    c=client.channel-1;
                if (rx.get_num_clients(r) == 0){
                    std::cout << "No channels for this radar.\n";
                    rx_status_flag=-1;
                }
                send_data(msgsock, &rx_status_flag, sizeof(int));

			    if (verbose>1)std::cout << "Client asking for rx samples (Radar,Channel): " <<
			    	client.radar << " " << client.channel << std::endl;
			    get_data_t0 = usrp->get_time_now();
			    r = client.radar-1; c = client.channel-1;
	
                if (verbose > 2){
                    std::cout << "radar: " << r << " channel: " << c << "\n";
                    std::cout << "bb_dptr: " << rx.get_bb_dptr(r) << std::endl;
                    std::cout << "rf dptr: " << rx.get_rf_dptr(r) << std::endl;
                    std::cout << "nrf_samples: " << rx.get_num_rf_samples() << std::endl;
                    std::cout << "nbb_samples: " << rx.get_num_bb_samples() << std::endl;
                    std::cout << "nchannels: " << rx.get_num_clients(r) << std::endl;
                    std::cout << "nants per radar: " << rx.get_num_ants_per_radar() << std::endl;
                    std::cout << "high sample rate: " << (float) RXRATE << std::endl;
                    std::cout << "low sample rate: " << (float) client.baseband_samplerate << std::endl;
                    std::cout << "frequency offsets: ";
                    for (size_t i=0; i<rx.get_num_clients(r); i++){
                        std::cout << (rx.get_freqs(r))[i] << std::endl;
                    }
                }
                if (verbose>1) std::cout << "About to enter rx_process_gpu()\n";
                if (rx_status_flag==0){
			        rx_process_threads.create_thread(boost::bind(rx_process_gpu,
                        rx.get_rf_dptr(r),
                        rx.get_bb_dptr(r),
                        rx.get_num_rf_samples(),
                        rx.get_num_bb_samples(),
			        	rx.get_num_clients(r),
			        	rx.get_num_ants_per_radar(),
			        	(float) RXRATE,
			        	(float) client.baseband_samplerate,
                        rx.get_freqs(r)));
                
                if (double_buf==0) rx_process_threads.join_all();

                if (verbose>1) printf("iseq: %i\n", iseq);
                if (verbose>1)std::cout << "set_bb_vec_ptrs: " << r << " " << c << std::endl;

                /* Set the pointers to vectors of baseband samples
                 * If double_buf is non-zero, then the function will create
                 * pointers to the samples created by the LAST pulse sequence.
                 * This allows the driver to collect samples for the current pulse
                 * sequence and process samples for the previous pulse sequence 
                 * simultaneously*/
                rx.set_bb_vec_ptrs(r,c, &bb_vec_ptrs,double_buf);

                beamform_main.resize(rx.get_num_ants_per_radar(), 1);
                beamform_back.resize(rx.get_num_ants_per_radar(), 1);
                rx_beamform(
                    shared_main_addresses[r][c][0],
			      	shared_back_addresses[r][c][0],
                    &bb_vec_ptrs,
                    //rx.get_num_ants_per_radar()/2,
                    //rx.get_num_ants_per_radar()/2,
                    1,
                    1,
                    rx.get_num_bb_samples(),
                    &beamform_main,
                    &beamform_back);


                gettimeofday(&t1,NULL);
                gettimeofday(&t6,NULL);
                elapsed=(t6.tv_sec-t1.tv_sec)*1E6;
                elapsed+=(t6.tv_usec-t1.tv_usec);
                if (verbose > 1) {
                  std::cout << "Data sent: Elapsed Microseconds: " << elapsed << "\n";
                }

			    iseq++;
			    
                if(verbose > 1 ){
			      std::cout << "Radar: " << client.radar << 
			             		"\tChannel: " << client.channel << std::endl;
			      std::cout << "r: " << r << "\tc: " << c << std::endl;
			    }
			    shm_memory=1; // Flag used to indicate to client if shared memory (mmap()) is used. 1 for yes.
                send_data(msgsock, &shm_memory, sizeof(shm_memory));
			    frame_offset=0;  // The GC316 cards normally produce rx data w/ a header of length 2 samples. 0 for usrp.
                send_data(msgsock, &frame_offset, sizeof(frame_offset));
			    dma_buffer=0; // Flag used to indicate to client if DMA tranfer is used. 1 for yes.
                send_data(msgsock, &dma_buffer, sizeof(dma_buffer));
			    //nrx_samples=client.number_of_samples;
                send_data(msgsock, &client.number_of_samples, sizeof(client.number_of_samples));
			    
			    if(IMAGING==0){
                  if(verbose > 1 ) std::cout << "Using shared memory addresses..: " << 
			      shared_main_addresses[r][c][0] << "\t" << shared_back_addresses[r][c][0] << std::endl;
			    }
			    if (verbose>1)std::cout << "Send data to client successful" << std::endl;
			    msg.status = rx_status_flag;
			    }
                send_data(msgsock, &msg, sizeof(struct DriverMsg));
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
			    break;

            case RECV_CLRFREQ:
                tx_threads.join_all();
			    receive_threads.join_all();
			    rx_process_threads.join_all();

			    if(verbose > 1) std::cout << "Doing clear frequency search!!!" << std::endl;
                recv_data(msgsock, &clrfreq_parameters,sizeof(struct CLRFreqPRM));
                recv_data(msgsock, &client,sizeof(struct ControlPRM));
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
			            usrp->set_rx_freq(1e3*center, chan);
			            usrp->set_rx_rate(1e6, chan);
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

                //if(pwr!=NULL) {free(pwr);pwr=NULL;}
                //pwr = (double*) malloc(N*sizeof(double));
                pwr.clear();
                pwr2.clear();
                pwr.resize(N,0);
                pwr2.resize(usable_bandwidth,0);
			    //for(int i=0;i<N;i++)
			    //	pwr[i]=0.;
                //if(pwr2!=NULL) {free(pwr2);pwr2=NULL;}
                //pwr2 = (double*) malloc(N*sizeof(double));

			    if(verbose>1)std::cout << "starting clr freq search\n";
                std::cout << "beam direction: " << client.tbeam << std::endl;
                std::cout << "beam direction: " << client.filter_bandwidth << std::endl;
                time_delay = 10*(16/2-client.tbeam);

                //usrp->set_rx_freq(1e3*center);
                //usrp->set_rx_rate(1e3*center);
			    rx_clrfreq_rval= recv_clr_freq(
                    usrp,
                    rx_stream,
                    usable_bandwidth,
                    (int) client.filter_bandwidth/1e3,
                    clrfreq_parameters.nave,
                    10,
                    &pwr2.front());
	
			    //pwr2 = &pwr[(int)unusable_sideband];

			    if(verbose > 0 ) printf("Send clrfreq data back\n");
                            send_data(msgsock, &clrfreq_parameters, sizeof(struct CLRFreqPRM));
                            send_data(msgsock, &usable_bandwidth, sizeof(int));
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
                send_data(msgsock, &pwr2.front(), sizeof(double)*usable_bandwidth);
                send_data(msgsock, &msg, sizeof(struct DriverMsg));

			    //clr_fd = fopen("/tmp/clr_data.txt","a+");
			    //for(int i=0;i<usable_bandwidth;i++){
                            //    printf("%4d: %8d %8.3lf\n",i,(int)(unusable_sideband+start+i),pwr2[i]);
                            //    fprintf(clr_fd,"%4d %8d %8.3lf\n",i,(int)(unusable_sideband+start+i),pwr2[i]);
                            //  }
                            //fclose(clr_fd);

                //if(pwr!=NULL) {free(pwr);pwr=NULL;}
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
