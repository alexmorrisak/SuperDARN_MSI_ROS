#include <complex>
#include <vector>
//Added by Alex for usrp
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/utils/thread_priority.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/utils/static.hpp>
#include <uhd/exception.hpp>
#include <boost/thread.hpp>
#include <thread>
#include <math.h>

#define NTHREADS 2

extern int verbose;

/***********************************************************************
 * recv_and_hold() function
 * A function to be used to receive samples from the USRP and
 * hold them in the network socket buffer. *client_buff_ptrs point to
 * the memory locations of each antenna's samples.   Meant to operate in its
 * own thread context so that it does not block the execution in main()
 **********************************************************************/
void recv_and_hold(
    double* trtimes,
    int npulses,
    uhd::usrp::multi_usrp::sptr usrp,
    uhd::rx_streamer::sptr rx_stream,
    std::vector<std::complex<int16_t> *> client_buff_ptrs,
    size_t num_requested_samples,
    uhd::time_spec_t start_time,
    int *return_status
){
    std::vector<std::vector<std::complex<short> > >  temp_buffs;
    std::vector<std::complex<short> *> temp_buff_ptrs;

    //setup streaming
    uhd::rx_metadata_t md;
    float timeout = 0.1;
    //num_requested_samples=(int)num_requested_samples;
    uhd::stream_cmd_t stream_cmd = uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE;
    stream_cmd.num_samps = num_requested_samples;
    stream_cmd.stream_now = false;
    stream_cmd.time_spec = start_time;

    usrp->set_gpio_attr("TXA","CTRL",0x0,0x40);
    usrp->set_gpio_attr("TXA","DDR",0x40,0x40);
    usrp->set_gpio_attr("TXA","CTRL",0x0,0x20);
    usrp->set_gpio_attr("TXA","DDR",0x20,0x20);
    usrp->set_command_time(start_time-10e-6);
    usrp->set_gpio_attr("TXA","OUT",0x40,0x40);

    usrp->issue_stream_cmd(stream_cmd);

    usrp->set_command_time(start_time+1e-6);
    usrp->set_gpio_attr("TXA","OUT",0x00,0x40);
    for (int i=0; i<npulses; i++){
        //std::cout << "ipulse: " << i << " time: " << trtimes[i] << std::endl;
        usrp->set_command_time(start_time+trtimes[i]);
        usrp->set_gpio_attr("TXA","OUT",0x20,0x20);

        usrp->set_command_time(start_time+trtimes[i]+600*1e-6);
        usrp->set_gpio_attr("TXA","OUT",0x00,0x20);
    }
        

    md.error_code = uhd::rx_metadata_t::ERROR_CODE_NONE;
    size_t num_rx_samps = rx_stream->recv(client_buff_ptrs, num_requested_samples, md, timeout);
	if (num_rx_samps != num_requested_samples){
        *return_status=-1;
		uhd::time_spec_t rx_error_time = usrp->get_time_now();
		std::cerr << "Error in receiving samples..(" << rx_error_time.get_real_secs() << ")\t";;
		std::cerr << "Samples rx'ed: " << num_rx_samps << 
			" (expected " << num_requested_samples << ")" << std::endl;
	}

        if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT) {
	    uhd::time_spec_t rx_error_time = usrp->get_time_now();
            std::cerr << "Timeout encountered at " << rx_error_time.get_real_secs() << std::endl;
	    *return_status=-1;
        }
        if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_OVERFLOW){
	    uhd::time_spec_t rx_error_time = usrp->get_time_now();
            std::cerr << "Overflow encountered at " << rx_error_time.get_real_secs() << std::endl;
	    *return_status=-1;
        }
        if (md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE){
	    uhd::time_spec_t rx_error_time = usrp->get_time_now();
	    std::cerr << "start time: " << start_time.get_real_secs() << std::endl;
            std::cerr << "Unexpected error code " << md.error_code <<
		" encountered at " << rx_error_time.get_real_secs() << std::endl;
	    *return_status=-1;
        }
        if (md.out_of_sequence){
	    uhd::time_spec_t rx_error_time = usrp->get_time_now();
	    std::cerr << "start time: " << start_time.get_real_secs() << std::endl;
            std::cerr << "Packets out of order " << 
		" encountered at " << rx_error_time.get_real_secs() << std::endl;
	    *return_status=-1;
        }

    //for (size_t i=0;i<usrp->get_rx_num_channels();i++)
    //	client_buff_ptrs[i] = &temp_buffs[i].front();

    //gettimeofday(&rt1,NULL); 
    //if(verbose > -1){
    	//std::cout << "recv_and_hold() elapsed time: " << 1e6*(rt1.tv_sec-rt0.tv_sec)+(rt1.tv_usec-rt0.tv_usec) << " usec" << std::endl;
    	//std::cout << "Expected time: " << 300.*(float)num_requested_samples << std::endl;
    //for (size_t i=0;i<usrp->get_rx_num_channels();i++)
    //	client_buff_ptrs[i] = &temp_buffs[i].front();

    if(verbose > 1 && *return_status==0){
	    std::cout << "recv_and_hold() succesful\n";
    }
}

