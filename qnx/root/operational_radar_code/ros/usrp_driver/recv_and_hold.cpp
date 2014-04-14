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
    uhd::usrp::multi_usrp::sptr usrp,
    uhd::rx_streamer::sptr rx_stream,
    std::vector<std::complex<short> *> client_buff_ptrs,
    size_t num_requested_samples,
    uhd::time_spec_t start_time,
    int *return_status
){
    struct timeval t0,t1,rt0,rt1;
    gettimeofday(&rt0,NULL);
    double elapsed_time=0;
    int isample=0,num_total_samps=0,inxbuff=0;
    int num_usrp_samples = (int) (num_requested_samples);
    //int num_usrp_samples = (int) (num_requested_samples*10000);
    std::vector<std::vector<std::complex<short> > >  temp_buffs;
    std::vector<std::complex<short> *> temp_buff_ptrs;
    //num_usrp_samples = 100000;

    for (size_t i=0; i<usrp->get_rx_num_channels(); i++)
	temp_buffs.push_back(std::vector<std::complex<short> >(num_usrp_samples));
    for (size_t i=0; i<usrp->get_rx_num_channels(); i++)
	temp_buff_ptrs.push_back(&temp_buffs[i].front());

    //setup streaming
    uhd::rx_metadata_t md;
    float timeout = 2;
    num_requested_samples=(int)num_requested_samples;
    uhd::stream_cmd_t stream_cmd = uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE;
    stream_cmd.num_samps = num_usrp_samples;
    stream_cmd.stream_now = false;
    stream_cmd.time_spec = start_time;

    usrp->issue_stream_cmd(stream_cmd);
    md.error_code = uhd::rx_metadata_t::ERROR_CODE_NONE;
    //while(num_total_samps < num_usrp_samples){
        int num_rx_samps = rx_stream->recv(client_buff_ptrs, num_usrp_samples, md, timeout);
	if (num_rx_samps != num_usrp_samples){
		uhd::time_spec_t rx_error_time = usrp->get_time_now();
		std::cerr << "Error in receiving samples..(" << rx_error_time.get_real_secs() << ")\t";;
		std::cerr << "Samples rx'ed: " << num_rx_samps << 
			" (expected " << num_usrp_samples << ")" << std::endl;
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
            std::cerr << "Unexpected error code " << md.error_code <<
		" encountered at " << rx_error_time.get_real_secs() << std::endl;
	    *return_status=-1;
        }

    //}
    for (size_t i=0;i<usrp->get_rx_num_channels();i++)
    	client_buff_ptrs[i] = &temp_buffs[i].front();

    gettimeofday(&rt1,NULL); 
    if(verbose > -1){
    	//std::cout << "recv_and_hold() elapsed time: " << 1e6*(rt1.tv_sec-rt0.tv_sec)+(rt1.tv_usec-rt0.tv_usec) << " usec" << std::endl;
    	//std::cout << "Expected time: " << 300.*(float)num_requested_samples << std::endl;
    }
}

