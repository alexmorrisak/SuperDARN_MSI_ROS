#include <complex>
#include <vector>
//Added by Alex for usrp
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/utils/thread_priority.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/utils/static.hpp>
#include <uhd/exception.hpp>


/***********************************************************************
 * recv_mix_to_buffer function
 * A function to be used to receive samples from the USRP(s) and
 * write them into a buffer.  Also performs bp-filtering, downsampling, and 
 * heterodyning
 **********************************************************************/
void recv_mix_to_buffer(
    uhd::usrp::multi_usrp::sptr usrp,
    uhd::rx_streamer::sptr rx_stream,
    std::vector<std::complex<short> *> recv_buffs,
    size_t num_requested_samples,
    float frequency,
    float sample_rate,
    uhd::time_spec_t start_time,
    int *return_status
){
    
    unsigned int osr = (int) ceil(frequency) / sample_rate;
    int num_total_samps = 0;
    int num_usrp_samples = (int) (num_requested_samples*osr);
    int ntaps = osr;
    std::vector<std::complex<double> > filter_taps(ntaps, 1.0/ntaps); //just a moving-average filter
    //float alpha = 32*9.86 / (0.8328*osr);
    //std::cout << "alpha: " << alpha << std::endl;
    //for (int i=0; i<ntaps; i++){
    //        filter_taps[i] = pow(alpha/3.14,0.5)*pow(2.7183, 
    //                -1*(alpha)*pow((((float)i-(ntaps-1)/2)/ntaps),2)) / ntaps;
    //}

    std::vector<std::complex<double> > buff0(osr);
    std::vector<std::complex<double> > buff1(osr);
    std::vector<std::complex<double> *> buffs(1,&buff0.front());
    buffs.push_back(&buff1.front());
    std::vector<std::complex<double> > circular_buff0;
    std::complex<double> temp(0,0);
    int ibuff = 0;
    int inxbuff = 0;

    uhd::rx_metadata_t md;
    //std::vector<samp_type> buff(samps_per_buff);
    float timeout = 0.1;

    //setup streaming
    num_requested_samples=(int)num_requested_samples;
    std::cout << "recv_to_buffer: num_requested_samples: " << num_requested_samples << std::endl;
    uhd::stream_cmd_t stream_cmd = uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE;
    stream_cmd.num_samps = num_usrp_samples;
    stream_cmd.stream_now = false;
    std::cout << 22./usrp->get_rx_rate() << "\n";
    stream_cmd.time_spec = start_time;//+1.5*300e-6-22./usrp->get_rx_rate(); //Start a little early to account for hw delay
    //std::cout << boost::format("Rx start time: %d") % start_time.get_real_secs() << std::endl;

    usrp->issue_stream_cmd(stream_cmd);
    md.error_code = uhd::rx_metadata_t::ERROR_CODE_NONE;
uhd::time_spec_t now_time = usrp->get_time_now();
while(num_total_samps < num_usrp_samples){
        size_t num_rx_samps = rx_stream->recv(buffs, osr, md, timeout);
	now_time=usrp->get_time_now();
	if (num_total_samps==0)
	std::cout << "Rx time: " << now_time.get_real_secs() <<
              " rx start time: " << stream_cmd.time_spec.get_real_secs() << std::endl;

	if (num_rx_samps != osr){
		uhd::time_spec_t rx_error_time = usrp->get_time_now();
		std::cerr << "Error in receiving samples..(" << rx_error_time.get_real_secs() << ")\t";;
		std::cerr << "Samples rx'ed: " << num_rx_samps << 
			" (expected " << osr << ")" << std::endl;
		std::cerr << "Total Samples rx'ed: " << num_total_samps << std::endl;
		num_rx_samps=osr;
		for (unsigned int i=0;i<osr;i++){
			buff0[i]=std::complex<double>(0,0);
			buff1[i]=std::complex<double>(0,0);
		}
	}
		
        inxbuff=num_total_samps%osr;
        // Place freshest samples from USRP into the circular buffer
	circular_buff0.resize(osr);
        for(unsigned int i=0;i<osr;i++){
                circular_buff0[(inxbuff+i)] = buff0[i];
        }

        ibuff+=1;
	//Filter and decimate
        temp=0;
        for(int i=0;i<ntaps-1;i++){
                temp+=filter_taps[i]*circular_buff0[(inxbuff+i)%ntaps];
        }

	//std::cout << temp << std::endl;
        recv_buffs[0][ibuff-1] =
                std::complex<int16_t>((int16_t)(16384*temp.real()),(int16_t)(16384*temp.imag()));

        if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT) {
	    uhd::time_spec_t rx_error_time = usrp->get_time_now();
            std::cerr << "Timeout encountered at " << rx_error_time.get_real_secs() << std::endl;
	    *return_status=-1;
	    break;
        }
        if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_OVERFLOW){
	    uhd::time_spec_t rx_error_time = usrp->get_time_now();
            std::cerr << "Overflow encountered at " << rx_error_time.get_real_secs() << std::endl;
	    *return_status=-1;
	    break;
        }
        if (md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE){
	    uhd::time_spec_t rx_error_time = usrp->get_time_now();
            std::cerr << "Unexpected error code " << md.error_code <<
		" encountered at " << rx_error_time.get_real_secs() << std::endl;
	    *return_status=-1;
	    break;
        }
    num_total_samps += num_rx_samps;
    }
    //for(int i=0;i<num_requested_samples;i++){
    //    std::cout << client_buff[i] << "\n";
    //}
    //return 0;
    std::cout << "Recv thread success! " << std::endl;
}

