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

#define NTHREADS 2

extern int verbose;

/***********************************************************************
 * multiply_and_add() function
 * A function to be used to convolve filter taps with input data and
 * decimate.  Should be placed in its own thread context for 
 * parallelization
 **********************************************************************/
void multiply_and_add(
    const std::vector<std::complex<double> > *buff,
    const std::vector<std::vector<std::complex<double> > > *filter_taps,
    std::vector<std::complex<double> > *temp,
    int inxbuff,
    size_t ntaps,
    int icpu
){
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(icpu,&cpuset);
    pthread_t current_thread=pthread_self();
    pthread_setaffinity_np(current_thread,sizeof(cpu_set_t),&cpuset);

    for(size_t i=0;i<filter_taps->size();i++){ //loop through center frequencies
    	std::complex<double> tlocal_temp = std::complex<double>(0,0);
    	for(size_t j=inxbuff;j<ntaps+inxbuff;j++)  //loop through filter coefficients
        	tlocal_temp += (*filter_taps)[i][j-inxbuff] * (*buff)[j%ntaps];
    (*temp)[i]=tlocal_temp;
    }
}


/***********************************************************************
 * recv_to_buffer function
 * A function to be used to receive samples from the USRP and
 * write them into a buffer.  Also performs filtering, mixing,  
 * samplerate conversion
 **********************************************************************/
void recv_to_buffer(
    uhd::usrp::multi_usrp::sptr usrp,
    uhd::rx_streamer::sptr rx_stream,
    std::vector<std::vector<std::complex<short> > *> client_buff_ptr,
    size_t num_requested_samples,
    float rf_sample_rate,
    float client_sample_rate,
    std::vector<float> center_freqs,
    uhd::time_spec_t start_time,
    int *return_status
){
    struct timeval t0,t1,rt0,rt1;
    gettimeofday(&rt0,NULL);
    double elapsed_time=0;
    int isample=0,num_total_samps=0,inxbuff=0;
    int osr = (int) rf_sample_rate / client_sample_rate;
    int nchannels = usrp->get_rx_num_channels();
    int num_usrp_samples = (int) (num_requested_samples*osr);
    int ntaps = 2*osr; //The filter length is 2x the decimation rate
    std::vector<std::vector<std::complex<double> > > filter_taps;
    std::vector<std::complex<double> > NCO;
    std::vector<std::complex<double> > NCO_bb;
    for (size_t i=0;i<center_freqs.size();i++){
	filter_taps.push_back(std::vector<std::complex<double> >(ntaps,1/ntaps)); //just a moving-average filter;
	NCO.push_back(std::complex<double>(1,0));
	NCO_bb.push_back(std::complex<double>(1,0));
    }
    int nprocessors = sysconf(_SC_NPROCESSORS_ONLN);
	
    /*Use 1 MHz for example client freq. In practice this should be a vector/array of values that the client requests
    for receiving.  Each frequency gets its own thread*/
    //double client_freq = 0; 
    std::vector<std::complex<double> > radfreq;
    for (int i=0;i<center_freqs.size();i++)
	radfreq.push_back(std::complex<double>(0,6.28*center_freqs[i]/rf_sample_rate));
    std::vector<std::complex<double> > radfreq_bb;
    for (int i=0;i<center_freqs.size();i++)
    	//radfreq_bb.push_back(std::complex<double>(0,6.28*fmod(center_freqs[0]*osr,1.)));
    	radfreq_bb.push_back(std::complex<double>(0,0));

    /*Hamming-windowed sinc. Same for all center frequencies*/
    for (int i=0;i<ntaps;i++){
            filter_taps[0][i] = 4*((0.54-0.46*cos((2*3.14*(double)(i))/ntaps))
            	*(sin(4*3.14*(double)(i-ntaps/2)/ntaps) / (4*3.14*(double)(i-ntaps/2)/ntaps)))/ ntaps;
    }
    filter_taps[0][ntaps/2]=4./ntaps;
    for (int i=1;i<center_freqs.size();i++)
    	filter_taps[1] = filter_taps[0];
    /*Mix the filter up(down) to the desired pass-band frequency*/
    /*Generally need two or more filters to rx multiple frequencies, but can
    perhaps use symmetry to kill two birds with one stone*/
    for (size_t i=0;i<center_freqs.size();i++){
    	for (int j=0;j<ntaps;j++){
    	        NCO[i] *= pow(2.718,-radfreq[i]);
    	        filter_taps[i][j] *= NCO[i];
	}
    }

    std::vector<std::vector<std::complex<double> > > circ_buff;
    for(int i=0;i<nchannels;i++)
	    circ_buff.push_back(std::vector<std::complex<double> > (ntaps,0));
    std::vector<std::complex<double> *> buff_ptrs;
    for(int i=0;i<nchannels;i++)
    	buff_ptrs.push_back(&circ_buff[i].front());
    std::vector<std::vector<std::complex<double> > > temp;
    //for(size_t i=0;i<center_freqs.size();i++){
    for(size_t i=0;i<nchannels;i++){
	temp.push_back(std::vector<std::complex<double> >(center_freqs.size(),0));
    }

    sched_param sch;
    //int policy;
    std::thread filter_threads[NTHREADS];
    for (int i=0;i<NTHREADS;i++){
  	sch.sched_priority = 99;
    	pthread_setschedparam(filter_threads[i].native_handle(),SCHED_FIFO,&sch);//){
    }
	

    //setup streaming
    uhd::rx_metadata_t md;
    float timeout = 0.1;
    num_requested_samples=(int)num_requested_samples;
    uhd::stream_cmd_t stream_cmd = uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE;
    stream_cmd.num_samps = num_usrp_samples;
    stream_cmd.stream_now = false;
    stream_cmd.time_spec = start_time;

    usrp->issue_stream_cmd(stream_cmd);
    md.error_code = uhd::rx_metadata_t::ERROR_CODE_NONE;
    while(num_total_samps < num_usrp_samples){
        size_t num_rx_samps = rx_stream->recv(buff_ptrs, osr, md, timeout);
	if (num_rx_samps != osr){
		uhd::time_spec_t rx_error_time = usrp->get_time_now();
		std::cerr << "Error in receiving samples..(" << rx_error_time.get_real_secs() << ")\t";;
		std::cerr << "Samples rx'ed: " << num_rx_samps << 
			" (expected " << osr << ")" << std::endl;
		std::cerr << "Total Samples rx'ed: " << num_total_samps << std::endl;
		num_rx_samps=osr;
		for(int i=0;i<nchannels;i++){
			for (int j=0;j<ntaps;j++)
				circ_buff[i][j]=std::complex<double>(0,0);
		}
	}

	//Filter and decimate
	for(size_t i=0;i<center_freqs.size();i++){
    		for(int j=0;j<nchannels;j++)
			temp[j][i]=std::complex<double>(0,0);
	}

    	//for(size_t i=0;i<nchannels;i++){
	gettimeofday(&t0,NULL);
    	for(int i=0;i<nchannels;i++){
		filter_threads[i] = std::thread(multiply_and_add,
			&circ_buff[i],
			&filter_taps,
			&temp[i],
			(inxbuff+osr)%ntaps,
			ntaps,
			i%nprocessors);
		//multiply_and_add(
		//	&circ_buff[i%2],
		//	&filter_taps,
		//	&temp[i%2],
		//	ntaps);
	}
	
    	for(size_t i=0;i<nchannels;i++)
		filter_threads[i].join();
	gettimeofday(&t1,NULL);
	elapsed_time += 1e6*(t1.tv_sec-t0.tv_sec)+(t1.tv_usec-t0.tv_usec);
	
	/*There is some residual frequency offset after filtering/downsampling.
	Now mix the downsampled signal to zero frequency.*/
    	for (size_t i=0;i<center_freqs.size();i++){
		NCO_bb[i] *= pow(2.718,-radfreq_bb[i]);
		for(int j=0;j<nchannels;j++)
			temp[j][i] *= NCO_bb[i];
	}

	/*Write the newest sample into the client buffer*/
	for(size_t i=0;i<center_freqs.size();i++){
        	(*client_buff_ptr[i])[isample] =
               		std::complex<int16_t>((int16_t)(16384*temp[0][i].real()),(int16_t)(16384*temp[0][i].imag()));
	}

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

    /*Change where the swing buffers point to*/
    isample+=1;
    inxbuff = (isample*osr)%ntaps;
    num_total_samps += num_rx_samps;
    //std::cout << (isample*osr)%ntaps << std::endl;
    for(int i=0;i<nchannels;i++)
   	buff_ptrs[i] = &circ_buff[i][inxbuff];
    }

    gettimeofday(&rt1,NULL); 
    if(verbose > -1){
    	std::cout << "filter elapsed time: " << elapsed_time << " usec" << std::endl;
    	std::cout << "recv_to_buff() elapsed time: " << 1e6*(rt1.tv_sec-rt0.tv_sec)+(rt1.tv_usec-rt0.tv_usec) << " usec" << std::endl;
    	std::cout << "Expected time: " << 300.*(float)num_requested_samples << std::endl;
    }
}

