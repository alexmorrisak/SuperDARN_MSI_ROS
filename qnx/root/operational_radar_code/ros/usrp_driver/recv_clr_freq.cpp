#include <complex>
#include <vector>

#include <fftw3.h>
//Added by Alex for usrp
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/utils/thread_priority.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/utils/static.hpp>
#include <uhd/exception.hpp>


/***********************************************************************
 * recv_clr_freq function
 * A function to be used to receive samples from the USRP and
 * calculate a periodogram
 **********************************************************************/
int recv_clr_freq(
    uhd::usrp::multi_usrp::sptr usrp,
    uhd::rx_streamer::sptr rx_stream,
    int bandwidth,
    int naverages,
    double *pwr
){
    int num_total_samps = 0;
    int num_usrp_samples = naverages * bandwidth;

    std::vector<std::vector<std::complex<double> > > rx_short_vecs;
    std::vector<std::complex<double> *> rx_vec_ptrs;
    fftw_complex *in=NULL, *out=NULL;
    fftw_plan plan;
    double *tmp_pwr=NULL;
    double hann_window[bandwidth];

    for(int i=0;i<bandwidth;i++)
	hann_window[i] = 0.5*(1-cos(6.28*i/(bandwidth-1)));

    if (tmp_pwr!=NULL) free(tmp_pwr);
    tmp_pwr = (double*) calloc(bandwidth,sizeof(double));

    for(size_t i=0;i<usrp->get_rx_num_channels();i++)
	rx_short_vecs.push_back(std::vector<std::complex<double> >(bandwidth,0));
    for(size_t i=0;i<usrp->get_rx_num_channels();i++)
	rx_vec_ptrs.push_back(&rx_short_vecs[i].front());

    uhd::time_spec_t start_time = usrp->get_time_now() + 0.02;
    uhd::rx_metadata_t md;
    float timeout = 0.1;

    //setup streaming
    //usrp->set_rx_freq(1e3*center_freq);
    //usrp->set_rx_rate(bandwidth);
    uhd::stream_cmd_t stream_cmd = uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE;
    stream_cmd.num_samps = num_usrp_samples;
    stream_cmd.stream_now = false;
    stream_cmd.time_spec = start_time;
    usrp->issue_stream_cmd(stream_cmd);

    md.error_code = uhd::rx_metadata_t::ERROR_CODE_NONE;
    while(num_total_samps < num_usrp_samples){
        int num_rx_samps = rx_stream->recv(rx_vec_ptrs, bandwidth, md, timeout);

	//Check for errors
	if (num_rx_samps != bandwidth){
		uhd::time_spec_t rx_error_time = usrp->get_time_now();
		std::cerr << "Error in receiving samples..(" << rx_error_time.get_real_secs() << ")\t";;
		std::cerr << "Samples rx'ed: " << num_rx_samps << 
			" (expected " << bandwidth << ")" << std::endl;
		std::cerr << "Total Samples rx'ed: " << num_total_samps << std::endl;
		num_rx_samps=bandwidth;
		for (size_t i=0;i<usrp->get_rx_num_channels();i++){
			for (int j=0;j<bandwidth;j++){
				rx_short_vecs[i][j]=std::complex<int16_t>(0,0);
			}
		}
	}
        if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT) {
	    uhd::time_spec_t rx_error_time = usrp->get_time_now();
            std::cerr << "Timeout encountered at " << rx_error_time.get_real_secs() << std::endl;
	    return -1;
        }
        if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_OVERFLOW){
	    uhd::time_spec_t rx_error_time = usrp->get_time_now();
            std::cerr << "Overflow encountered at " << rx_error_time.get_real_secs() << std::endl;
	    return -1;
        }
        if (md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE){
	    uhd::time_spec_t rx_error_time = usrp->get_time_now();
            std::cerr << "Unexpected error code " << md.error_code <<
		" encountered at " << rx_error_time.get_real_secs() << std::endl;
	    return -1;
        }
	//Done checking for errors
		
	//Execute fft for each sample buffer
	if (in!=NULL) {free(in);in=NULL;}
	in = (fftw_complex*) fftw_malloc(sizeof(fftw_complex)*bandwidth);
	if (out!=NULL) {free(out);out=NULL;}
	out = (fftw_complex*) fftw_malloc(sizeof(fftw_complex)*bandwidth);

	plan = fftw_plan_dft_1d(bandwidth, in, out, FFTW_FORWARD, FFTW_ESTIMATE);

	for (int i=0;i<bandwidth;i++){
        	in[i][0] = (hann_window[i]*rx_short_vecs[0][i].real());
        	in[i][1] = (hann_window[i]*rx_short_vecs[0][i].imag());
        	//in[i][0] = rx_short_vecs[0][i].real();
        	//in[i][1] = rx_short_vecs[0][i].imag();
	}

	fftw_execute(plan);
	//Done executing fft
	
	//Add the current spectrum to the running total
	for (int i=0;i<bandwidth;i++){
        	tmp_pwr[i] += out[i][0]*out[i][0] + out[i][1]*out[i][1] / 
			double(bandwidth*bandwidth*naverages);
	}
	//Done adding spectrum to running total

    	num_total_samps += num_rx_samps;
	fftw_destroy_plan(plan);
	if (in!=NULL) {free(in); in=NULL;}
	if (out!=NULL) {free(out); out=NULL;}
    }
    
    //Center the fft (fftshift)
    for(int i=0;i<(bandwidth/2);i++){
    	pwr[bandwidth/2+i]=tmp_pwr[i];
    	pwr[i]=tmp_pwr[bandwidth/2+i];
    }   
    //Done centering

    if (tmp_pwr!=NULL) free(tmp_pwr);

    return 0;
}

