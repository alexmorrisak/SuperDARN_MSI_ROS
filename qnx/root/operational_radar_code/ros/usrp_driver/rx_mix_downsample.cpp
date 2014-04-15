#include <complex>
#include <vector>
//Added by Alex for usrp
//#include <uhd/usrp/multi_usrp.hpp>
//#include <uhd/utils/thread_priority.hpp>
//#include <uhd/utils/safe_main.hpp>
//#include <uhd/utils/static.hpp>
//#include <uhd/exception.hpp>
#include <boost/thread.hpp>
#include <thread>
#include <math.h>

#define NTHREADS 2

extern int verbose;

/***********************************************************************
 * multiply_and_add() function
 * A function to be used to convolve filter taps with input data and
 * decimate.  Should be placed in its own thread context for 
 * parallelization
***********************************************************************/
/***********************************************************************
<<<<<<< HEAD
 * TODO: this should be function should be broken down further such
=======
 * TODO: this function should be broken down further such
>>>>>>> d384cc40c0b48a95bcfd4623fcbc3af820bb80d1
 * that each thread gets its own center frequency, not just its own 
 * antenna channel. There should be NANTS*NFREQS threads.
 * Think memory performance: this means that the received 
 * samples in one buffer must be shared between multiple threads.
 **********************************************************************/
void multiply_and_add(
    const std::complex<int16_t> *buff,
<<<<<<< HEAD
    const std::vector<std::vector<std::complex<double> > > *filter_taps,
    std::vector<std::complex<double> > *temp,
    size_t ntaps,
    int icpu
){
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(icpu,&cpuset);
    pthread_t current_thread=pthread_self();
    pthread_setaffinity_np(current_thread,sizeof(cpu_set_t),&cpuset);
    std::complex<double> dtemp;

    for(size_t i=0;i<filter_taps->size();i++){ //loop through center frequencies
    	std::complex<double> tlocal_temp = std::complex<double>(0,0);
    	for(size_t j=0;j<ntaps;j++){  //loop through filter coefficients
		//dtemp = std::complex<double>((buff+j)->real() / 16384., (buff+j)->imag() / 16384);
		dtemp = std::complex<double>((float)(buff+j)->real() / 16384, (float)(buff+j)->imag()/16384);
		//dtemp = std::complex<double>((float)(buff+j)->real(), (float)(buff+j)->imag());
        	tlocal_temp += (*filter_taps)[i][j] * dtemp;
    		//std::cout << dtemp.real() << "\t" << dtemp.imag() << std::endl;
	}
    (*temp)[i]=tlocal_temp;
    //(*temp)[i]=dtemp;
    }
=======
    const std::vector<std::vector<std::complex<float> > > *filter_taps,
    std::vector<std::complex<float> > *temp,
    size_t ntaps,
    int icpu
){
    //cpu_set_t cpuset;
    //CPU_ZERO(&cpuset);
    //CPU_SET(icpu,&cpuset);
    //pthread_t current_thread=pthread_self();
    //pthread_setaffinity_np(current_thread,sizeof(cpu_set_t),&cpuset);
    std::complex<float> dtemp;

    for(size_t i=0;i<filter_taps->size();i++){ //loop through center frequencies
    	std::complex<float> tlocal_temp = std::complex<float>(0,0);
    	for(size_t j=0;j<ntaps;j++){  //loop through filter coefficients
        	dtemp = std::complex<float>((float)(buff+j)->real() / 16384, (float)(buff+j)->imag() / 16384);
        	tlocal_temp += (*filter_taps)[i][j] * dtemp;
        }
    (*temp)[i]=tlocal_temp;
    }
    //std::vector<std::complex<float> > tlocal_temp(filter_taps->size(),0);
    //for(size_t itap=0;itap<ntaps;itap++){  //loop through filter coefficients
    //    dtemp = std::complex<float>((float)(buff+itap)->real() / 16384, (float)(buff+itap)->imag() / 16384);
    //	for(size_t ifreq=0;ifreq<filter_taps->size();ifreq++){ //loop through center frequencies
    //    	tlocal_temp[ifreq] += (*filter_taps)[ifreq][itap] * dtemp;
    //    }
    //}
    //for (size_t ifreq=0;ifreq<filter_taps->size();ifreq++)
    //	(*temp)[ifreq]=tlocal_temp[ifreq];
>>>>>>> d384cc40c0b48a95bcfd4623fcbc3af820bb80d1
}


/***********************************************************************
 * rx_mix_downsample() function
 * A function to be used to process the entire sequence of samples and
 * create baseband samples as requested by the client. Performs 
 * filtering, mixing, samplerate conversion
 **********************************************************************/
<<<<<<< HEAD
int rx_mix_downsample(
    std::vector<std::complex<short> *> rx_buff_ptrs,
=======
void rx_mix_downsample(
    std::vector<std::complex<int16_t> *> rx_buff_ptrs,
>>>>>>> d384cc40c0b48a95bcfd4623fcbc3af820bb80d1
    std::vector<std::complex<float> *> client_buff_ptrs,
    size_t nrf_samples,
    size_t nbb_samples,
    float rf_sample_rate,
    float client_sample_rate,
    std::vector<float> center_freqs, // center frequencies
    std::vector<float> bws // bandwidth of each center frequency
){
    struct timeval t0,t1,rt0,rt1;
    gettimeofday(&rt0,NULL);
    double elapsed_time=0;

    int osr = (int) rf_sample_rate / client_sample_rate;
<<<<<<< HEAD
    std::cout << "osr: " << osr << std::endl;
    //int nsamples=(int) nrf_samples/osr;
    int nsamples= nbb_samples;
    std::cout << "nsamples: " << nsamples << std::endl;
    //std::cout << "vec size: " << 

    int nchannels = rx_buff_ptrs.size();

=======
    int nsamples= nbb_samples;

    int nchannels = rx_buff_ptrs.size();

    if (verbose > 2) std::cout << "nsamples: " << nsamples << "\nosr: " << osr << std::endl;

>>>>>>> d384cc40c0b48a95bcfd4623fcbc3af820bb80d1
    int ntaps = 2*osr; //The filter length is 2x the decimation rate

    std::vector<std::complex<int16_t> *> buff_ptrs;

<<<<<<< HEAD
    std::vector<std::vector<std::complex<double> > > filter_taps;
    std::vector<std::complex<double> > NCO;
    std::vector<std::complex<double> > NCO_bb;

    for (size_t i=0;i<center_freqs.size();i++){
	filter_taps.push_back(std::vector<std::complex<double> >(ntaps,1/ntaps)); //just a moving-average filter;
	NCO.push_back(std::complex<double>(1,0));
	NCO_bb.push_back(std::complex<double>(1,0));
=======
    std::vector<std::vector<std::complex<float> > > filter_taps;
    std::vector<std::complex<float> > NCO;
    std::vector<std::complex<float> > NCO_bb;

    for (size_t i=0;i<center_freqs.size();i++){
	filter_taps.push_back(std::vector<std::complex<float> >(ntaps,1/ntaps)); //just a moving-average filter;
	NCO.push_back(std::complex<float>(1,0));
	NCO_bb.push_back(std::complex<float>(1,0));
>>>>>>> d384cc40c0b48a95bcfd4623fcbc3af820bb80d1
    }
    int nprocessors = sysconf(_SC_NPROCESSORS_ONLN);
	
    /*Calculate the per-sample phase shift of each NCO*/
<<<<<<< HEAD
    std::vector<std::complex<double> > radfreq;
    for (size_t i=0;i<center_freqs.size();i++)
	radfreq.push_back(std::complex<double>(0,2*M_PI*center_freqs[i]/rf_sample_rate));
    std::vector<std::complex<double> > radfreq_bb;
    for (size_t i=0;i<center_freqs.size();i++)
    	radfreq_bb.push_back(std::complex<double>(0,fmod(osr*radfreq[i].imag(),2*M_PI)));//*fmod(center_freqs[0]*osr,1.)));
    	//radfreq_bb.push_back(std::complex<double>(0,0));
=======
    std::vector<std::complex<float> > radfreq;
    for (size_t i=0;i<center_freqs.size();i++)
	radfreq.push_back(std::complex<float>(0,2*M_PI*center_freqs[i]/rf_sample_rate));
    std::vector<std::complex<float> > radfreq_bb;
    for (size_t i=0;i<center_freqs.size();i++)
    	radfreq_bb.push_back(std::complex<float>(0,fmod(osr*radfreq[i].imag(),2*M_PI)));//*fmod(center_freqs[0]*osr,1.)));
    	//radfreq_bb.push_back(std::complex<float>(0,0));
>>>>>>> d384cc40c0b48a95bcfd4623fcbc3af820bb80d1

    /*Hamming-windowed sinc. Right now it's same for all center frequencies, 
    but it should take into account the requested bw*/
    for (int i=0;i<ntaps;i++){
<<<<<<< HEAD
            filter_taps[0][i] = 4*((0.54-0.46*cos((2*M_PI*(double)(i))/ntaps))
            	*(sin(2*M_PI*(double)(i-ntaps/2)/ntaps) / (2*M_PI*(double)(i-ntaps/2)/ntaps)))/ ntaps;
=======
            filter_taps[0][i] = 4*((0.54-0.46*cos((2*M_PI*(float)(i))/ntaps))
            	*(sin(2*M_PI*(float)(i-ntaps/2)/ntaps) / (2*M_PI*(float)(i-ntaps/2)/ntaps)))/ ntaps;
>>>>>>> d384cc40c0b48a95bcfd4623fcbc3af820bb80d1
    }

    filter_taps[0][ntaps/2]=4./ntaps;

    for (size_t i=1;i<center_freqs.size();i++)
    	filter_taps[1] = filter_taps[0];

    /*Mix each filter up(down) to the desired pass-band frequency*/
    for (size_t i=0;i<center_freqs.size();i++){
    	for (int j=0;j<ntaps;j++){
    	        NCO[i] *= pow(M_E,-radfreq[i]);
    	        filter_taps[i][j] *= NCO[i];
	}
    }

<<<<<<< HEAD
    std::vector<std::vector<std::complex<double> > > temp;
    //for(size_t i=0;i<center_freqs.size();i++){
    for(int i=0;i<nchannels;i++){
	temp.push_back(std::vector<std::complex<double> >(center_freqs.size(),0));
=======
    std::vector<std::vector<std::complex<float> > > temp;
    //for(size_t i=0;i<center_freqs.size();i++){
    for(int i=0;i<nchannels;i++){
	temp.push_back(std::vector<std::complex<float> >(center_freqs.size(),0));
>>>>>>> d384cc40c0b48a95bcfd4623fcbc3af820bb80d1
    }

    sched_param sch;
    //int policy;
    std::thread filter_threads[NTHREADS];
    for (int i=0;i<NTHREADS;i++){
<<<<<<< HEAD
  	sch.sched_priority = 99;
=======
        sch.sched_priority = 99;
>>>>>>> d384cc40c0b48a95bcfd4623fcbc3af820bb80d1
    	pthread_setschedparam(filter_threads[i].native_handle(),SCHED_FIFO,&sch);//){
    }

    //while(num_total_samps < num_usrp_samples){
    for(int isample=0;isample<nsamples;isample++){

	buff_ptrs.clear();
	for (int i=0; i<nchannels; i++)
		buff_ptrs.push_back(rx_buff_ptrs[i] + osr*isample);
<<<<<<< HEAD
	//std::cout << isample << ": " << *(buff_ptrs[0]) << std::endl;
=======
>>>>>>> d384cc40c0b48a95bcfd4623fcbc3af820bb80d1

	//Filter and decimate
	for(size_t i=0;i<center_freqs.size();i++){
    		for(int j=0;j<nchannels;j++)
<<<<<<< HEAD
			temp[j][i]=std::complex<double>(0,0);
	}

    	//for(size_t i=0;i<nchannels;i++){
=======
			temp[j][i]=std::complex<float>(0,0);
	}

    	//for(size_t i=0;i<nchannels;i++){
    	if (verbose > 2) std::cout << "isample: " << isample << "\n";
>>>>>>> d384cc40c0b48a95bcfd4623fcbc3af820bb80d1
	gettimeofday(&t0,NULL);
    	for(int i=0;i<nchannels;i++){
		filter_threads[i] = std::thread(multiply_and_add,
			buff_ptrs[i],
			&filter_taps,
			&temp[i],
			ntaps,
			i%nprocessors);
		//multiply_and_add(
<<<<<<< HEAD
		//	&circ_buff[i%2],
		//	&filter_taps,
		//	&temp[i%2],
		//	ntaps);
=======
		//	buff_ptrs[i],
		//	&filter_taps,
		//	&temp[i],
		//	ntaps,
		//	i%nprocessors);
>>>>>>> d384cc40c0b48a95bcfd4623fcbc3af820bb80d1
	}
	
    	for(int i=0;i<nchannels;i++)
		filter_threads[i].join();
	gettimeofday(&t1,NULL);
	elapsed_time += 1e6*(t1.tv_sec-t0.tv_sec)+(t1.tv_usec-t0.tv_usec);
	
	/*There is some residual frequency offset after filtering/downsampling.
	Now mix the downsampled signal to zero frequency.*/
    	for (size_t i=0;i<center_freqs.size();i++){
		NCO_bb[i] *= pow(M_E,-radfreq_bb[i]);
		for(int j=0;j<nchannels;j++)
			temp[j][i] *= NCO_bb[i];
	}

<<<<<<< HEAD
	/*Write the newest sample into the client buffer*/
	for(size_t i=0;i<center_freqs.size();i++){
        	*(client_buff_ptrs[0]+isample) = temp[0][i];
=======
	/*Write the output samples into the client buffer*/
	for(size_t i=0;i<center_freqs.size();i++){
        	*(client_buff_ptrs[i]+isample) = temp[0][i];
>>>>>>> d384cc40c0b48a95bcfd4623fcbc3af820bb80d1
               		//std::complex<float>(nt16_t)(temp[0][i].real()),(int16_t)(16384*temp[0][i].imag()));
	}

    	gettimeofday(&rt1,NULL); 
    }
    if(verbose > -1){
    	std::cout << "filter elapsed time: " << elapsed_time << " usec" << std::endl;
    	std::cout << "rx_mix_downsample() elapsed time: " << 1e6*(rt1.tv_sec-rt0.tv_sec)+(rt1.tv_usec-rt0.tv_usec) << " usec" << std::endl;
<<<<<<< HEAD
    	//std::cout << "Expected time: " << 300.*(float)num_requested_samples << std::endl;
    }
    return 0;
=======
    }
>>>>>>> d384cc40c0b48a95bcfd4623fcbc3af820bb80d1
}

