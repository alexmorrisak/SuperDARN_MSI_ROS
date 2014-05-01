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
 * interpolate_and_multiply() function
 * A function to be used to upsample/interpolate between two bb samples
 * and multiply by the input NCO vector(s).
 * Should be placed in its own thread context for parallelization
 **********************************************************************/
/***********************************************************************
 * TODO: this should be function should be broken down further such
 * that each thread gets its own center frequency, not just its own 
 * antenna channel. In total there should be NANTS*NFREQS threads.
 **********************************************************************/

void interpolate_and_multiply(
    std::complex<double> buff[2],
    const std::vector<std::vector<std::complex<double> > > *NCO,
    std::vector<std::vector<std::complex<float> > > *output,
    std::vector<float> pds,
    int ant,
    int ibb,
    size_t osr
){
    //cpu_set_t cpuset;
    //CPU_ZERO(&cpuset);
    //CPU_SET(icpu,&cpuset);
    //pthread_t current_thread=pthread_self();
    //pthread_setaffinity_np(current_thread,sizeof(cpu_set_t),&cpuset);

    int nfreqs = NCO->size();
    std::vector<std::complex<double> > inc(nfreqs,(buff[1]-buff[0]) / std::complex<double>(osr,0));
    std::complex<double> temp;// = std::complex<double>(0,0);

    /* radfreq_bb is a phase correction between baseband samples since there might be a 
    non-integer number of cycles in the span of an interpolation*/
    std::vector<double> radfreq_bb(nfreqs,0);
    for(int i=0;i<nfreqs;i++)
        radfreq_bb[i] = ibb * fmod(osr * (std::arg((*NCO)[i][1]) - std::arg((*NCO)[i][0])), 2*M_PI);

    std::vector<std::complex<double> > bb_accum(nfreqs,0);
    for (int j=0;j<nfreqs;j++){
	bb_accum[j] = pow(M_E,std::complex<double>(0,ant*pds[j]+radfreq_bb[j])) * buff[0];
	inc[j] *= pow(M_E,std::complex<double>(0,ant*pds[j]+radfreq_bb[j]));
    }

    //for(int i=0;i<nfreqs;i++){
    //    std::cout << "ibb: " << ibb << "\t";
    //    std::cout << "radfreq_bb: " << fmod(radfreq_bb[i], 2*M_PI) << "\t";
    //    std::cout << bb_accum[i] << "\t" << inc[i] << std::endl;
    //}

    for(size_t i=0;i<osr;i++){ //loop through rf samples
	temp = std::complex<double>(0,0);
    	for(int j=0;j<nfreqs;j++){  //loop through oscillator frequencies
		//if(bb_accum != std::complex<double>(0,0)) std::cout << bb_accum << "\n" << (*NCO)[j][i] << "\n";
        	temp += (*NCO)[j][i] * bb_accum[j];
    		bb_accum[j]+=inc[j];
	}
    (*output)[ant][ibb*osr+i] = temp;
    }
}


/***********************************************************************
 * tx_mix_upsample function
 * A function to be used to take a single vector of floats (based on 
 * the master tsg sequence), mix and upsample to the required
 * frequencies, and write to the antenna channel vectors pointed 
 * to by outputs_rf.
 **********************************************************************/
int tx_mix_upsample(
    std::vector<std::complex<float> >*input_bb,//input vector (same tx sequence for all antennas)
    std::vector<std::vector<std::complex<float> > > *outputs_rf, //output vectors (number of antennas)
    float bb_sample_rate, //rate of input samples
    float rf_sample_rate, //rate of samples from host cpu to usrp
    float usrp_center_freq,
    std::vector<float> center_freqs, //list of center frequencies to mix up (down) to
    float time_delay //time offset between each antenna in degrees
){
    //int nprocessors = sysconf(_SC_NPROCESSORS_ONLN);

    struct timeval t0,t1,rt0,rt1;
    gettimeofday(&rt0,NULL);
    double elapsed_time=0;

    int osr = (int) (rf_sample_rate / bb_sample_rate);
    //int nbb_samps=input_bb->size()+1;
    //int nrf_samps=osr*nbb_samps;
    int nants = outputs_rf->size();

    std::complex<double> temp[2];

    std::vector<double> radfreq; //a phase-shift-per-sample for each center frequency
    std::vector<std::vector<std::complex<double> > > NCO; //a vector for each center frequency
    std::vector<std::complex<double> > NCO_bb; //one for each center frequency
    std::vector<float> phase_delays; //phase delay per antenna for each center frequency

    for (size_t i=0;i<center_freqs.size();i++)
	radfreq.push_back(-2*M_PI*center_freqs[i]/rf_sample_rate);

    std::cout << "Phase delays: \n";
    for (size_t i=0;i<center_freqs.size();i++){
	//float pd = fmod((time_delay*(usrp_center_freq+center_freqs[i])*1e-9), 1.) * 2*M_PI;
	phase_delays.push_back(fmod((time_delay*(usrp_center_freq+center_freqs[i])*1e-9), 1.) * 2*M_PI);
	std::cout << "\t " << phase_delays[i];
    }
    std::cout << std::endl;
        //phase_delays.push_back(pd);
	
    for (size_t i=0;i<center_freqs.size();i++){
	NCO.push_back(std::vector<std::complex<double> >(osr,1));
	//NCO_bb.push_back(std::vector<std::complex<double> >(nbb_samps,1));
    }
    for (size_t i=0;i<center_freqs.size();i++){
    	for (int j=0;j<osr;j++){
		NCO[i][j] = pow(M_E,std::complex<double>(0,-j*radfreq[i]));
		//std::cout << NCO[i][j] << "\t";
		//NCO_bb.push_back(std::complex<double> >(nbb_samps,1));
	}
    }

    //sched_param sch;
    //int policy;
    //std::thread filter_threads[NTHREADS];
    //for (int i=0;i<NTHREADS;i++){
    //    sch.sched_priority = 99;
    //	pthread_setschedparam(filter_threads[i].native_handle(),SCHED_FIFO,&sch);//){
    //}
	
   //temp[0]=(*input_bb)[i] * NCO_bb[i][j];
   //temp[1]=(*input_bb)[i+1];
   //input_bb->push_back(std::complex<float>(0,0));
   for(size_t i=0;i<input_bb->size()-1;i++){

	gettimeofday(&t0,NULL);

	temp[0]=(*input_bb)[i];
	temp[1]=(*input_bb)[i+1];
	
    	for(int iant=0;iant<nants;iant++){
		//filter_threads[i] = std::thread(interpolate_and_multiply,
		//	temp,
		//	&NCO,
		//	&outputs_rf[j][i*osr],
		//	osr);
		//for (int i=0;i<2;i++)
		//	temp[i] *= std::exp(std::complex<double>(0,iant*));
		interpolate_and_multiply(
			temp,
			&NCO,
			outputs_rf,
			phase_delays,
			iant,
			i,
			osr);
	}
	
    	//for(int i=0;i<nants;i++)
	//	filter_threads[i].join();
	gettimeofday(&t1,NULL);
	elapsed_time += 1e6*(t1.tv_sec-t0.tv_sec)+(t1.tv_usec-t0.tv_usec);
	
    }
    gettimeofday(&rt1,NULL); 
    if(verbose > -1){
    	std::cout << "interpolate_and_multiply() elapsed time: " << elapsed_time << " usec" << std::endl;
    	std::cout << "upsample elapsed time: " << 1e6*(rt1.tv_sec-rt0.tv_sec)+(rt1.tv_usec-rt0.tv_usec) << " usec" << std::endl;
    }
    return 0;
}

