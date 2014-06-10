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
 * transmit_worker function
 **********************************************************************/
void transmit_worker(
    uhd::tx_streamer::sptr tx_stream,
    std::vector<std::complex<short> *> pulse_seq_ptrs,
    int sequence_length,
    uhd::time_spec_t start_time
){
    //setup the metadata flags to send the first buffer's worth of data
    uhd::tx_metadata_t md;
    md.start_of_burst = true;
    md.end_of_burst   = false;
    md.has_time_spec  = true;
    md.time_spec = start_time;
    //for (int i=0; i<pulse_seq_ptrs.size(); i++)
    //    pulse_seq_ptrs[i] += 2000;
    //sequence_length -= 2000;
    std::vector<std::complex<short> *> temp_ptrs = pulse_seq_ptrs;

    size_t nacc_samps = 0;
    size_t spb = tx_stream->get_max_num_samps()/2;

    //Now go for it!
    while(nacc_samps < sequence_length - spb){
        size_t ntx_samps = tx_stream->send(temp_ptrs, spb, md);
        if (ntx_samps != spb)
            std::cerr << "Error transmitting samples\n";
        md.start_of_burst = false;
        md.has_time_spec = false;
        nacc_samps += ntx_samps;
        //std::cout << "nacc_samps: " << nacc_samps <<std::endl;
        for (size_t i=0; i<pulse_seq_ptrs.size(); i++){ // Advance the pulse_seq pointer
            //temp_ptrs[i] = pulse_seq_ptrs[i] + nacc_samps;
            temp_ptrs[i] += spb;
        }
    }
    // Now we're on the last packet
    md.end_of_burst = true;
    spb = sequence_length - nacc_samps;
    size_t ntx_samps = tx_stream->send(temp_ptrs, spb, md);
    if (ntx_samps != spb)
        std::cerr << "Error transmitting samples\n";
        
}
