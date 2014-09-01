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
int debug=0;

/***********************************************************************
 * tx_worker function
 **********************************************************************/
void tx_worker(
    uhd::tx_streamer::sptr tx_stream,
    std::vector<std::complex<short> *>& pulse_seq_ptrs,
    int sequence_length,
    uhd::time_spec_t start_time
){
    //setup the metadata flags to send the first buffer's worth of data
    uhd::tx_metadata_t md;
    md.start_of_burst = true;
    md.end_of_burst   = false;
    md.has_time_spec  = true;
    md.time_spec = start_time;

    //Initialize the temporary pointers according to the argument passed to the function
    std::vector<std::complex<int16_t> *> temp_ptrs(pulse_seq_ptrs.size());
    if (debug) std::cout << "num tx channels: " << tx_stream->get_num_channels() << std::endl;
    if (debug) std::cout << "pointer values: " << std::endl;
    for (int i=0; i<pulse_seq_ptrs.size(); i++){
        temp_ptrs[i] = pulse_seq_ptrs.at(i);
        //std::cout << temp_ptrs[i] << std::endl;
    }

    size_t nacc_samps = 0;
    //size_t spb = 10*tx_stream->get_max_num_samps();
    size_t spb = tx_stream->get_max_num_samps()/2;
    //size_t spb = 1000;

    //Now go for it!
    int ipacket =0;
    while(nacc_samps < sequence_length - spb){
        ipacket++;
        //std::cout << "about to transmit.. " << std::endl;
        if (debug) std::cout << "transmitting packet: " << ipacket << " of " << sequence_length / spb << std::endl;
        size_t ntx_samps = tx_stream->send(temp_ptrs, spb, md);
        if (ntx_samps != spb){
            std::cerr << "Error transmitting samples\n";
            std::cerr << "spb: " << spb << " ntx_samps: " << ntx_samps << std::endl;
            std::cerr << "ipacket: " << ipacket << " / " << sequence_length / spb << std::endl;
        }
        md.start_of_burst = false;
        md.has_time_spec = false;
        nacc_samps += ntx_samps;
        if (debug) std::cout << "nacc_samps: " << nacc_samps << " of " << sequence_length << std::endl;
        for (size_t i=0; i<pulse_seq_ptrs.size(); i++){ // Advance the pulse_seq pointers
            //temp_ptrs[i] = pulse_seq_ptrs[i] + nacc_samps;
            temp_ptrs[i] += spb;
        }
    }

    // Now we're on the last packet
    if (debug) std::cout << "transmitting last packet..\n";
    md.end_of_burst = true;
    spb = sequence_length - nacc_samps;
    //for (int i=0; i<2; i++){
    //    for (int j=0; j<spb; j++){
    //        std::cout << i << " " << j << " " << temp_ptrs[i][j] << std::endl;
    //    }
    //}
    if (debug) std::cout << "sequence length:" << sequence_length << " nacc_samps: " << nacc_samps << " spb: " << spb <<  std::endl;
    size_t ntx_samps = tx_stream->send(temp_ptrs, spb, md);
    if (ntx_samps != spb)
        std::cerr << "Error transmitting samples\n";
    if (debug) std::cout << "done transmitting\n";
        
}
