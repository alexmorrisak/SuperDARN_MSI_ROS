#include <stdlib.h>
#include <vector>
#include <complex>
#include <iostream>
//#include <txrx_data.hpp>
#include <control_program.h>
#include <timing_defs.h>
extern int verbose;

#define X_BIT 0x04 //Transmit bit
#define TR_BIT 0x02 //TR bit
#define S_BIT 0x80 //Scope sync
#define P_BIT 0x10 //Phase bit.  0 for 0 degrees, 1 for 180 degrees
#define MAX_PULSES 100 

class tx_data{
    struct TSGbuf pulseseqs[4];
    std::vector<unsigned char> pulseseq_reps[4];
    std::vector<unsigned char> pulseseq_codes[4];

    std::vector<uint32_t> tr_times_starts;
    std::vector<uint32_t> tr_times_durations;

    int old_index;
    std::vector<unsigned char> seq_buf[4];

    std::vector<std::complex<float> > bb_vec;

    struct ControlPRM client;
    std::vector<ControlPRM> clients;

    size_t nclients; //number of clients registered to the usrp driver
    std::vector<std::vector<std::complex<float> > > tx_bb_vecs; //baseband data, tx_bb_vecs[nradars][nbbsamps]
    float bb_rate, rf_rate; //baseband and rf sample rates
    std::vector<std::vector<std::complex<int16_t> > > tx_rf_vecs; //rf data, one vector for each antenna
    std::vector<int16_t*> tx_rf_fronts;
    std::vector<std::vector<float> > tx_freqs; //Vectors of transmit frequencies, one vector for each radar
    std::vector<std::vector<float> > time_delays; //Time delay between two adjacent transmit antennas. One vector for each radar
    size_t num_ants, num_radars, num_ants_per_radar;
    std::vector<size_t> radmap, chanmap;
    float bb_samp_rate, rf_samp_rate;
    float center_freq;
    float samp_rate;

    public:
    tx_data(size_t nradars, size_t nants, float center_freq, float rf_samp_rate);
    ~tx_data();
    void ready_client(struct ControlPRM* client);

    struct TSGbuf* get_tsg_ptr(size_t index);
    void allocate_pulseseq_mem(size_t index);

    void unpack_pulseseq(size_t index);
    size_t get_seqbuf_len(size_t index);

    void make_bb_vecs(int32_t trise);
    void make_tr_times(struct TRTimes* tr_times);
    void allocate_rf_vec_mem();
    //void add_freq(size_t radar, size_t channel, float freq);
    void clear_channel_list();

    size_t get_num_ants(); //Get number of antennas
    size_t get_num_ants_per_radar();
    size_t get_num_radars(); //Get number of radars
    size_t get_num_channels(); //Get total number of clients for all radars
    size_t get_num_channels(size_t radar); //Get number of clients for that radar
    size_t get_num_bb_samples(); //Get number of bb samples.  It is and must be the same for all radar channels!
    size_t get_num_rf_samples(); //Get number of rf samples.  It is and must be the same for all antenna channels!
    float* get_bb_vec_ptr(size_t radar);
    void zero_rf_vec(size_t radar); //Allocate memory for rf vectors
    void set_rf_vec_ptrs(std::vector<std::complex<int16_t> *>* rf_vec_ptrs);
    int16_t** get_rf_vec_ptrs(size_t radar);
    float* get_freqs(size_t radar);
    float* get_time_delays(size_t radar);
};

tx_data::tx_data(size_t nradars, size_t nants, float center_freq, float rf_samp_rate){
    num_ants = nants;
    num_radars = nradars;
    num_ants_per_radar = nants/nradars;
    tx_bb_vecs.resize(nradars);
    tx_freqs.resize(nradars);
    time_delays.resize(nradars);
    tx_rf_vecs.resize(nants);

    samp_rate = rf_samp_rate;
    old_index = -10;
}

tx_data::~tx_data(){} //Don't need to free memory here--trust that memory allocated to vectors is freed when the vectors fall out of scope

void tx_data::ready_client(struct ControlPRM* client){
    tx_freqs[client->radar-1].push_back(1e3*client->tfreq);
    time_delays[client->radar-1].push_back(10*(16/2-client->tbeam)); //10 ns per antenna per beam
}

struct TSGbuf* tx_data::get_tsg_ptr(size_t index){
    return &pulseseqs[index];
}

void tx_data::allocate_pulseseq_mem(size_t index){
    pulseseq_reps[index].resize(pulseseqs[index].len);
    pulseseqs[index].rep = &pulseseq_reps[index].front();
    pulseseq_codes[index].resize(pulseseqs[index].len);
    pulseseqs[index].code = &pulseseq_codes[index].front();
}


void tx_data::unpack_pulseseq(size_t index){
    if (index!=old_index){
        int step=(int)((double)pulseseqs[index].step/(double)STATE_TIME+0.5);
                
        //If DDS or RX Offset is negative pad the seq_buf with the maximum negative offset
        //offset_pad=(int)((double)MIN(dds_offset,rx_offset)/((double)STATE_TIME+0.5))-2;
        int offset_pad=0;
        if (verbose > -1) std::cout << "offset pad: " << offset_pad << "\n";  
        for(int i=0;i>offset_pad;i--) {
          seq_buf[index].push_back(0);
        }
        for (int i=0; i<pulseseqs[index].len; i++){
            for (int j=0; j<step*pulseseqs[index].rep[i]; j++){
                seq_buf[index].push_back(pulseseqs[index].code[i]);
            }
        }
        old_index=index;
    }
}

size_t tx_data::get_seqbuf_len(size_t index){
    return seq_buf[index].size();
}

void tx_data::make_bb_vecs(int32_t trise){
    std::vector<float> taps((size_t)(25e3/trise), trise/25.e3/4);
    std::vector<std::complex<float> > rawsignal(seq_buf[old_index].size());

    bb_vec.resize(seq_buf[old_index].size(),0);

    for (size_t i=0; i<seq_buf[old_index].size(); i++) {
        if ((seq_buf[old_index][i] & X_BIT) == X_BIT){
            if ((seq_buf[old_index][i] & P_BIT) == P_BIT){
                rawsignal[i] = std::complex<float>(-1,0);
            }
            else {
                rawsignal[i] = std::complex<float>(1,0);
            }
        }
    }

    std::complex<float> temp;
    size_t signal_len = rawsignal.size();
    size_t taps_len = taps.size();

    /*Calculate taps for Gaussian filter. This is reference code for future use..*/
    //alpha = 32*(9.86/(2e-8*client.trise)) / (0.8328*usrp->get_rx_rate());
    ////std::cout << "alpha: " << alpha << std::endl;
    ////for (i=0; i<filter_table_len; i++){
    ////  filter_table[i] = pow(alpha/3.14,0.5)*pow(M_E, 
    ////      -1*(alpha)*pow((((float)i-(filter_table_len-1)/2)/filter_table_len),2))/filter_table_len;
    
    for (size_t i=0; i<taps_len/2; i++){
            temp = std::complex<float>(0,0);
            for(size_t j=0; j<taps_len/2+i; j++){
                    temp += rawsignal[i+j] * taps[taps_len/2-i+j];
            }
            //if (i %5 == 0 ) std::cout << i << " " << temp << std::endl;
            bb_vec[i] = temp;
    }
    
    for (size_t i=taps_len/2; i<signal_len-taps_len; i++){
            temp = std::complex<float>(0,0);
            for(size_t j=0; j<taps_len; j++){
                    temp += rawsignal[i+j] * taps[j];
            }
            bb_vec[i] = temp;
            //std::cout << i << " " << temp << std::endl;
    }

    for (size_t i=signal_len-taps_len; i<signal_len/2; i++){
            temp = std::complex<float>(0,0);
            for(size_t j=0; j<signal_len-i; j++){
                    temp += rawsignal[i+j] * taps[j];
            }
            bb_vec[i] = temp;
            //if (i 5 == 0 ) std::cout << i << " " << temp << std::endl;
    }
}

void tx_data::make_tr_times(struct TRTimes* tr_times){
    int tr_event = 0;
    tr_times->length = 0;
    tr_times_starts.clear();
    tr_times_durations.clear();
    for (size_t i=0; i<seq_buf[old_index].size(); i++){
        if ((seq_buf[old_index][i] & TR_BIT) == TR_BIT){
            if (tr_event==0){
                std::cout << "TR sample start: " << i << std::endl;
                tr_times->length++;
                if (tr_times->length > 0 && tr_times->length < MAX_PULSES){
                    tr_times_starts.push_back(i*STATE_TIME);
                    tr_times_durations.push_back(STATE_TIME);
                }
                else {
                    std::cerr << "Error in number of transmit pulses!\n\n";
                    exit(EXIT_FAILURE);
                }
            }
            else {
                tr_times_durations[tr_times->length-1] += STATE_TIME;
            }
            tr_event = 1;
        }
        else {
            if (tr_event == 1 && verbose > 1){
                std::cout << "TR sample end: " << i << std::endl;
            }
            tr_event = 0;
        }
    }
    tr_times->start_usec = &tr_times_starts.front();
    tr_times->duration_usec = &tr_times_durations.front();
}

void tx_data::allocate_rf_vec_mem(){
    int tx_osr = round(samp_rate*(float)STATE_TIME*1e-6);
    size_t nsamps = (tx_osr*seq_buf[old_index].size());
    for (size_t i=0; i<tx_rf_vecs.size(); i++){
        tx_rf_vecs[i].resize(nsamps);
    }
}


void tx_data::clear_channel_list(){
    // Clear the frequency list for each radar
    for (int irad=0; irad<tx_freqs.size(); irad++){
        tx_freqs[irad].clear();
        time_delays[irad].clear();
    }
}


size_t tx_data::get_num_ants(){
    return num_ants;
}

size_t tx_data::get_num_radars(){
    return num_radars;
}

size_t tx_data::get_num_ants_per_radar(){
    return num_ants_per_radar;
}

size_t tx_data::get_num_channels(){
    size_t nchans=0;
    for (int i=0; i<num_radars; i++)
        nchans += tx_freqs[i].size();
    return nchans;
}

size_t tx_data::get_num_channels(size_t radar){
    return tx_freqs[radar].size();
}

size_t tx_data::get_num_bb_samples(){
    //return tx_bb_vecs[0].size();
    return bb_vec.size();
}

size_t tx_data::get_num_rf_samples(){
    return tx_rf_vecs[0].size();
}

float* tx_data::get_bb_vec_ptr(size_t radar){
    //return (float*) &tx_bb_vecs[radar].front();
    return (float*) &bb_vec.front();
}

void tx_data::zero_rf_vec(size_t radar){
    for (size_t i=radar*(num_ants_per_radar); i<(radar+1)*(num_ants_per_radar); i++){
        tx_rf_vecs[i].assign(tx_rf_vecs[0].size(), std::complex<int16_t>(0,0));
    }
}

void tx_data::set_rf_vec_ptrs(std::vector<std::complex<int16_t> *>* rf_vec_ptrs){
    rf_vec_ptrs->resize(num_ants);
    for (size_t i=0; i<num_ants; i++){
        (*rf_vec_ptrs)[i] = &tx_rf_vecs[i].front();
    }
}

int16_t** tx_data::get_rf_vec_ptrs(size_t radar){
    tx_rf_fronts.resize(num_ants_per_radar);
    //    " to " <<  (radar+1)*(num_ants_per_radar)-1 << std::endl;
    //for (size_t i=radar*(num_ants_per_radar); i<(radar+1)*(num_ants_per_radar); i++){
    for (size_t i=0; i<num_ants_per_radar; i++){
        tx_rf_fronts[i] = (int16_t*) &tx_rf_vecs[radar*num_ants_per_radar+i].front();
    }
    return (int16_t**) &tx_rf_fronts.front();
}

float* tx_data::get_freqs(size_t radar){
    return (float*) &tx_freqs[radar].front();
}

float* tx_data::get_time_delays(size_t radar){
    return (float*) &time_delays[radar].front();
}
