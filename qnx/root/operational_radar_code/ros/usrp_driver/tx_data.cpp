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

class tx_data{
    struct TSGbuf pulseseqs[4];
    std::vector<unsigned char> pulseseq_reps[4];
    std::vector<unsigned char> pulseseq_codes[4];

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
    size_t num_ants, num_radars, num_ants_per_radar;
    std::vector<size_t> radmap, chanmap;
    float bb_samp_rate, rf_samp_rate;
    float center_freq;

    public:
    tx_data(size_t nradars, size_t nants, float center_freq, float rf_samp_rate);
    ~tx_data();
    void add_client();
    void add_client(size_t radar);
    void register_client(struct ControlPRM new_client);
    void ready_client(struct ControlPRM* client);

    struct TSGbuf* get_seq_ptr(size_t index);
    void allocate_pulseseq_mem(size_t index);

    void unpack_pulseseq(size_t index);
    unsigned char* get_seqbuf_ptr(size_t index);
    size_t get_seqbuf_len(size_t index);

    void make_bb_vecs(int32_t trise);
    //void add_freq(size_t radar, size_t channel, float freq);
    void clear_freqs();

    void unregister_client(size_t radar, size_t channel);
    void drop_client();
    void drop_client(size_t radar);
    void add_pulse_seq(size_t channel, float freq, std::vector<std::complex<float> >& bb_vec);
    void add_pulse_seq(size_t radar, size_t channel, float freq, std::vector<std::complex<float> >& bb_vec);
    size_t get_num_ants(); //Get number of antennas
    size_t get_num_ants_per_radar();
    size_t get_num_radars(); //Get number of radars
    size_t get_num_clients(); //Get total number of clients for all radars
    size_t get_num_clients(size_t radar); //Get number of clients for that radar
    size_t get_num_bb_samples(); //Get number of bb samples.  It is and must be the same for all radar channels!
    size_t get_num_rf_samples(); //Get number of rf samples.  It is and must be the same for all antenna channels!
    float* get_bb_vec_ptr(size_t radar);
    void set_rf_vec_size(size_t nrf_samples); //Allocate memory for rf vectors
    void zero_rf_vec(size_t radar); //Allocate memory for rf vectors
    void set_rf_vec_ptrs(std::vector<std::complex<int16_t> *>* rf_vec_ptrs);
    int16_t** get_rf_vec_ptrs(size_t radar);
    float* get_freqs(size_t radar);
};

tx_data::tx_data(size_t nradars, size_t nants, float center_freq, float rf_samp_rate){
    num_ants = nants;
    num_radars = nradars;
    num_ants_per_radar = nants/nradars;
    tx_bb_vecs.resize(nradars);
    tx_freqs.resize(nradars);
    tx_rf_vecs.resize(nants);

    old_index = -10;
}

tx_data::~tx_data(){} //Don't need to free memory here--trust that memory allocated to vectors is freed when the vectors fall out of scope

void tx_data::add_client(){
    tx_freqs[0].resize(tx_freqs[0].size()+1);
}

void tx_data::add_client(size_t radar){
    tx_freqs[radar].resize(tx_freqs[radar].size()+1);
}

void tx_data::register_client(struct ControlPRM new_client){
    tx_freqs[new_client.radar-1].resize(tx_freqs[new_client.radar-1].size()+1);
    radmap.push_back(new_client.radar-1);
    chanmap.push_back(new_client.channel-1);
}

void tx_data::ready_client(struct ControlPRM* client){
    tx_freqs[client->radar-1].push_back(1e3*client->tfreq);
    //radmap.push_back(new_client.radar-1);
    //chanmap.push_back(new_client.channel-1);
}

struct TSGbuf* tx_data::get_seq_ptr(size_t index){
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
        //if (verbose > -1) std::cout << "Need to unpack pulseseq " << r << " " << c << " " << inde
        //if (verbose > -1) std::cout << "Pulseseq length: " << tx.get_seq_ptr(index)->len << "\n";
    
        // unpack the timing sequence
        //seq_count[r][c]=0;
        //step=(int)((double)pulseseqs[r][c][index]->step/(double)STATE_TIME+0.5);
        int step=(int)((double)pulseseqs[index].step/(double)STATE_TIME+0.5);
                
        //If DDS or RX Offset is negative pad the seq_buf with the maximum negative offset
        //offset_pad=(int)((double)MIN(dds_offset,rx_offset)/((double)STATE_TIME+0.5))-2;
        int offset_pad=0;
        if (verbose > -1) std::cout << "offset pad: " << offset_pad << "\n";  
        for(int i=0;i>offset_pad;i--) {
          seq_buf[index].push_back(0);
          //seq_count[r][c]++;
        }
        for (int i=0; i<pulseseqs[index].len; i++){
            for (int j=0; j<step*pulseseqs[index].rep[i]; j++){
                seq_buf[index].push_back(pulseseqs[index].code[i]);
                //seq_count[index]++;
            }
        }
        old_index=index;
    }
}

unsigned char* tx_data::get_seqbuf_ptr(size_t index){
    //std::cout << "tx_data::get_seq_ptr " << "step : " << pulseseqs[index].step << std::endl;
    //std::cout << "tx_data::get_seq_ptr " << "length : " << pulseseqs[index].len << std::endl;
    return &seq_buf[index].front();
}

size_t tx_data::get_seqbuf_len(size_t index){
    //std::cout << "tx_data::get_seq_ptr " << "step : " << pulseseqs[index].step << std::endl;
    //std::cout << "tx_data::get_seq_ptr " << "length : " << pulseseqs[index].len << std::endl;
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
    for (size_t i=0; i<bb_vec.size(); i+=50){
        std::cout << i << " " << bb_vec[i] << std::endl;
    }
}

//void tx_data::add_freq(size_t radar, size_t channel, float freq){
//    tx_freqs[radar].push_back(freq);
//}

void tx_data::clear_freqs(){
    // Clear frequency list for each radar
    for (int irad=0; irad<tx_freqs.size(); irad++){
        tx_freqs[irad].clear();
    }
}


void tx_data::unregister_client(size_t radar, size_t channel){
    if (verbose > 2) std::cout << "TX_DATA: UNREGISTER_CLIENT: radar: " << radar << " channel: " << channel << std::endl;
    size_t ichan=0;
    for (size_t i=0; i<radmap.size(); i++){
        if (radmap[i] == radar && chanmap[i] == channel){
            tx_freqs[radar].erase(tx_freqs[radar].begin()+ichan);
            radmap.erase(radmap.begin()+i);
            chanmap.erase(chanmap.begin()+i);
            break;
        }
        if (radmap[i] == radar){
            ichan++;
        }
    }
    //tx_freqs[radar].resize(tx_freqs[radar].size()+1);
    //radmap.push_back(radar);
    //chanmap.push_back(channel);
}

void tx_data::drop_client(){
    tx_freqs[0].resize(tx_freqs[0].size()-1);
}

void tx_data::drop_client(size_t radar){
    tx_freqs[radar].resize(tx_freqs[radar].size()-1);
}

void tx_data::add_pulse_seq(size_t channel, float freq, std::vector<std::complex<float> >& bb_vec){
    //if (channel==0){
    //    tx_bb_vecs[0] = bb_vec;
    //}
    tx_freqs[0][channel] = freq;
}

void tx_data::add_pulse_seq(size_t radar, size_t channel, float freq, std::vector<std::complex<float> >& bb_vec){
    for (size_t i=0; i<num_radars; i++){
        if (i != radar && tx_bb_vecs[i].size() != bb_vec.size()){
           tx_bb_vecs[i].resize(bb_vec.size(), 0);
        }
        if (channel==0){
            tx_bb_vecs[radar] = bb_vec; //Add the new vector of baseband values to the vector of clients for that radar
        }
    }
    tx_freqs[radar][channel] = freq;
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

size_t tx_data::get_num_clients(){
    size_t nchans=0;
    for (int i=0; i<num_radars; i++)
        nchans += tx_freqs[i].size();
    return nchans;
}

size_t tx_data::get_num_clients(size_t radar){
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

void tx_data::set_rf_vec_size(size_t nrf_samples){
    for (size_t i=0; i<tx_rf_vecs.size(); i++){
        tx_rf_vecs[i].resize(nrf_samples);
    }
}

//void tx_data::ready_client(int seq_length, float bb_samp_rate){
//    size_t nrf_samples = seq_length * rf_samp_rate / bb_samp_rate;
//    for (size_t i=0; i<tx_rf_vecs.size(); i++){
//        tx_rf_vecs[i].resize(nrf_samples);
//    }
//}

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
