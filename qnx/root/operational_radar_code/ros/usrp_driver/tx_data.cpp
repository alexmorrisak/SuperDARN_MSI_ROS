#include <stdlib.h>
#include <vector>
#include <complex>
#include <iostream>
//#include <txrx_data.hpp>
#include <control_program.h>
extern int verbose;

class tx_data{
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
    void register_client(size_t radar, size_t channel);
    void register_client(struct ControlPRM new_client);
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
}

tx_data::~tx_data(){} //Don't need to free memory here--trust that memory allocated to vectors is freed when the vectors fall out of scope

void tx_data::add_client(){
    tx_freqs[0].resize(tx_freqs[0].size()+1);
}

void tx_data::add_client(size_t radar){
    tx_freqs[radar].resize(tx_freqs[radar].size()+1);
}

void tx_data::register_client(size_t radar, size_t channel){
    tx_freqs[radar].resize(tx_freqs[radar].size()+1);
    radmap.push_back(radar);
    chanmap.push_back(channel);
}

void tx_data::register_client(struct ControlPRM new_client){
    tx_freqs[new_client.radar-1].resize(tx_freqs[new_client.radar-1].size()+1);
    radmap.push_back(new_client.radar-1);
    chanmap.push_back(new_client.channel-1);
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
    if (channel==0){
        tx_bb_vecs[0] = bb_vec;
    }
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
    return tx_bb_vecs[0].size();
}

size_t tx_data::get_num_rf_samples(){
    return tx_rf_vecs[0].size();
}

float* tx_data::get_bb_vec_ptr(size_t radar){
    return (float*) &tx_bb_vecs[radar].front();
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

