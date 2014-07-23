#include <stdlib.h>
#include <vector>
#include <complex>
#include <iostream>
#include "control_program.h"
#include "site.h"
extern int verbose;

class rx_data{
    private:
    struct ControlPRM client;
    std::vector<ControlPRM> clients;
    int buf;
    size_t nclients; //number of clients registered to the usrp driver
    std::vector<std::vector<std::vector<std::vector<std::complex<float> > > > > rx_bb_vecs[2]; //baseband data, rx_bb_vecs[2][radar][channel][ant][nbbsamps]
    float bb_rate,rf_rate; //bb and rf sample rates
    float center_freq;
    size_t nbb_samples, nrf_samples; 
    float minrate, mintime;
    std::vector<size_t> nclient_samples;  //Number of samples requested by each client
    std::vector<std::vector<std::complex<int16_t> > > rx_rf_vecs[2]; //rf data, rx_rf_vecs[2][nants][nrfsamps]
    std::vector<std::vector<int16_t*> > rx_rf_vec_ptrs; 
    std::vector<std::vector<std::vector<float*> > > rx_bb_vec_ptrs;  //rx_bb_vec_ptrs[radar][channel][num_ants_per_radar]
    std::vector<std::vector<float**> > rx_bb_client_ptrs;  //rx_bb_client_ptrs[radar][channel]
    std::vector<std::vector<float> > rx_freqs; //Vectors of receive frequencies, one vector for each radar
    std::vector<std::vector<float> > rx_freq_offs[2]; //Vectors of receive frequencies, one vector for each radar
    std::vector<float> rx_all_freqs; //Vectors of all receive frequencies
    std::vector<float> rx_rel_freqs; //Vectors of all receive frequencies
    std::vector<float> bandwidths; //Vectors of all receive bandwidths
    size_t num_ants, num_radars, num_ants_per_radar;
    std::vector<size_t> nchannels; //Number of channels for each radar nchannels[nradars]

    public:
    rx_data(size_t nradars, size_t nants, float rxfreq, float rfrate);
    ~rx_data();
    void ready_client(struct ControlPRM* client);
    void reset_swing_buf();
    void clear_channel_list();
    size_t get_num_rxfreqs();
    size_t get_num_ants(); //Get number of antennas
    size_t get_num_radars(); //Get number of radars
    //size_t get_num_clients(); //Get total number of clients for all radars
    size_t get_num_clients(size_t radar); //Get number of clients for that radar
    float* get_freqs(size_t radar);
    //void set_freqs(std::vector<std::vector<float> >* rx_freqs);
    size_t get_num_bb_samples(); //Get number of bb samples.  It is and must be the same for all clients!
    size_t get_num_rf_samples(); //Get number of rf samples.  It is and must be the same for all antenna channels!
    size_t get_num_ants_per_radar();
    void set_bb_vec_ptrs(size_t radar, 
        size_t channel,
        std::vector<std::complex<float> *>* bb_vec_ptrs,
        int double_buf); //Set pointers to bb vectors
    void set_rf_vec_ptrs(std::vector<std::complex<int16_t> *>* rf_vec_ptrs);
    int16_t** get_rf_dptr(size_t radar);
    float*** get_bb_dptr(size_t radar);
};


rx_data::rx_data(size_t nradars, size_t nants, float rxfreq, float rfrate){
    nbb_samples =0; nrf_samples=0; minrate=0; mintime=0; 
    num_radars = nradars;
    num_ants = nants;
    num_ants_per_radar = nants/nradars;
    for (int i=0; i<2; i++){
        rx_rf_vecs[i].resize(nants);
    }
    rx_freqs.resize(nradars);
    for (int i=0; i<2; i++){
        rx_freq_offs[i].resize(nradars);
    }
    nchannels.resize(nradars);
    rx_rf_vec_ptrs.resize(nradars);
    rx_bb_vec_ptrs.resize(nradars);
    rx_bb_client_ptrs.resize(nradars);
    rf_rate = rfrate;
    buf = 0;
    center_freq = rxfreq;

    for (int i=0; i<2; i++){
        rx_bb_vecs[i].resize(nradars);
        for (int irad=0; irad<nradars; irad++){
            rx_bb_vecs[i][irad].resize(MAX_CHANNELS);
            for (int ichan=0; ichan<MAX_CHANNELS; ichan++){
                rx_bb_vecs[i][irad][ichan].resize(num_ants_per_radar);
            }
        }
    }
}

rx_data::~rx_data(){} //Don't need to free memory here--trust that memory allocated to vectors is freed when the vectors fall out of scope

size_t rx_data::get_num_rxfreqs(){
    return rx_freqs.size();
}

void rx_data::ready_client(struct ControlPRM* client){
    bb_rate = client->baseband_samplerate;
    nbb_samples = client->number_of_samples;
    nrf_samples = (size_t) (rf_rate * (float) nbb_samples / bb_rate);
    //rx_freqs[client->radar-1][client->channel-1] = client.rfreq;
    //rx_freq_offs[client->radar-1][client->channel-1] = 1e3*client->rfreq - center_freq;
    rx_freq_offs[buf][client->radar-1].push_back(1e3*client->rfreq - center_freq);
    for (int i=0; i<2; i++){ //for both buffers
        for (size_t iant=0; iant<num_ants_per_radar; iant++){ //for all antennas
            rx_bb_vecs[i][client->radar-1][client->channel-1][iant].resize(nbb_samples); 
        }
    }
    for (int i=0; i<2; i++){ //for both buffers
        //rx_rf_vecs[i].resize(num_ants);
        for (size_t iant=0; iant<num_ants; iant++){ //for all antennas
            rx_rf_vecs[i][iant].resize(nrf_samples);
        }
    }
}

void rx_data::reset_swing_buf(){
    buf = 0;
}

void rx_data::clear_channel_list(){
    for (int irad=0; irad<num_radars; irad++){
        rx_freq_offs[buf][irad].clear();
    }
}
    
size_t rx_data::get_num_ants(){
    return num_ants;
}

size_t rx_data::get_num_radars(){
    return num_radars;
}

size_t rx_data::get_num_ants_per_radar(){
    return num_ants_per_radar;
}

float* rx_data::get_freqs(size_t radar){
    return (float*) &rx_freq_offs[(buf+1)%2][radar].front();
}

size_t rx_data::get_num_clients(size_t radar){
    return rx_freq_offs[(buf+1)%2][radar].size();
}

size_t rx_data::get_num_bb_samples(){
    return nbb_samples;
}

size_t rx_data::get_num_rf_samples(){
    return nrf_samples;
}

void rx_data::set_bb_vec_ptrs(size_t radar, size_t channel, std::vector<std::complex<float> *>* bb_vec_ptrs, int double_buf){
    bb_vec_ptrs->resize(num_ants_per_radar);

    if (double_buf == 0){ //No swing buffering of raw data samples
        for (size_t i=0; i<num_ants_per_radar; i++){
            (*bb_vec_ptrs)[i] = &rx_bb_vecs[buf][radar][channel][i].front();
        }
    }
    else { //Yes swing-buffering of raw data samples
        for (size_t i=0; i<num_ants_per_radar; i++){
            (*bb_vec_ptrs)[i] = &rx_bb_vecs[(buf+1)%2][radar][channel][i].front();
        }
    }
}

void rx_data::set_rf_vec_ptrs(std::vector<std::complex<int16_t> *>* rf_vec_ptrs){
    if (verbose > 2) std::cout << "rx_data::set_rf_vec_ptrs(): buf num: " << buf << std::endl;
    rf_vec_ptrs->resize(num_ants);
    for (size_t i=0; i<num_ants; i++){ //for each antenna
        (*rf_vec_ptrs)[i] = &rx_rf_vecs[buf][i].front();
    }
    buf = (buf+1)%2; //Change the buffer we're pointing to
}

int16_t** rx_data::get_rf_dptr(size_t radar){
    if (verbose > 2) std::cout << rx_rf_vec_ptrs[radar].size() << std::endl;
    rx_rf_vec_ptrs[radar].resize(num_ants_per_radar);
    for (size_t i=0; i<num_ants_per_radar; i++){
        rx_rf_vec_ptrs[radar][i] = (int16_t*) &rx_rf_vecs[buf][radar*num_ants_per_radar+i].front();
    }
    return (int16_t**) &rx_rf_vec_ptrs[radar].front();
}

float*** rx_data::get_bb_dptr(size_t radar){
    //int nclients = rx_freqs[radar].size();
    int nchannels = rx_freq_offs[(buf+1)%2][radar].size();
    rx_bb_vec_ptrs[radar].resize(nchannels);
    rx_bb_client_ptrs[radar].resize(nchannels);
    for (int ichan=0; ichan<nchannels; ichan++){
        if (rx_bb_vecs[buf][radar][ichan][0].size() == 0){
            continue;
        }
        rx_bb_vec_ptrs[radar][ichan].resize(num_ants_per_radar);
        for (int j=0; j<num_ants_per_radar; j++){
            rx_bb_vec_ptrs[radar][ichan][j] = (float*) &rx_bb_vecs[buf][radar][ichan][j].front();
        }
        rx_bb_client_ptrs[radar][ichan] = (float**) &rx_bb_vec_ptrs[radar][ichan].front();
    }
    return (float***) &rx_bb_client_ptrs[radar].front();
}
