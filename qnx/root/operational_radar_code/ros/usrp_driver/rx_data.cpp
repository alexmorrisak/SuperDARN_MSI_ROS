#include <stdlib.h>
#include <vector>
#include <complex>
#include <iostream>
#include <control_program.h>
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
    std::vector<std::vector<float> > rx_freq_offs; //Vectors of receive frequencies, one vector for each radar
    std::vector<float> rx_all_freqs; //Vectors of all receive frequencies
    std::vector<float> rx_rel_freqs; //Vectors of all receive frequencies
    std::vector<float> bandwidths; //Vectors of all receive bandwidths
    size_t num_ants, num_radars, num_ants_per_radar;
    std::vector<size_t> radnum; //client-radar mapping, list of clients' radar they are "attached" to
    std::vector<size_t> channum; //client-channel mapping, list of clients' channel they are "attached" to

    public:
    rx_data(size_t nradars, size_t nants, float rxfreq, float rfrate);
    ~rx_data();
    void register_client(struct ControlPRM client);
    void unregister_client(size_t radar, size_t channel);
    void ready_client(struct ControlPRM client);
    void drop_client();
    void drop_client(size_t radar);
    size_t get_num_rxfreqs();
    size_t get_num_ants(); //Get number of antennas
    size_t get_num_radars(); //Get number of radars
    size_t get_num_clients(); //Get total number of clients for all radars
    size_t get_num_clients(size_t radar); //Get number of clients for that radar
    float* get_freqs();
    float* get_freqs(size_t radar);
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
    for (int i=0; i<2; i++){
        rx_bb_vecs[i].resize(nradars);
    }
    nbb_samples =0; nrf_samples=0; minrate=0; mintime=0; 
    num_radars = nradars;
    num_ants = nants;
    num_ants_per_radar = nants/nradars;
    for (int i=0; i<2; i++){
        rx_rf_vecs[i].resize(nants);
    }
    rx_freqs.resize(nradars);
    rx_freq_offs.resize(nradars);
    rx_rf_vec_ptrs.resize(nradars);
    rx_bb_vec_ptrs.resize(nradars);
    rx_bb_client_ptrs.resize(nradars);
    rf_rate = rfrate;
    buf = 0;
    center_freq = rxfreq;
}

rx_data::~rx_data(){} //Don't need to free memory here--trust that memory allocated to vectors is freed when the vectors fall out of scope

size_t rx_data::get_num_rxfreqs(){
    return rx_freqs.size();
}

void rx_data::register_client(struct ControlPRM new_client){
    clients.push_back(new_client);
    nclient_samples.resize(nclient_samples.size() + 1);
    bandwidths.resize(nclient_samples.size() + 1);
    rx_freqs[new_client.radar-1].resize(rx_freqs[new_client.radar-1].size()+1);
    rx_freq_offs[new_client.radar-1].resize(rx_freq_offs[new_client.radar-1].size()+1);
    rx_all_freqs.resize(rx_all_freqs.size()+1);
    rx_rel_freqs.resize(rx_rel_freqs.size()+1);
    for (int i=0; i<2; i++){
        rx_bb_vecs[i][new_client.radar-1].resize(rx_bb_vecs[i][new_client.radar-1].size()+1);
    }
    radnum.push_back(new_client.radar-1);
    channum.push_back(new_client.channel-1);
}

void rx_data::unregister_client(size_t radar, size_t channel){
    rx_freqs[radar].erase(rx_freqs[radar].begin() + channel);
    rx_freq_offs[radar].erase(rx_freq_offs[radar].begin() + channel);
    for (size_t i=0; i<radnum.size(); i++){
        if (radnum[i] == radar && channum[i] == channel){
            nclient_samples.erase(nclient_samples.begin()+i);
            bandwidths.erase(bandwidths.begin()+i);
            rx_all_freqs.erase(rx_all_freqs.begin()+i);
            rx_rel_freqs.erase(rx_rel_freqs.begin()+i);
            radnum.erase(radnum.begin()+i);
            channum.erase(channum.begin()+i);
            break;
        }
    }
}

void rx_data::ready_client(struct ControlPRM client){
    size_t c= -1;
    for (size_t i=0; i<radnum.size(); i++){
        //std::cout << "radar: " << client.radar << " channel: " << client.channel << std::endl;
        if (radnum[i] == client.radar-1 && channum[i] == client.channel-1){
            c = i;
            break;
        }
    }
    if (c == -1){
        std::cerr << "Error! Invalid radar/channel mapping to client\n";
    }

    nclient_samples[c] = client.number_of_samples;
    bandwidths[c] = client.baseband_samplerate;
    if (bandwidths[c] > minrate){
        minrate = bandwidths[c];
    }
    if (client.number_of_samples/minrate > mintime){
        mintime = client.number_of_samples/minrate;
    }
    bb_rate = minrate; //Perhaps add some checking for number "roundness"
    nbb_samples = (size_t) (mintime * (float)bb_rate); //Perhaps add some checking for number "roundness"
    //nrf_samples = (size_t) 2*(rf_rate * (float) nbb_samples / bb_rate);
    nrf_samples = (size_t) (rf_rate * (float) nbb_samples / bb_rate);
    rx_freqs[client.radar-1][client.channel-1] = client.rfreq;
    rx_all_freqs[c] = client.rfreq;
    rx_rel_freqs[c] = 1e3*client.rfreq - center_freq;
    rx_freq_offs[client.radar-1][client.channel-1] = 1e3*client.rfreq - center_freq;
    for (int i=0; i<2; i++){ //for both buffers
        //rx_bb_vecs[i].resize(radnum.size());
        for (size_t irad=0; irad<num_radars; irad++){ //for all radars
            for (size_t ichan=0; ichan<rx_bb_vecs[i][irad].size(); ichan++){ //for all channels
                rx_bb_vecs[i][irad][ichan].resize(num_ants_per_radar); 
                for (size_t iant=0; iant<num_ants_per_radar; iant++){ //for all antennas
                    rx_bb_vecs[i][irad][ichan][iant].resize(nbb_samples); 
                }
            }
        }
    }
    for (int i=0; i<2; i++){ //for both buffers
        //for (int irad=0; irad<num_radars; irad++){ //for all radars
            rx_rf_vecs[i].resize(num_ants);
            for (size_t iant=0; iant<num_ants; iant++){ //for all antennas
                rx_rf_vecs[i][iant].resize(nrf_samples);
            }
    }
}

//void rx_data::ready_clients(size_t radar, size_t channel, size_t nsamples, float freq, float bandwidth){
//    size_t c= -1;
//    for (size_t i=0; i<radnum.size(); i++){
//        if (radnum[i] == radar && channum[i] == channel){
//            c = i;
//            break;
//        }
//    }
//    if (c == -1){
//        std::cerr << "Error! Invalid radar/channel mapping to client\n";
//    }
//
//    nclient_samples[c] = nsamples;
//    bandwidths[c] = bandwidth;
//    if (bandwidth > minrate){
//        minrate = bandwidth;
//    }
//    if (nsamples/minrate > mintime){
//        mintime = nsamples/minrate;
//    }
//    bb_rate = minrate; //Perhaps add some checking for number "roundness"
//    nbb_samples = (size_t) (mintime * (float)bb_rate); //Perhaps add some checking for number "roundness"
//    nrf_samples = (size_t) 2*(rf_rate * (float) nbb_samples / bb_rate);
//    std::cout << "nrf_samples: " << nrf_samples << "\n";
//    rx_freqs[radar][channel] = freq;
//    rx_all_freqs[c] = freq;
//    rx_rel_freqs[c] = 1e3*freq - center_freq;
//    for (int i=0; i<2; i++){ //for both buffers
//        rx_bb_vecs[i].resize(radnum.size());
//        for (size_t j=0; j<rx_bb_vecs[i].size(); j++){ //for all clients
//            rx_bb_vecs[i][j].resize(num_ants); 
//            for (size_t k=0; k<num_ants; k++){ //for all antennas
//                rx_bb_vecs[i][j][k].resize(nbb_samples);
//            }
//        }
//    }
//    for (int i=0; i<2; i++){ //for both buffers
//        rx_rf_vecs[i].resize(num_ants);
//        for (size_t k=0; k<num_ants; k++){ //for all antennas
//            rx_rf_vecs[i][k].resize(nrf_samples);
//        }
//    }
//}

void rx_data::drop_client(){
    rx_freqs[0].resize(rx_freqs[0].size()-1);
    //rx_bb_vecs.resize(rx_bb_vecs.size()-1);
    radnum.pop_back();
}

void rx_data::drop_client(size_t radar){
    rx_freqs[radar].resize(rx_freqs[radar].size()-1);
    //rx_bb_vecs.resize(rx_bb_vecs.size()-1);
    radnum.pop_back();
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

float* rx_data::get_freqs(){
    return (float*) &rx_rel_freqs.front();
}

float* rx_data::get_freqs(size_t radar){
    return (float*) &rx_freq_offs[radar].front();
}

size_t rx_data::get_num_clients(){
    return radnum.size();
}

size_t rx_data::get_num_clients(size_t radar){
    return rx_freq_offs[radar].size();
}

size_t rx_data::get_num_bb_samples(){
    return nbb_samples;
}

size_t rx_data::get_num_rf_samples(){
    return nrf_samples;
}

//float* rx_data::get_bb_vec_ptr(){
//    return (float*) &rx_bb_vecs[0].front();
//}
//
//float* rx_data::get_bb_vec_ptr(size_t radar){
//    return (float*) &tx_bb_vecs[radar].front();
//}

//void rx_data::set_rf_vec_size(size_t nrf_samples){
//    for (size_t i=0; i<tx_rf_vecs.size(); i++){
//        tx_rf_vecs[i].resize(nrf_samples);
//    }
//}

//void rx_data::zero_rf_vec(size_t radar){
//    //std::fill(tx_rf_vecs[radar].begin(), tx_rf_vecs[radar].end(), std::complex<int16_t>(0,0));
//    std::cout << "Entering zero antenna channel " << std::endl;
//    std::cout << "num ants: " << num_ants_per_radar << std::endl;
//    std::cout << radar*(num_ants_per_radar) << " " <<  (radar+1)*(num_ants_per_radar) << std::endl;
//    for (size_t i=radar*(num_ants_per_radar); i<(radar+1)*(num_ants_per_radar); i++){
//        //vec_ptrs[i] = (int16_t*) &tx_rf_vecs[i].front();
//        tx_rf_vecs[i].assign(tx_rf_vecs[0].size(), std::complex<int16_t>(0,0));
//        std::cout << "Zeroing antenna channel " << i << std::endl;
//    }
//}
void rx_data::set_bb_vec_ptrs(size_t radar, size_t channel, std::vector<std::complex<float> *>* bb_vec_ptrs, int double_buf){
    //bb_vec_ptrs->resize(num_ants_per_radar);
    bb_vec_ptrs->resize(num_ants_per_radar);
    for (size_t i=0; i<num_ants_per_radar; i++){
        if (double_buf == 0){
            (*bb_vec_ptrs)[i] = &rx_bb_vecs[buf][radar][channel][i].front();
        } 
        else {
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

//int16_t** rx_data::get_rf_dptr(){
//    rx_rf_vec_ptrs.resize(num_ants);
//    for (size_t i=0; i<num_ants; i++){
//        rx_rf_vec_ptrs[i] = (int16_t*) &rx_rf_vecs[buf][i].front();
//    }
//    return (int16_t**) &rx_rf_vec_ptrs.front();
//}

int16_t** rx_data::get_rf_dptr(size_t radar){
    if (verbose > 2) std::cout << rx_rf_vec_ptrs[radar].size() << std::endl;
    rx_rf_vec_ptrs[radar].resize(num_ants_per_radar);
    for (size_t i=0; i<num_ants_per_radar; i++){
        rx_rf_vec_ptrs[radar][i] = (int16_t*) &rx_rf_vecs[buf][radar*num_ants_per_radar+i].front();
    }
    return (int16_t**) &rx_rf_vec_ptrs[radar].front();
}

//float*** rx_data::get_bb_dptr(){
//    int num_clients = rx_all_freqs.size();
//    std::cout << "get_bb_dptr: numclients: " << num_clients << std::endl;
//    rx_bb_vec_ptrs.resize(num_clients);
//    rx_bb_client_ptrs.resize(num_clients);
//    for (int i=0; i<num_clients; i++){
//        rx_bb_vec_ptrs[i].resize(num_ants);
//        for (int j=0; j<num_ants; j++){
//            rx_bb_vec_ptrs[i][j] = (float*) &rx_bb_vecs[buf][i][j].front();
//        }
//        rx_bb_client_ptrs[i] = (float**) &rx_bb_vec_ptrs[i].front();
//    }
//    return (float***) &rx_bb_client_ptrs.front();
//}

float*** rx_data::get_bb_dptr(size_t radar){
    int nclients = rx_freqs[radar].size();
    rx_bb_vec_ptrs[radar].resize(nclients);
    rx_bb_client_ptrs[radar].resize(nclients);
    for (int i=0; i<nclients; i++){
        rx_bb_vec_ptrs[radar][i].resize(num_ants_per_radar);
        for (int j=0; j<num_ants_per_radar; j++){
            //std::cout << i << " " << j << std::endl;
            //std::cout << rx_bb_vecs[0].size() << std::endl;
            //std::cout << rx_bb_vecs[0][0].size() << std::endl;
            //std::cout << rx_bb_vecs[0][0][0].size() << std::endl;
            //std::cout << rx_bb_vecs[0][0][0][0].size() << std::endl;
            //std::cout <<  (float*) &rx_bb_vecs[buf][radar][i][j].front() << std::endl;
            rx_bb_vec_ptrs[radar][i][j] = (float*) &rx_bb_vecs[buf][radar][i][j].front();
        }
        rx_bb_client_ptrs[radar][i] = (float**) &rx_bb_vec_ptrs[radar][i].front();
    }
    return (float***) &rx_bb_client_ptrs[radar].front();
}
//float*** rx_data::get_bb_dptr(){
//    int num_clients = rx_all_freqs.size();
//    std::cout << "get_bb_dptr: numclients: " << num_clients << std::endl;
//    rx_bb_vec_ptrs.resize(num_clients);
//    rx_bb_client_ptrs.resize(num_clients);
//    for (int i=0; i<num_clients; i++){
//        rx_bb_vec_ptrs[i].resize(num_ants);
//        for (int j=0; j<num_ants; j++){
//            rx_bb_vec_ptrs[i][j] = (float*) &rx_bb_vecs[buf][i][j].front();
//        }
//        rx_bb_client_ptrs[i] = (float**) &rx_bb_vec_ptrs[i].front();
//    }
//    return (float***) &rx_bb_client_ptrs.front();
//}
