#include <control_program.h>

class tx_data{
    private:
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
    //std::vector<size_t> nclients; //List of number of clients attached to each radar

    public:
    tx_data(size_t nradars, size_t nants, float center_freq, float rf_samp_rate);
    ~tx_data();
    void ready_client(struct ControlPRM* new_client);

    struct TSGbuf* get_tsg_ptr(size_t index);
    void allocate_pulseseq_mem(size_t index);

    void unpack_pulseseq(size_t index);
    size_t get_seqbuf_len(size_t index);

    void make_bb_vecs(int32_t trise);
    void make_tr_times(struct TRTimes* tr_times);
    //void add_freq(size_t radar, size_t channel, float freq);
    void clear_channel_list();

    size_t get_num_ants(); //Get number of antennas
    size_t get_num_radars(); //Get number of radars
    size_t get_num_clients(); //Get total number of clients for all radars
    size_t get_num_clients(size_t radar); //Get number of clients for that radar
    size_t get_num_bb_samples(); //Get number of bb samples.  It is and must be the same for all radar channels!
    size_t get_num_rf_samples(); //Get number of rf samples.  It is and must be the same for all antenna channels!
    size_t get_num_ants_per_radar();
    float* get_bb_vec_ptr(); //Get pointer to bb vector
    float* get_bb_vec_ptr(size_t radar);
    void set_rf_vec_size(size_t nrf_samples); //Allocate memory for rf vectors
    void ready_client(int seq_len, float bb_samp_rate);
    void zero_rf_vec(size_t radar); //Allocate memory for rf vectors
    void set_rf_vec_ptrs(std::vector<std::complex<int16_t> *>* rf_vec_ptrs);
    int16_t** get_rf_vec_ptrs(size_t radar);
    float* get_freqs(size_t radar);
};

class rx_data{
    private:
    struct ControlPRM client;
    std::vector<ControlPRM> clients;
    int buf;
    size_t nclients; //number of clients registered to the usrp driver
    std::vector<std::vector<std::vector<std::vector<std::complex<float> > > > > rx_bb_vecs[2]; //baseband data, rx_bb_vecs[2][nclients][nants][nbbsamp
    float bb_rate,rf_rate; //bb and rf sample rates
    float center_freq;
    size_t nbb_samples, nrf_samples;
    float minrate, mintime;
    std::vector<size_t> nclient_samples;  //Number of samples requested by each client
    std::vector<std::vector<std::complex<int16_t> > > rx_rf_vecs[2]; //rf data, rx_rf_vecs[2][nants][nrfsamps]
    std::vector<std::vector<int16_t*> > rx_rf_vec_ptrs;
    std::vector<std::vector<std::vector<float*> > > rx_bb_vec_ptrs;  //rx_bb_vec_ptrs[nclients][nants]
    std::vector<std::vector<float**> > rx_bb_client_ptrs;  //rx_bb_client_ptrs[nclients]
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
    //void add_client(size_t nsamples, float freq, float bandwidth);
    void register_client(struct ControlPRM new_client);
    void unregister_client(size_t radar, size_t channel);
    void ready_client(struct ControlPRM client);
    void drop_client();
    void drop_client(size_t radar);
    size_t get_num_rxfreqs();
    size_t get_num_ants(); //Get number of antennas
    size_t get_num_radars(); //Get number of radars
    size_t get_num_clients(); //Get total number of clients for all radars
    size_t get_num_clients(size_t radar); //Get number of clients for that radar
    float* get_freqs(); //Get pointer to rx frequencies
    float* get_freqs(size_t radar);
    size_t get_num_bb_samples(); //Get number of bb samples.  It is and must be the same for all clients!
    size_t get_num_rf_samples(); //Get number of rf samples.  It is and must be the same for all antenna channels!
    size_t get_num_ants_per_radar();
    void set_bb_vec_ptrs(size_t radar,
        size_t channel,
        std::vector<std::complex<float> *>* bb_vec_ptrs,
        int double_buf);
    void set_rf_vec_ptrs(std::vector<std::complex<int16_t> *>* rf_vec_ptrs);
    int16_t** get_rf_dptr(size_t radar);
    float*** get_bb_dptr(size_t radar);
};
