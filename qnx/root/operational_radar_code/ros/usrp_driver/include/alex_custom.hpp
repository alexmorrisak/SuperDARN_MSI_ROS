void _convolve(
    std::vector<std::complex<float> > *signal,
    std::vector<std::complex<float> > *taps);
void _convolve2(
    std::complex<float>* signal,
    unsigned int signal_len,
    std::complex<float>* taps,
    unsigned int taps_len
);
void recv_and_hold(
    unsigned int* trtimes,
    unsigned int* trdurations,
    unsigned int npulses,
    uhd::usrp::multi_usrp::sptr usrp,
    uhd::rx_streamer::sptr rx_stream,
    std::vector<std::complex<short> *> client_buff_ptrs,
    size_t num_requested_samples,
    uhd::time_spec_t start_time,
    int *return_status);
int rx_mix_downsample(
    std::vector<std::complex<short> *> rx_buff_ptrs,
    std::vector<std::complex<float> *> client_vec_ptrs,
    size_t nrf_samples,
    size_t nbb_samples,
    float rf_sample_rate,
    float client_sample_rate,
    std::vector<float> center_freqs,
    std::vector<float> bws);
void recv_to_buffer(
    uhd::usrp::multi_usrp::sptr usrp,
    uhd::rx_streamer::sptr rx_stream,
    std::vector<std::vector<std::complex<short> > *> recv_buffs,
    size_t num_requested_samples,
    float rf_sample_rate,
    float client_sample_rate,
    std::vector<float> center_freqs,
    uhd::time_spec_t start_time,
    int *return_status);
int recv_clr_freq(
    uhd::usrp::multi_usrp::sptr usrp,
    uhd::rx_streamer::sptr rx_stream,
    int bandwidth,
    int naverages,
    double *pwr);
int tx_mix_upsample(
    std::vector<std::complex<float> >*input_bb,//input vector (same tx sequence for all antennas)
    std::vector<std::vector<std::complex<float> > > *outputs_rf, //output vectors (number of antennas)
    float bb_sample_rate, //rate of input samples
    float rf_sample_rate, //rate of samples from host cpu to usrp
    float usrp_center_freq, //Center frequency of USRP
    std::vector<float> center_freqs, //list of center frequencies to mix up (down) to
    float time_delay); //time delay between each antenna in nanoseconds
void transmit_worker(
    uhd::tx_streamer::sptr tx_stream,
    std::vector<std::complex<short> *> pulse_seq_ptrs,
    int sequence_length,
    uhd::time_spec_t start_time);

