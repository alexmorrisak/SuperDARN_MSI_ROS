void _convolve(std::vector<std::complex<float> >& signal, const std::vector<std::complex<float> >& taps);
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
