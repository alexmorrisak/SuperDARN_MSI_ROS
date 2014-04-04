int _convolve(std::vector<std::complex<float> >& signal, const std::vector<std::complex<float> >& taps);
int recv_to_buffer(
    uhd::usrp::multi_usrp::sptr usrp,
    uhd::rx_streamer::sptr rx_stream,
    std::vector<std::complex<short> *> recv_buffs,
    size_t num_requested_samples,
    float sample_rate,
    uhd::time_spec_t start_time,
    int *return_status);
