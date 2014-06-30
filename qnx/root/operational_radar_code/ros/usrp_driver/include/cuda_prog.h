void rx_process_gpu(
    int16_t **rx_buff_ptrs,
    float ***client_buff_ptr,
    size_t nrf_samples, //perhaps this should be different for each center frequency..
    size_t nbb_samples, //perhaps this should be different for each center frequency..
    size_t nfreqs,
    size_t nants,
    float rf_sample_rate,
    float client_sample_rate,
    float* rx_freqs // rx frequencies
);
 void tx_process_gpu(
    float* input_bb,//input vector (same tx sequence for all frequencies, antennas)
    int16_t** outputs_rf, //output vectors [NANTS][NRF_SAMPLES]
    size_t nbb_samples, //rate of input samples
    size_t nrf_samples, //rate of samples from host cpu to usrp
    float usrp_center_freq,
    float usrp_samp_rate,
    float *center_freqs, //list of center frequencies to mix up (down) to
    float *time_delays,//per-antenna time offset in ns for each beam
    size_t nchannels, // number of beam directions and/or center frequencies
    size_t nants // number of antennas
);

