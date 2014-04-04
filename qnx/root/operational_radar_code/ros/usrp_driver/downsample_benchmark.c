

#define OSR 60 //OverSample Rate. Converting 200 kHz to 3.333 kHz is OSR of 60.

/***********************************************************************
 * recv_to_buffer function
 * A function to be used to receive samples from the USRP and
 * write them into a buffer.  Also performs filtering and 
 * samplerate conversion
 **********************************************************************/
void recv_to_buffer(
    uhd::usrp::multi_usrp::sptr usrp, //handle for USRP's usrp object
    uhd::rx_streamer::sptr rx_stream, //handle for USRP's rx_stream object
    std::vector<std::complex<short> *> recv_buffs, //baseband samples to return to main()
    size_t num_requested_samples, //number of samples requested by main()
    uhd::time_spec_t start_time, //When to start the streaming (same as tx stream)
    int *return_status
){
    int num_total_samps = 0;
    int num_usrp_samples = (int) (num_requested_samples*OSR);

    //std::vector<std::complex<double> > filter_taps(ntaps, 1.0/ntaps); //just a moving-average filter
    double filter_taps[OSR]; //c-style version of filter_taps
    for(int i=0;i<OSR-1;i++){ //initialize to moving average filter values
    	filter_taps[i]=1/OSR;
    }

    ///*Calculate taps for a Gaussian filter*/
    //float alpha = 32*9.86 / (0.8328*osr);
    //for (int i=0; i<ntaps; i++){
    //        filter_taps[i] = pow(alpha/3.14,0.5)*pow(2.7183, 
    //                -1*(alpha)*pow((((float)i-(ntaps-1)/2)/ntaps),2)) / ntaps;
    //}

    //std::vector<std::complex<double> > buff1(osr);
    //std::vector<std::complex<double> *> buff_ptrs(1,&buff0.front());
    //buffs.push_back(&buff1.front());
    double buff[OSR][2]; //buffer for incoming sample stream
    //std::vector<std::complex<double> > circular_buff0;
    //std::complex<double> temp(0,0);
    double temp[2]; //This is a C-compatible version of std::complex<double>
    int ibuff = 0;

    //setup streaming
	//
	//

    //start streaming
    while(num_total_samps < num_usrp_samples){
     	//receive osr number of samples and store them in the vector(s) pointed to by buffs
    	// This function will need to be adjusted if this program isn't connected to the USRP
        size_t num_rx_samps = rx_stream->recv(buff_ptrs, osr, md, timeout);
    		
    	//Filter and decimate the simulated data buff[][]
        temp[ibuff][0]=0;
        temp[ibuff][1]=0;
        for(int i=0;i<ntaps-1;i++){
                temp[ibuff][0]+=(filter_taps[i][0]*buff[i][0]; //I-component
                temp[ibuff][1]+=(filter_taps[i][0]*buff[i][1]; //Q-component
        }
        num_total_samps += num_rx_samps;
	ibuff = num_total_samps % OSR;
    }
    printf("Recv thread success!\n");
}

