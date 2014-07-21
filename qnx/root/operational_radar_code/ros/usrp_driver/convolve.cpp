#include <iostream>
#include <complex>
#include <vector>

void convolve(
    std::complex<float>* signal, 
    unsigned int signal_len,
    std::complex<float>* taps, 
    unsigned int taps_len
    //std::complex<float>* outsignal
){
    std::vector<std::complex<float> > outsignal(signal_len);
    std::complex<float>  temp;
    unsigned int i,j;
    
    for (i=0; i<taps_len/2; i++){
            temp = std::complex<float>(0,0);
            for(j=0; j<taps_len/2+i; j++){
                    temp += signal[i+j] * taps[taps_len/2-i+j];
            }
            //if (i %5 == 0 ) std::cout << i << " " << temp << std::endl;
            outsignal[i] = temp;
    }
    
    for (i=taps_len/2; i<signal_len-taps_len; i++){
            temp = std::complex<float>(0,0);
            for(j=0; j<taps_len; j++){
                    temp += signal[i+j] * taps[j];
            }
            outsignal[i] = temp;
            //std::cout << i << " " << temp << std::endl;
    }

    for (i=signal_len-taps_len; i<signal_len/2; i++){
            temp = std::complex<float>(0,0);
            for(j=0; j<signal_len-i; j++){
                    temp += signal[i+j] * taps[j];
            }
            outsignal[i] = temp;
            //if (i 5 == 0 ) std::cout << i << " " << temp << std::endl;
    }
    for (i=0; i<signal_len; i++)
        signal[i] = outsignal[i];
}


