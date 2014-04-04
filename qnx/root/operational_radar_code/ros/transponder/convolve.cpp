#include <complex>
#include <vector>

int _convolve(std::vector<std::complex<float> >& signal, const std::vector<std::complex<float> >& taps){
        std::vector<std::complex<float> > output;
        std::complex<float>  temp;
        unsigned int i,j;
        output.clear();

        for (i=0; i<taps.size(); i++){
                temp = std::complex<float>(0,0);
                for(j=0; j<(i+1); j++){
                        temp += signal[i+j] * taps[taps.size()-1-j];
                }
                output.push_back(temp);
        }

        for (i=0; i<(signal.size()-taps.size()); i++){
                temp = std::complex<float>(0,0);
                for(j=0; j<taps.size(); j++){
                        temp += signal[i+j] * taps[j];
                }
                output.push_back(temp);
        }

        for (i=0; i<signal.size(); i++){
                signal[i] = output[i];// / std::complex<float> (30,0);
        }
        return 1;
}


