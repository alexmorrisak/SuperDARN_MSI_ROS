#include <complex>
#include <vector>
//Added by Alex for usrp
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/utils/thread_priority.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/utils/static.hpp>
#include <uhd/exception.hpp>
#include <boost/thread.hpp>
#include <thread>
#include <math.h>

extern int verbose;

/***********************************************************************
 * rx_beamform() function
 * A function to be used to do the final processing on the baseband
 * samples.
 *
 * If in non-imaging configuration, this function simply takes
 * the main array samples and the back array samples and bit-packs them
 * so that they are ready for the client process.
 *
 * If in imaging configuration, this function multiplies the array
 * vector for each range gate by a "beam forming vector."  This beam 
 * forming vector is generated in the application main() and passed to
 * this function by pointer.  Vector size == number antennas. For simple
 * beamforming it is a vector of unit-weight taps and uniform phase
 * shifts from antenna to antenna.  For sidelobe suppression or other
 * beamforming techniques it may be some other vector.
 **********************************************************************/
/***********************************************************************
 * TODO: For speed in imaging configuration, this function may need to
 * be executed in its own thread context (double-buffer), and/or 
 * executed via CUDA/GPU.
 **********************************************************************/

void rx_beamform(
    uint32_t* client_main,
    uint32_t* client_back,
    std::vector<std::complex<float> *>* bb_vec_ptrs,
    size_t nants_main,
    size_t nants_back,
    size_t nsamps,
    std::vector<std::complex<float> >* beamform_main,
    std::vector<std::complex<float> >* beamform_back)
{
    int debug=0;
    /* Full Imaging Configuration*/
    if (nants_main==16 && nants_back==4){
        if (debug) printf("Using Full imaging configuration\n");
        for (size_t isamp=0; isamp<nsamps; isamp++){
            int16_t temp_main[2] = {0,0};
            for (size_t iant=0; iant<15; iant++){
                temp_main[0] += (*bb_vec_ptrs)[iant][isamp].real() * (*beamform_main)[iant].real() -
                    (*bb_vec_ptrs)[iant][isamp].imag() * (*beamform_main)[iant].imag();
                temp_main[1] += (*bb_vec_ptrs)[iant][isamp].real() * (*beamform_main)[iant].imag() +
                    (*bb_vec_ptrs)[iant][isamp].imag() * (*beamform_main)[iant].real();
            }
            client_main[isamp] = ((uint32_t) (temp_main[1] << 16) & 0xffff0000) | ((uint32_t) temp_main[0] & 0x0000ffff);

            int16_t temp_back[2] = {0,0};
            for (size_t iant=16; iant<20; iant++){
                temp_back[0] += (*bb_vec_ptrs)[iant][isamp].real() * (*beamform_back)[iant].real() -
                    (*bb_vec_ptrs)[iant][isamp].imag() * (*beamform_back)[iant].imag();
                temp_back[1] += (*bb_vec_ptrs)[iant][isamp].real() * (*beamform_back)[iant].imag() +
                    (*bb_vec_ptrs)[iant][isamp].imag() * (*beamform_back)[iant].real();
            }
            client_back[isamp] = ((uint32_t) (temp_back[1] << 16) & 0xffff0000) | ((uint32_t) temp_back[0] & 0x0000ffff);
            if (debug) printf("Sample %i: (%i, %i)\t(%i, %i)\n", isamp, temp_main[0], temp_main[1], temp_back[0], temp_back[1]);
        }
    }
    /* Non-Imaging Configuration*/
    else if (nants_main==1 && nants_back==1){
        if (debug) printf("Using non-imaging configuration\n");
        for (size_t isamp=0; isamp<nsamps; isamp++){
            int16_t temp_main[2] = {0,0};
            temp_main[0] += (*bb_vec_ptrs)[0][isamp].real();
            temp_main[1] += (*bb_vec_ptrs)[0][isamp].imag();
            client_main[isamp] = ((uint32_t) (temp_main[1] << 16) & 0xffff0000) | ((uint32_t) temp_main[0] & 0x0000ffff);

            int16_t temp_back[2] = {0,0};
            temp_back[0] += (*bb_vec_ptrs)[1][isamp].real();
            temp_back[1] += (*bb_vec_ptrs)[1][isamp].imag();
            client_back[isamp] = ((uint32_t) (temp_back[1] << 16) & 0xffff0000) | ((uint32_t) temp_back[0] & 0x0000ffff);
            if (debug) printf("Sample %i: (%i, %i)\t(%i, %i)\n", isamp, temp_main[0], temp_main[1], temp_back[0], temp_back[1]);
        }
    }
    /* Quasi-Imaging Configuration, for testing. Requires custom main-array/back-array mapping*/
    else {
        if (debug) printf("Using custom configuration\n");
        for (size_t isamp=0; isamp<nsamps; isamp++){
            int16_t temp_main[2] = {0,0};
            for (size_t iant=0; iant<nants_main; iant++){
                temp_main[0] += 
                    0.2*(*bb_vec_ptrs)[iant][isamp].real() * (*beamform_main)[iant].real() -
                    0.2*(*bb_vec_ptrs)[iant][isamp].imag() * (*beamform_main)[iant].imag();
                temp_main[1] += 
                    0.2*(*bb_vec_ptrs)[iant][isamp].real() * (*beamform_main)[iant].imag() +
                    0.2*(*bb_vec_ptrs)[iant][isamp].imag() * (*beamform_main)[iant].real();
            }
            client_main[isamp] = ((uint32_t) (temp_main[1] << 16) & 0xffff0000) | ((uint32_t) temp_main[0] & 0x0000ffff);

            int16_t temp_back[2] = {0,0};
            for (size_t iant=0; iant<nants_back; iant++){
                temp_back[0] += (*bb_vec_ptrs)[iant][isamp].real() * (*beamform_main)[iant].real() -
                    (*bb_vec_ptrs)[iant][isamp].imag() * (*beamform_main)[iant].imag();
                temp_back[1] += (*bb_vec_ptrs)[iant][isamp].real() * (*beamform_main)[iant].imag() +
                    (*bb_vec_ptrs)[iant][isamp].imag() * (*beamform_main)[iant].real();
            }
            client_back[isamp] = ((uint32_t) (temp_back[1] << 16) & 0xffff0000) | ((uint32_t) temp_back[0] & 0x0000ffff);
            if (debug) printf("Sample %i: (%i, %i)\t(%i, %i)\n", isamp, temp_main[0], temp_main[1], temp_back[0], temp_back[1]);
        }
    }
}

            
