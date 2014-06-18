//tx_process_gpu.cu
/* This function mixes and upsamples an arbitrary number
of frequency/beam channels onto each of the 16 antenna
channels.  Input rate = ~ 200 kHz, output rate = ~10 MHz.
Upsampling is done via 1st order linear interpolation
Current known limitations: 
    --Only simple beamforming, (i.e. equal
        amplitude and linear phase shift. No sidelobe suppression,etc.)  
    --Same pulse sequence for each freq/beam channel
*/
#include <complex>
#include <math.h>
#include <stdio.h>
#include <complex.h>

#define TEST 0
#define NRF_SAMPS 1000
//#define NRF_SAMPS 4000
#define NBB_SAMPS 100
#define NANTS 2
//extern int verbose;

/***********************************************************************
 * interpolate_and_multiply() function
 * A function to be used to upsample/interpolate between two bb samples.
 * This vector is then modulated by NCO(s) to bring it to the desired
 * center frequency.  A phase shift is applied to each carrier for 
 * beam-forming
 **********************************************************************/
__global__ void interpolate_and_multiply(
    float* indata, 
    int16_t** outdata, 
    float* radfreqs, 
    float* phase_delays
){
    /*Declare shared memory array for samples.
    Vectors are written into this array, one for each
    frequency/beam channel.  Then the thread block linearly combines
    the frequency/beam channels into a single vector to be
    transmitted on the antenna*/
    __shared__ float irf_samples[2000];
    __shared__ float qrf_samples[2000];

    //Calculate the increment between two adjacent rf samples
    float inc_i;
    float inc_q;
    inc_i = (indata[2*blockIdx.x+2] - indata[2*blockIdx.x]) / blockDim.x;
    inc_q = (indata[2*blockIdx.x+3] - indata[2*blockIdx.x+1]) / blockDim.x;

    /*Calculate the sample's phase value due to NCO mixing and beamforming*/
    float phase = fmod((double)(blockDim.x*blockIdx.x + threadIdx.x)*radfreqs[threadIdx.y], 2*M_PI) + 
        blockIdx.y*phase_delays[threadIdx.y];

    /*Calculate the output sample vectors, one for each freq/beam channel*/
    unsigned int localInx = threadIdx.y*blockDim.x+threadIdx.x;
    irf_samples[localInx] = 
        (indata[2*blockIdx.x] + threadIdx.x*inc_i) * cos(phase) - 
        (indata[2*blockIdx.x+1] + threadIdx.x*inc_q) * sin(phase);
    qrf_samples[localInx] =
        (indata[2*blockIdx.x] + threadIdx.x*inc_i) * sin(phase) +
        (indata[2*blockIdx.x+1] + threadIdx.x*inc_q) * cos(phase);

    /* Now linearly combine all freq/beam channels into a single vector*/
    __syncthreads();
    unsigned int outInx = blockDim.x*blockIdx.x+threadIdx.x;
    if(threadIdx.y == 0){
        for (unsigned int i=1; i<blockDim.y; i++){
            irf_samples[threadIdx.x] += irf_samples[threadIdx.x + i*blockDim.x];
            qrf_samples[threadIdx.x] += qrf_samples[threadIdx.x + i*blockDim.x];
        }
        outdata[blockIdx.y][2*outInx] = (int16_t) (0.95*32768*irf_samples[threadIdx.x]) & 0xfffe;
        outdata[blockIdx.y][2*outInx+1] = (int16_t) (0.95*32768*qrf_samples[threadIdx.x]) & 0xfffe;
    }
    
}

/***********************************************************************
 * tx_process_gpu() function
 * A function to be used to take a single vector of floats (based on 
 * the master tsg sequence), mix and upsample to the required
 * frequencies, and write to the antenna channel vectors pointed 
 * to by outputs_rf.
 **********************************************************************/
void tx_process_gpu(
    float* input_bb,//input vector (same tx sequence for all frequencies, antennas)
    int16_t** outputs_rf, //output vectors [NANTS][NRF_SAMPLES]
    size_t nbb_samples, //number of input samples
    size_t nrf_samples, //number of samples from host cpu to usrp
    float usrp_center_freq,
    float usrp_samp_rate,
    float *center_freqs, //list of center frequencies to mix up (down) to
    float *time_delays,//per-antenna time offset in ns for each beam
    size_t nchannels, // number of beam directions and/or center frequency channels
    size_t nants // number of antennas
){
   printf("Entering tx_process_gpu\n");
   float mixer_freqs[nchannels];
   float phase_delays[nchannels];

   //Calculate mixer frequencies
   printf("calculating mixer freqs\n");
   for(size_t c=0; c<nchannels; c++){
    mixer_freqs[c] = 2*M_PI*(center_freqs[c] - usrp_center_freq) / usrp_samp_rate;
   }

   //Calculate phase delays
   for(size_t c=0; c<nchannels; c++){
    float rad_phase = fmod(2*M_PI*1e-9*time_delays[c]*center_freqs[c], 2*M_PI);
    printf("rad_phase: %f\n", rad_phase);
    phase_delays[c] = rad_phase;
   }

   // Allocate memory for input vectors and copy data to GPU
   printf("allocating input samples\n");
   float* bbvec_d; 
   cudaMalloc((void**)&bbvec_d, 2*(nbb_samples+1)*sizeof(float));
   cudaMemset(bbvec_d, 0, 2*(nbb_samples+1)*sizeof(float));
   cudaMemcpy(bbvec_d, input_bb, 2*nbb_samples*sizeof(float), cudaMemcpyHostToDevice);

   // Allocate memory for output vectors and their pointers
   printf("allocating output sample memory\n");
   int16_t** rfvecptrs_d;
   int16_t* rfvecs_d[nants];
   int16_t* rfvecs_h[nants];
   cudaMalloc((void***)&rfvecptrs_d, nants*sizeof(int16_t*));
   for (size_t iant=0; iant<nants; iant++){
    cudaMalloc((void**)&rfvecs_d[iant], 2*nrf_samples*sizeof(int16_t));
    rfvecs_h[iant] = rfvecs_d[iant];
   }
   cudaMemcpy(rfvecptrs_d, rfvecs_h, nants*sizeof(int16_t*), cudaMemcpyHostToDevice);

   // Allocate memory for mixer frequencies and copy data to GPU
   float* mxrs_d; 
   cudaMalloc((void**)&mxrs_d, nchannels*sizeof(float));
   cudaMemcpy(mxrs_d, mixer_freqs, nchannels*sizeof(float), cudaMemcpyHostToDevice);

   // Allocate memory for phase delays (beam-forming) and copy data to GPU
   float* pds_d; 
   cudaMalloc((void**)&pds_d, nchannels*sizeof(float));
   cudaMemcpy(pds_d, phase_delays, nchannels*sizeof(float), cudaMemcpyHostToDevice);

   //Launch Kernel
   printf("Launching kernel\n");
   dim3 dimGrid(nbb_samples, nants, 1);
   dim3 dimBlock(nrf_samples/nbb_samples, nchannels, 1);
   interpolate_and_multiply<<<dimGrid,dimBlock>>>(bbvec_d, rfvecptrs_d, mxrs_d, pds_d);

   printf("copy back\n");
   //Copy output data back to host and free memory
   for (int i=0; i<nants; i++){
    cudaMemcpy(outputs_rf[i], rfvecs_d[i], 2*nrf_samples*sizeof(int16_t), cudaMemcpyDeviceToHost);
    cudaFree(rfvecs_d[i]);
   }
   printf("free\n");

   //Free other device memory
   cudaFree(rfvecptrs_d);
   cudaFree(pds_d);
   cudaFree(mxrs_d);
   cudaFree(bbvec_d);

   //printf("output samples head:\n");
   ////for (int i=0; i<nrf_samples; i+=1000){
   //int i = 0;
   //int count = 0;
   //while (i<nrf_samples && count < 100){
   // //if (input_bb[2*i] >= 0.1){
   // if (outputs_rf[0][2*i+1] >= 0.1){
   //     count++;
   //     for (int a=0; a<nants; a++){
   //         float mag = cabsf(outputs_rf[a][2*i] + outputs_rf[a][2*i+1]*I);
   //         float phi = cargf(outputs_rf[a][2*i] + outputs_rf[a][2*i+1]*I);
   //         //printf("output %i %i: (%f, %f)\t", a,i,outputs_rf[a][2*i], outputs_rf[a][2*i+1]);
   //         printf("output %i %i: (%f, %f)\t", a,i,mag, phi);
   //     }
   //     //if (input_bb[2*i] >= 0.1){
   //     //    printf("output %i %i: (%f, %f)\n", a,i,input_bb[2*i], input_bb[2*i+1]);
   //     //}
   // printf("\n");
   // }
   // i++;
   //}


}

#if TEST==1
int main(){
    float* input_vec;
    input_vec = (float*) malloc(2*NBB_SAMPS*sizeof(float));
    for (int i=0; i<NBB_SAMPS; i++){
        //input_vec[2*i] = 0;
        //input_vec[2*i+1] = 0;
        //if ((i%10) < 5){
        input_vec[2*i] = (float)i / (NBB_SAMPS);
        input_vec[2*i+1] = 0;
        //}
    }
    int16_t* output_vec[NANTS];
    for (int i=0; i<NANTS; i++){
        output_vec[i] = (int16_t*) malloc(2*NRF_SAMPS*sizeof(int16_t));
    }
    float center_freq[2] = {10.0, 10.2};
    //float center_freq[1] = {10.0};
    float time_delay[2] = {0.123, 0.123};
    //float time_delay = 0.0;

tx_process_gpu(
    input_vec,//input vector (same tx sequence for all frequencies, antennas)
    &output_vec[0], //output vectors [NANTS][NRF_SAMPLES]
    NBB_SAMPS, //rate of input samples
    NRF_SAMPS, //rate of samples from host cpu to usrp
    10,
    10,
    &center_freq[0], //list of center frequencies to mix up (down) to
    &time_delay[0],//per-antenna time offset in ns for each beam
    2, // number of beam directions and/or center frequencies
    NANTS); // number of antennas

    //for (int i=0; i<NRF_SAMPS; i++){
    //    for (int j=0; j<NANTS; j++){
    //        float mag = cabsf(output_vec[j][2*i] + output_vec[j][2*i+1]*I);
    //        float phi = cargf(output_vec[j][2*i] + output_vec[j][2*i+1]*I);
    //        printf("output %i %i: (%f, %f)\t",j, i, mag, phi);
    //    }
    //    printf("\n");
    //}

    return 0;
}
#endif
