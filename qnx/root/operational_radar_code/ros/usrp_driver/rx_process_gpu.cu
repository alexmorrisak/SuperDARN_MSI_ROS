// rx_process_gpu.cu

/*This function takes ~20 antenna channels that are each sampled at a very high rate
(~10 MHz) and mixes, filters, and decimates an arbitrary number of frequency channels
out of each antenna channel.  The input data is 2-dimensional [NANTS][NRFSAMPLES] and
the output data is 3-dimensional [NFREQS][NANTS][NBBSAMPLES].  This operation lends
itself to parallelization, so it is performed on an Nvidia GPU.*/

/*Description of the general strategy can be found at the UAF SuperDARN google drive:
/UAFSuperDARN/General/Notes/SuperDARN_gpu_processing*/

#include <math.h>
#include <stdio.h>
#include <sys/types.h>
#include <unistd.h>

#define TEST 0
#define NRFSAMPS 5000
#define NANTS 1
#define NFREQS 1
#define DECIM_RATE 200
#define MAX_BLOCK_SIZE 512

#if TEST == 0
    extern int verbose;
#endif
#if TEST == 1
    int verbose;
#endif

__global__ void multiply_and_add(float ***samples, float ***odata, float **filter)
{
    __shared__ float itemp[1024];//Max number of threads in a block
    __shared__ float qtemp[1024]; 

    unsigned int tid = threadIdx.y*blockDim.x+threadIdx.x;
    unsigned int tfilt;
    unsigned int tsamp;

    //Each block writes exactly a baseband sample for each frequency
    /*blockIdx.x corresponds to time-domain tiling; NBBSAMPS == gridDim.x
    blockIdx.y corresponds to antenna channel tiling; NANTS == gridDim.y*/
    unsigned int output_idx = 2*blockIdx.x;

    // calculate index of sample from global memory samples array
    tsamp = (blockIdx.x*blockDim.x)/4 + 4*threadIdx.x;

    double i0 = (double) samples[threadIdx.y][blockIdx.y][tsamp];
    double q0 = (double) samples[threadIdx.y][blockIdx.y][tsamp+1];
    double i1 = (double) samples[threadIdx.y][blockIdx.y][tsamp+2];
    double q1 = (double) samples[threadIdx.y][blockIdx.y][tsamp+3];

    // get filter values from global memory
    tfilt = 4*threadIdx.x;
    float p0re = filter[threadIdx.y][tfilt];
    float p0im = filter[threadIdx.y][tfilt+1];
    float p1re = filter[threadIdx.y][tfilt+2];
    float p1im = filter[threadIdx.y][tfilt+3];
        
    // mix samples with nco, perform first reduction
    itemp[tid] = p0re * i0 - p0im * q0 + p1re * i1 - p1im * q1;
    qtemp[tid] = p0re * q0 + p0im * i0 + p1re * q1 + p1im * i1;
        
     __syncthreads();

     // parallel reduce samples (could unroll loop for speedup?)
     // Do this as long as the reduction is a power of 2
     unsigned int s, rem;
     s = blockDim.x;// / blockDim.y;
     rem = blockDim.x % 2;
     while(s > 0 && rem == 0){
        s /= 2;
        if (threadIdx.x < s) {
            itemp[tid] += itemp[tid + s];
            qtemp[tid] += qtemp[tid + s];
        }
        rem = s % 2;

        __syncthreads();
     }
     //// Now do a serial reduction for the remaining
     if(threadIdx.x == 0){
        for(unsigned int i=1; i<s; i++){
           itemp[tid] += itemp[tid + i];
           qtemp[tid] += qtemp[tid + i];
        }
     }
     __syncthreads();

     if (threadIdx.x == 0) {
        odata[threadIdx.y][blockIdx.y][output_idx] = (float) itemp[tid];
        odata[threadIdx.y][blockIdx.y][output_idx+1] = (float) qtemp[tid];
     }
}     


__global__ void multiply_mix_add(int16_t **samples, float ***odata, float **filter)
{
    __shared__ float itemp[1024];
    __shared__ float qtemp[1024]; 

    unsigned int tid = threadIdx.y*blockDim.x+threadIdx.x;
    unsigned int tfilt;
    unsigned int tsamp;

    //Each block writes exactly a baseband sample for each frequency
    /*blockIdx.x corresponds to time-domain tiling; NBBSAMPS == gridDim.x
    blockIdx.y corresponds to antenna channel tiling; NANTS == gridDim.y
    threadIdx.y corresponds to frequency channel tiling; NFREQS == blockDim.y*/
    unsigned int output_idx = 2*blockIdx.x;

    // calculate index of sample from global memory samples array
    tsamp = (blockIdx.x*blockDim.x) + 4*threadIdx.x;
    //double i0 = (double) samples[blockIdx.y][tsamp];
    //double q0 = (double) samples[blockIdx.y][tsamp+1];
    //double i1 = (double) samples[blockIdx.y][tsamp+2];
    //double q1 = (double) samples[blockIdx.y][tsamp+3];

    // get filter values from global memory
    tfilt = 4*threadIdx.x;
    //double p0re = filter[threadIdx.y][tfilt];
    //double p0im = filter[threadIdx.y][tfilt+1];
    //double p1re = filter[threadIdx.y][tfilt+2];
    //double p1im = filter[threadIdx.y][tfilt+3];
        
    // mix samples with nco, perform first reduction
    //itemp[tid] = p0re * i0 - p0im * q0 + p1re * i1 - p1im * q1;
    itemp[tid] = 
        filter[threadIdx.y][tfilt] * samples[blockIdx.y][tsamp] - 
        filter[threadIdx.y][tfilt+1] * samples[blockIdx.y][tsamp+1] +
        filter[threadIdx.y][tfilt+2] * samples[blockIdx.y][tsamp+2] -
        filter[threadIdx.y][tfilt+3] * samples[blockIdx.y][tsamp+3];
    //qtemp[tid] = p0re * q0 + p0im * i0 + p1re * q1 + p1im * i1;
    qtemp[tid] = 
        filter[threadIdx.y][tfilt] * samples[blockIdx.y][tsamp+1] + 
        filter[threadIdx.y][tfilt+1] * samples[blockIdx.y][tsamp] +
        filter[threadIdx.y][tfilt+2] * samples[blockIdx.y][tsamp+3] +
        filter[threadIdx.y][tfilt+3] * samples[blockIdx.y][tsamp+2];
        
     __syncthreads();

     /* Example: dmrate==100,
     100 -> 50 -> 25 -> 5 -> 1*/
     // parallel reduce samples (could unroll loop for speedup?)
     // Do this as long as the reduction is a power of 2
     unsigned int s, rem;
     s = blockDim.x;
     rem = blockDim.x % 2;
     while(s > 0 && rem == 0){
        s /= 2;
        if (threadIdx.x < s) {
            itemp[tid] += itemp[tid + s];
            qtemp[tid] += qtemp[tid + s];
        }
        rem = s % 2;

        __syncthreads();
     }

     // Do this as long as the reduction is a power of 5
     rem = s % 5;
     while(s > 0 && rem == 0){
        s /= 5;
        if (threadIdx.x < s) {
            itemp[tid] = itemp[tid] + itemp[tid + s] + itemp[tid+2*s] + itemp[tid+3*s] + itemp[tid+4*s];
            qtemp[tid] = qtemp[tid] + qtemp[tid + s] + qtemp[tid+2*s] + qtemp[tid+3*s] + qtemp[tid+4*s];
        }
        rem = s % 5;

        __syncthreads();
     }

     // Now do a serial reduction for the remaining
     if(threadIdx.x == 0){
        for(unsigned int i=1; i<s; i++){
           itemp[tid] += itemp[tid + i];
           qtemp[tid] += qtemp[tid + i];
        }
     }
     __syncthreads();

     if (threadIdx.x == 0) {
        /*Now do phase adjustment on the output samples*/
        // phase increment of the NCO can be calculated from two adjacent filter taps
        //double phase_inc = atan(p1im / p1re) - atan(p0im / p0re);
        double phase_inc = 
            atan(filter[threadIdx.y][tfilt+3] / filter[threadIdx.y][tfilt+2]) - 
            atan(filter[threadIdx.y][tfilt+1] / filter[threadIdx.y][tfilt]);

        /*Phase remainder exists because the NCO oscillator 
        may not complete an exact 360% rotation in a filter window*/
        double phi_rem = blockIdx.x*fmod((0.5*blockDim.x) * phase_inc, 2*M_PI);

        double ltemp = (double) itemp[tid];
        itemp[tid] = itemp[tid] * cos(phi_rem) - qtemp[tid] * sin(phi_rem);
        qtemp[tid] = ltemp * sin(phi_rem) + qtemp[tid] * cos(phi_rem);

        //deciding the output
        odata[threadIdx.y][blockIdx.y][output_idx] = (float) itemp[tid];
        //odata[threadIdx.y][blockIdx.y][output_idx] = threadIdx.x;
        odata[threadIdx.y][blockIdx.y][output_idx+1] = (float) qtemp[tid];
        //odata[threadIdx.y][blockIdx.y][output_idx+1] = 20;
     }
}     

void rx_process_gpu(
    int16_t **rx_buff_ptrs,
    float ***client_buff_ptr,
    size_t nrf_samples, 
    size_t nbb_samples, //perhaps this should be different for each center frequency..
    size_t nfreqs, 
    size_t nants, 
    float rf_sample_rate,
    float client_sample_rate,
    float* rx_freqs // rx center frequencies
    //std::vector<float> center_freqs, // center frequencies
    //std::vector<float> bws // bandwidth of each center frequency
){
    int debug = 0;
    struct timespec t0, t1;
    struct timespec tick, tock;
    float elapsed_t, elapsed_proc_t;
    int dmrate = nrf_samples / nbb_samples;
    int dmrate0 = dmrate;
    //nants *= 10;
    if (debug) printf("Entered rx_process_gpu()\n");


    //if (dmrate*nfreqs > MAX_BLOCK_SIZE){
    //    dmrate0 = MAX_BLOCK_SIZE / nfreqs;
    //    dmrate1 = dmrate / dmrate0;
    //    extra_stages=1;
    //}
    dmrate0 = 100;
    int dmrate1 = 2*dmrate / dmrate0;
        
    int ntaps0 = 2*dmrate0; //The coarse filter length is 2x the decimation rate
    int ntaps1 = 8*dmrate1; //The fine filter length is 8x the decimation rate

    float filter_taps0[nfreqs][ntaps0][2];
    float filter_taps1[nfreqs][ntaps1][2];
    double NCO0[nfreqs][ntaps0][2]; //filter taps sent to the GPU

    /*Calculate the per-sample phase shift of each NCO*/
    double radfreq0[nfreqs];
    for (size_t i=0;i<nfreqs;i++){
	    radfreq0[i] = double (2*M_PI* rx_freqs[i]) / rf_sample_rate;
        if (debug) printf("radfreq0[%i]: %f\n", i, radfreq0[i]);
        if (debug) printf("rx_freqs[%i]: %f\n", i, rx_freqs[i]);
    }

    /*Rectangular-window for coarse filtering.  
    [TODO: design Kaiser (beta=5) or similar window for better performance.
    See Google drive doc SuperDARN_GPU_processing]*/
    for (int i=0;i<ntaps0;i++){
            filter_taps0[0][i][0] = 1./ntaps0; //Q-component is zero
            //filter_taps0[0][i][0] = (0.54-0.46*cos((2*M_PI*((float)(i)+0.5))/ntaps0));
            
            filter_taps0[0][i][1] = 0; //Q-component is zero
    }

    /*Mix each filter up(down) to the desired pass-band frequency.
    Use the first set of coefficients as the reference*/
    double ftemp;
    for (size_t i=1;i<nfreqs;i++){
    	for (int j=0;j<ntaps0;j++){
                NCO0[i][j][0] = cos(-j*radfreq0[i]);
                NCO0[i][j][1] = sin(-j*radfreq0[i]);
                ftemp = filter_taps0[0][j][0];
    	        filter_taps0[i][j][0] = NCO0[i][j][0] * filter_taps0[0][j][0] - NCO0[i][j][1] * filter_taps0[0][j][1];
    	        filter_taps0[i][j][1] = NCO0[i][j][0] * filter_taps0[0][j][1] + NCO0[i][j][1] * ftemp;
	    }
    }
    /*Now go back to the first set of coefficients and modulate that*/
    for (int j=0;j<ntaps0;j++){
            NCO0[0][j][0] = cos(-j*radfreq0[0]);
            NCO0[0][j][1] = sin(-j*radfreq0[0]);
            ftemp = filter_taps0[0][j][0];
            filter_taps0[0][j][0] = NCO0[0][j][0] * filter_taps0[0][j][0] - NCO0[0][j][1] * filter_taps0[0][j][1];
            filter_taps0[0][j][1] = NCO0[0][j][0] * filter_taps0[0][j][1] + NCO0[0][j][1] * ftemp;
	}



    if (debug) printf("Allocating and copying filter tap data\n");
    /*Allocate device memory for filter taps and copy data into it*/
    float *taps_dptr[nfreqs];// = (float*) malloc(nfreqs*sizeof(float*));
    float* taps_ptrs[nfreqs];
    float **taps_ptr_dptr;
    cudaMalloc( (void***)&taps_ptr_dptr, nfreqs*sizeof(float*)); //for the pointers to the taps
    for (int ifreq=0;ifreq<nfreqs;ifreq++){
        cudaMalloc( (void**)&taps_dptr[ifreq], ntaps0*sizeof(float)*2); //for the taps themselves
        cudaMemcpy(
             taps_dptr[ifreq], 
             &filter_taps0[ifreq][0][0],
             ntaps0*2*sizeof(float),
             cudaMemcpyHostToDevice);
        taps_ptrs[ifreq] = taps_dptr[ifreq];
    }
    cudaMemcpy(
         taps_ptr_dptr,
         taps_ptrs,
         nfreqs*sizeof(float*),
         cudaMemcpyHostToDevice);

    if (debug) printf("Allocating and copying input data\n");
    /*Allocate device memory for input data and copy data into it*/
    /* TODO: this should be a function in recv_and_hold(); i.e. data
    should be ready to rock by the time this function is called*/
    int16_t *indata_vp_d[nants];// = malloc(nfreqs*sizeof(int16_t*));
    int16_t *indata_p2vp_h[nants];// = (int16_t**) malloc(nants*sizeof(int16_t*));// = (int16_t**) malloc(nfreqs*sizeof(int16_t*));
    int16_t** indata_p2vp_d;
    cudaMalloc( (void***)&indata_p2vp_d, nants*sizeof(int16_t*));
    for (int iant=0; iant<nants; iant++){
        cudaMalloc( (void**)&indata_vp_d[iant], (nrf_samples+ntaps0)*sizeof(int16_t)*2);
        cudaMemset(indata_vp_d[iant], 0, (nrf_samples+ntaps0)*sizeof(int16_t)*2);
        //cudaMemset(indata_vp_d[iant], 0, ntaps0*sizeof(int16_t));
        //cudaMemset(indata_vp_d[iant]+2*nrf_samples+ntaps0, 0, ntaps0*sizeof(int16_t));
        //indata_vp_d[iant] += dmrate0;
        if (debug) printf("rf data to GPU. nrfsamples: %i\n", nrf_samples);
        cudaMemcpy(
                indata_vp_d[iant] + ntaps0,
                rx_buff_ptrs[iant], 
                nrf_samples*2*sizeof(int16_t),
                cudaMemcpyHostToDevice);
        indata_p2vp_h[iant] = indata_vp_d[iant] + dmrate0;
    }
    if (debug) printf("rf data ptrs to GPU\n");
    cudaMemcpy(
            indata_p2vp_d,
            indata_p2vp_h,
            nants*sizeof(int16_t*),
            cudaMemcpyHostToDevice);
        
    if (debug) printf("Allocating output data\n");
    /*Allocate device memory for output data*/
    float* outdata_vp_d[nfreqs][nants];// = malloc(nfreqs*sizeof(float*));
    float* outdata_vp_h[nfreqs][nants];// = (float**) malloc(nants*sizeof(float*));// = (float**) malloc(nfreqs*sizeof(float*));
    float** outdata_p2vp_d[nfreqs];
    float** outdata_p2vp_h[nfreqs];// = (float**) malloc(nants*sizeof(float*));// = (float**) malloc(nfreqs*sizeof(float*));
    float*** outdata_p2channels_d;
    cudaMalloc( (void****)&outdata_p2channels_d, nfreqs*sizeof(float**));
    for (int ifreq=0; ifreq<nfreqs; ifreq++){
        cudaMalloc( (void***)&outdata_p2vp_d[ifreq], nants*sizeof(float*));
        //outdata_p2vp_h[ifreq] = outdata_p2vp_d[ifreq];
        for (int iant=0; iant<nants; iant++){
            cudaMalloc( (void**)&outdata_vp_d[ifreq][iant], (nrf_samples/dmrate0)*sizeof(float)*2);
            outdata_vp_h[ifreq][iant] = outdata_vp_d[ifreq][iant];
        }
        cudaMemcpy(
                outdata_p2vp_d[ifreq],
                &outdata_vp_h[ifreq][0],
                nants*sizeof(float*),
                cudaMemcpyHostToDevice);
    outdata_p2vp_h[ifreq] = outdata_p2vp_d[ifreq];
    }
    cudaMemcpy(
            outdata_p2channels_d,
            outdata_p2vp_h,
            nfreqs*sizeof(float**),
            cudaMemcpyHostToDevice);

    //cudaMalloc( (void**)&outdata_dptr, nfreqs*nbb_samples*nants*sizeof(float)*2);
    //float output_buffer[NFREQS][NANTS][nbb_samples][2];// = (float*) malloc(nbb_samples*nants*nfreqs*2*sizeof(float));


    dim3 dimGrid(nrf_samples/dmrate0+1,nants,1);
    dim3 dimBlock(ntaps0/2,nfreqs,1);

    if (debug){
        printf("nrfsamples: %i\n", nrf_samples);
        printf("decimation rate 0: %i\n", dmrate0);
        printf("nimtsamples: %i\n", nrf_samples/dmrate0);
        printf("decimation rate 1: %i\n", dmrate1);
        printf("nbbsamples: %i\n", nbb_samples);
        printf("dimGrid: (%i, %i, %i)\n", nrf_samples/dmrate0, nants, 1);
        printf("dimBlock: (%i, %i, %i)\n", ntaps0/2, nfreqs, 1);
    }


    //printf("About to print output of first stage\n");
    //for (int i=0; i<nrf_samples/dmrate0; i+=100){
    //    //printf("rx_process_gpu: output samples head\n");
    //    int iant =0, ifreq=0;
    //    printf("output %i, %i: (%i, %i)\n",ifreq, iant,
    //    (int) outdata_p2vp_h[0][0][2*i],
    //    (int) outdata_p2vp_h[0][0][2*i+1]);
    //  printf("\n\n");
    //}

    clock_gettime(CLOCK_MONOTONIC, &tick);
    multiply_mix_add<<<dimGrid, dimBlock>>>(indata_p2vp_d, outdata_p2channels_d, taps_ptr_dptr);
    cudaDeviceSynchronize();
    clock_gettime(CLOCK_MONOTONIC, &tock);



    /* Uncomment the following to take a sneak-peak at the intermediate stage data*/

    
    if (debug>1 | TEST==1){
        float* vps[nfreqs][nants];
        for (int ifreq=0; ifreq<nfreqs; ifreq++){
            for (int iant=0; iant<nants; iant++){
                vps[ifreq][iant] = (float*) malloc(nrf_samples/dmrate0 * 2*sizeof(float));
                cudaMemcpy( 
                    vps[ifreq][iant], 
                    outdata_vp_d[ifreq][iant], 
                    (nrf_samples/dmrate0) * 2*sizeof(float),
                    cudaMemcpyDeviceToHost);
            }
        }
        printf("\nPrinting intermediate stage data\n\n");
        for (int ifreq=0; ifreq<nfreqs; ifreq++){
            for (int iant=0; iant<nants; iant++){
                if (iant ==0){
                    for (int isamp=0; isamp < nrf_samples/dmrate0; isamp++){
                        printf("%i, %i, %i: (%.1f, %.1f)\n", ifreq, iant, isamp, vps[ifreq][iant][2*isamp], vps[ifreq][iant][2*isamp+1]);
                    }
                }
            }
        }
    }
    
    
    

    /*

    First Stage Done!!  Now do a finer filtering at the much-lower sample rate

    */

    /* Free memory related to raw rf samples*/
    cudaFree(indata_p2vp_d);
    for (int iant=0; iant<nants; iant++){
        cudaFree(indata_vp_d[iant]);
    }

    ntaps1 = 8*dmrate1;

    /*Allocate device memory for 2nd stage input data,
    copy output of previous stage into new location,
    and free all memory related to 1st filter stage*/ 
    float* imt_vp_d[nfreqs][nants];
    float* imt_vp_h[nfreqs][nants];
    float** imt_p2vp_d[nfreqs];
    float** imt_p2vp_h[nfreqs];
    float*** imt_p2channels_d;
    cudaMalloc( (void****)&imt_p2channels_d, nfreqs*sizeof(float**));
    cudaFree(outdata_p2channels_d);
    for (int ifreq=0; ifreq<nfreqs; ifreq++){
        cudaMalloc( (void***)&imt_p2vp_d[ifreq], nants*sizeof(float*));
        cudaFree(outdata_p2vp_d[ifreq]);
        for (int iant=0; iant<nants; iant++){
            cudaMalloc( (void**)&imt_vp_d[ifreq][iant], ((nrf_samples/dmrate0)+ntaps1)*sizeof(float)*2);
            cudaMemset( imt_vp_d[ifreq][iant], 0x00, ((nrf_samples/dmrate0)+ntaps1)*sizeof(float)*2);
            cudaMemcpy( 
                imt_vp_d[ifreq][iant]+ntaps1, 
                outdata_vp_d[ifreq][iant], 
                (nrf_samples/dmrate0) * 2*sizeof(float),
                cudaMemcpyDeviceToDevice);
            //imt_vp_d[ifreq][iant] += ntaps1;
            //imt_vp_h[ifreq][iant] = imt_vp_d[ifreq][iant] + ntaps1;
            imt_vp_h[ifreq][iant] = imt_vp_d[ifreq][iant];
            cudaFree(outdata_vp_d[ifreq][iant]);
        }
        cudaMemcpy(
                imt_p2vp_d[ifreq],
                imt_vp_h[ifreq],
                nants*sizeof(float*),
                cudaMemcpyHostToDevice);
    imt_p2vp_h[ifreq] = imt_p2vp_d[ifreq];
    }
    cudaMemcpy(
            imt_p2channels_d,
            imt_p2vp_h,
            nfreqs*sizeof(float**),
            cudaMemcpyHostToDevice);

    /* Calculate filtertaps for second filter stage
    The filters for each channel are probably the same but in certain applications
    they might not be.  Take care of that here if you like*/
    for (int ifreq=0; ifreq<nfreqs; ifreq++){
        for (int i=0;i<ntaps1;i++){
                double x = 4*(2*M_PI*((float)i/ntaps1) - M_PI);
                filter_taps1[ifreq][i][0] = (0.54-0.46*cos((2*M_PI*((float)(i)+0.5))/ntaps1))
                	*sin(x)/(x);
                //filter_taps1[ifreq][i][0] = 1./ntaps1; //Q-component is zero
                filter_taps1[ifreq][i][1] = 0; //Q-component is zero
                //printf("filter tap %i: %f\n", i, filter_taps1[ifreq][i][0]);
        }
        filter_taps1[ifreq][ntaps1/2][0]=1./ntaps1; //handle the divide-by-zero condition
    }

    /*Allocate device memory for filter taps and copy data into it*/
    cudaFree(taps_ptr_dptr);
    for (int ifreq=0;ifreq<nfreqs;ifreq++)
        cudaFree(taps_dptr[ifreq]);

    cudaMalloc( (void***)&taps_ptr_dptr, nfreqs*sizeof(float*)); //for the pointers to the taps
    for (int ifreq=0;ifreq<nfreqs;ifreq++){
        cudaMalloc( (void**)&taps_dptr[ifreq], ntaps1*sizeof(float)*2); //for the taps themselves
        cudaMemcpy(
             taps_dptr[ifreq], 
             filter_taps1[ifreq][0],
             ntaps1*2*sizeof(float),
             cudaMemcpyHostToDevice);
        taps_ptrs[ifreq] = taps_dptr[ifreq];
    }
    cudaMemcpy(
         taps_ptr_dptr,
         taps_ptrs,
         nfreqs*sizeof(float*),
         cudaMemcpyHostToDevice);

    /*Allocate device memory for final output data*/
    float* fout_vp_d[nfreqs][nants];// = malloc(nfreqs*sizeof(float*));
    float* fout_vp_h[nfreqs][nants];// = (float**) malloc(nants*sizeof(float*));// = (float**) malloc(nfreqs*sizeof(float*));
    float** fout_p2vp_d[nfreqs];
    float** fout_p2vp_h[nfreqs];// = (float**) malloc(nants*sizeof(float*));// = (float**) malloc(nfreqs*sizeof(float*));
    float*** fout_p2channels_d;
    cudaMalloc( (void****)&fout_p2channels_d, nfreqs*sizeof(float**));
    for (int ifreq=0; ifreq<nfreqs; ifreq++){
        cudaMalloc( (void***)&fout_p2vp_d[ifreq], nants*sizeof(float*));
        //outdata_p2vp_h[ifreq] = outdata_p2vp_d[ifreq];
        for (int iant=0; iant<nants; iant++){
            cudaMalloc( (void**)&fout_vp_d[ifreq][iant], nbb_samples*sizeof(float)*2);
            fout_vp_h[ifreq][iant] = fout_vp_d[ifreq][iant];
        }
        cudaMemcpy(
                fout_p2vp_d[ifreq],
                fout_vp_h[ifreq],
                nants*sizeof(float*),
                cudaMemcpyHostToDevice);
    fout_p2vp_h[ifreq] = fout_p2vp_d[ifreq];
    }
    cudaMemcpy(
            fout_p2channels_d,
            fout_p2vp_h,
            nfreqs*sizeof(float**),
            cudaMemcpyHostToDevice);

    if (debug){
        printf("nimtsamples: %i\n", nrf_samples/dmrate0);
        printf("nbbsamples: %i\n", nbb_samples);
        printf("decimation rate 1: %i\n", dmrate1);
        printf("dimGrid: (%i, %i, %i)\n", nbb_samples, nants, 1);
        printf("dimBlock: (%i, %i, %i)\n", ntaps1/2, nfreqs, 1);
    }

    dim3 dimGrid1(nbb_samples,nants,1);
    dim3 dimBlock1(ntaps1/2,nfreqs,1);
    

    clock_gettime(CLOCK_MONOTONIC, &t0);
    multiply_and_add<<<dimGrid1, dimBlock1>>>(imt_p2channels_d, fout_p2channels_d, taps_ptr_dptr);
    cudaDeviceSynchronize();
    clock_gettime(CLOCK_MONOTONIC, &t1);

    //here is the output
    for (int ifreq=0; ifreq<nfreqs; ifreq++){
        cudaFree(taps_dptr[ifreq]);
        cudaFree(fout_p2vp_d[ifreq]);
        cudaFree(imt_p2vp_d[ifreq]);
        for (int iant=0; iant<nants; iant++){
        //printf("copying %i samples back to client buffer\n",nbb_samples);
        //cudaMemcpy(
        //    client_buff_ptr[ifreq][iant],
        //    imt_vp_d[ifreq][iant], 
        //    nbb_samples*2*sizeof(float),
        //    cudaMemcpyDeviceToHost);
        
        cudaMemcpy(
            client_buff_ptr[ifreq][iant],
            fout_vp_d[ifreq][iant], 
            nbb_samples*2*sizeof(float),
            cudaMemcpyDeviceToHost);

        /* Uncomment the following to get a peak at the output samples*/
        if (debug > 1 | TEST==1){
            for(int i=0;i<nbb_samples;i+=1){
                    //if (iant == 0){
                    printf("output %i, %i, %i: (%i, %i)\n",ifreq, iant, i,
                         (int) client_buff_ptr[ifreq][iant][2*i],
                         (int) client_buff_ptr[ifreq][iant][2*i+1]);
                    //}
            }
        }

        cudaFree(fout_vp_d[ifreq][iant]);
        cudaFree(imt_vp_d[ifreq][iant]);
        //printf("rx_process_gpu: output samples head\n");
        }
      //printf("\n\n");
    }
    cudaFree(fout_p2channels_d);
    cudaFree(imt_p2channels_d);
    cudaFree(taps_ptr_dptr);

    elapsed_t = (1e9*t1.tv_sec + t1.tv_nsec) - (1e9*t0.tv_sec + t0.tv_nsec);
    elapsed_proc_t = (1e9*tock.tv_sec + tock.tv_nsec) - (1e9*tick.tv_sec + tick.tv_nsec);
    if(verbose>0) printf("GPU: first stage time: %.2f ms, second stage time: %.2f ms, total time: %.2f ms\n",
                            elapsed_proc_t/1e6, elapsed_t/1e6, (elapsed_proc_t+elapsed_t)/1e6);
    //if(verbose>0) printf("GPU: total time (includes data transfers): %f ms\n", elapsed_t/1e6);


}



#if TEST == 1
int main(){
    int16_t *fake_samples;//[NANTS][NRFSAMPS+DECIM_RATE][2];
    int16_t **vec_ptrs;

    float** ant_channels[NFREQS];
    float* float_vecs[NFREQS][NANTS];
    float*** output_buffer = (float***) malloc(NFREQS*sizeof(float**));
    for (int ifreq=0; ifreq<NFREQS; ifreq++){
        ant_channels[ifreq] = (float**) malloc(NANTS*sizeof(float*));
        for (int iant=0; iant<NANTS; iant++){
            float_vecs[ifreq][iant] = (float*) malloc(NRFSAMPS/DECIM_RATE * 2*sizeof(float));
        }
        ant_channels[ifreq] = float_vecs[ifreq];
    }
    output_buffer = ant_channels;
            

    fake_samples = (int16_t*) malloc(NANTS*(NRFSAMPS)*2*sizeof(int16_t));

    for(int iant=0; iant<NANTS; iant++){
        for (int i=0;i<NRFSAMPS;i++){
            fake_samples[iant*((NRFSAMPS)*2)+2*i] = i;//sin(M_PI*i/4);
            fake_samples[iant*((NRFSAMPS)*2)+2*i+1] = 0;
            printf("fake_samples %i: %i\n", fake_samples[iant*((NRFSAMPS)*2)+2*i]);
        }
    }

    vec_ptrs = (int16_t**) malloc(NANTS*sizeof(int16_t *));
    printf("building vec ptrs\n");

    for (int i=0; i<NANTS; i++)
        *(vec_ptrs+i) = fake_samples + i*((NRFSAMPS)*2);

    printf("built vec ptrs\n");

    struct timespec tick, tock;
    float elapsed;
    float center_freqs[NFREQS];
    for (int i=0;i<NFREQS;i++){
        //center_freqs[i] = M_PI/4;
        center_freqs[i] = 0;
    }
    clock_gettime(CLOCK_MONOTONIC, &tick);
    rx_process_gpu(
        vec_ptrs,
        output_buffer,
        NRFSAMPS,
        NRFSAMPS/DECIM_RATE,
        NFREQS,
        NANTS,
        NRFSAMPS,
        2,
        &center_freqs[0]);
    clock_gettime(CLOCK_MONOTONIC, &tock);
    elapsed = (1e9*tock.tv_sec + tock.tv_nsec) - (1e9*tick.tv_sec + tick.tv_nsec);

    free(fake_samples);

    //here is the output
    //for (int ifreq=0; ifreq<NFREQS; ifreq++){

    //    printf("copying %i samples back to client buffer\n",NRFSAMPS/DECIM_RATE);
    //    printf("rx_process_gpu: output samples head\n");

    //    for(int i=0;i<NRFSAMPS/DECIM_RATE;i+=1){
    //        for (int iant=0; iant<NANTS; iant++){
    //                printf("output %i, %i: (%f, %f)\t",ifreq, iant,
    //                     (float) output_buffer[ifreq][iant][2*i],
    //                     (float) output_buffer[ifreq][iant][2*i+1]);
    //                    //*(client_buff_ptr+(ifreq*nbb_samples*nants*2) + iant*nbb_samples*2 + 2*i),
    //                    //*(client_buff_ptr+(ifreq*nbb_samples*nants*2) + iant*nbb_samples*2 + 2*i + 1));
    //        }
    //        printf("\n");
    //    }
    //  printf("\n\n");
    //}


    //for (int ifreq=0; ifreq<NFREQS; ifreq++){
    //    for(int i=0;i<NRFSAMPS/DECIM_RATE;i++){
    //        for (int j=0; j<NANTS; j++)
    //            printf("output %i,%i: (%f,%f)\t",ifreq,j,
    //                output_buffer[ifreq][j][i][0],
    //                output_buffer[ifreq][j][i][1]);
    //    printf("\n");
    //    }
    //  printf("\n\n");
    //}

    printf("processesd fake samples\n");
    printf("elapsed time: %f ms\n", elapsed/1e6);

    return 0;
}
#endif
