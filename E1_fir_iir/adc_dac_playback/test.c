#include <stdio.h>
#include "filter.h"

int main(){
    float* pSrc = (float*)malloc(AUDIO_BUFFER_SIZE);
    float* pDst = (float*)malloc(AUDIO_BUFFER_SIZE);
    // initialize input buffer
    for(int i=0; i<(int)(AUDIO_BUFFER_SIZE / sizeof(float)); i++){
        pSrc[i] = 0; // or any initialization value
        if(i<=3){pSrc[i] = 1;}
    }

    float fir_coef[] = {   -0.0006,   -0.0018,   -0.0026,   -0.0016,    0.0024,    0.0083,    0.0118,    0.0068,   -0.0092,   -0.0297,   -0.0398,   -0.0223,    0.0310,    0.1111,    0.1925,    0.2437,    0.2437,    0.1925,    0.1111,     0.0310,   -0.0223,   -0.0398,   -0.0297,   -0.0092,    0.0068,    0.0118,    0.0083,    0.0024,   -0.0016,   -0.0026,   -0.0018,   -0.0006};
    int data_buffer_cnt = 0;
    pDst = FIR_4C(fir_coef, pSrc, &data_buffer_cnt);
    for(int i=0; i<(int)(AUDIO_BUFFER_SIZE / sizeof(float)); i++){
        printf("%f, ",pDst[i]);        
    }

    free(pSrc);
    free(pDst);
    return 0;
}