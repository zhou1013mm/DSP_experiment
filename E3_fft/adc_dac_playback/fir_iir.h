#include<stdlib.h>
#include"adc_dac_playback.h"

#define fir_level 32
#define iir_level 8

#ifndef NUM_CHANNELS
#define NUM_CHANNELS 4
#endif

#ifndef AUDIO_BUFFER_SIZE
#define AUDIO_BUFFER_SIZE (512*4)
#endif

#define seq_length ((int)(AUDIO_BUFFER_SIZE / (sizeof(float) * NUM_CHANNELS)))

float* FIR_4C(float * fir_coef, float * data_buffer, int * data_buffer_cnt)
{
    float* y_n  = (float*)malloc(AUDIO_BUFFER_SIZE); // note: malloc uses bytes, AUDIO_BUFFER_SIZE is in bytes

    for(int c = 0; c < NUM_CHANNELS; c++){
        for(int n = 0; n < seq_length; n++){
            float acc = 0.0f;
            for(int k = 0; k < fir_level; k++){
                int x_index = n - k;
                float x = 0.0f;
                if(x_index >= 0){
                    x = data_buffer[x_index * NUM_CHANNELS + c];
                }
                acc += fir_coef[k] * x;
            }
            y_n[n * NUM_CHANNELS + c] = acc;
        }
    }

    if(data_buffer_cnt != NULL){
        *data_buffer_cnt = 0;
    }
    return y_n;
}

float* IIR_4C(float * iir_coef, int iir_coef_length, float * data_buffer, int * data_buffer_cnt, float gain)
{
    int n_sos = iir_coef_length / 6;
    if((iir_coef_length % 6) != 0){
        return NULL;
    }

    float* y_n = (float*)malloc(AUDIO_BUFFER_SIZE);
    float* stage_in = (float*)malloc(seq_length * sizeof(float));
    float* stage_out = (float*)malloc(seq_length * sizeof(float));

    for(int c = 0; c < NUM_CHANNELS; c++){
        for(int n = 0; n < seq_length; n++){
            stage_in[n] = data_buffer[n * NUM_CHANNELS + c];
        }

        for(int s = 0; s < n_sos; s++){
            float b0 = iir_coef[s * 6 + 0];
            float b1 = iir_coef[s * 6 + 1];
            float b2 = iir_coef[s * 6 + 2];
            float a1 = iir_coef[s * 6 + 4];
            float a2 = iir_coef[s * 6 + 5];
            float x1 = 0.0f, x2 = 0.0f;
            float y1 = 0.0f, y2 = 0.0f;

            for(int n = 0; n < seq_length; n++){
                float in = stage_in[n];
                float out = b0 * in + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;

                x2 = x1;
                x1 = in;
                y2 = y1;
                y1 = out;
                stage_out[n] = out;
            }

            for(int n = 0; n < seq_length; n++){
                stage_in[n] = stage_out[n];
            }
        }

        for(int n = 0; n < seq_length; n++){
            y_n[n * NUM_CHANNELS + c] = stage_in[n] * gain;
        }
    }

    free(stage_in);
    free(stage_out);
    if(data_buffer_cnt != NULL){
        *data_buffer_cnt = 0;
    }
    return y_n;
}