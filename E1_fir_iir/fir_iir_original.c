#include<stdio.h>
#include<stdlib.h>

#define fir_level 32
#define iir_level 8
#define seq_length 128

float sum(float* coef, float* buffer, int level, int* data_buffer_cnt){
    float s = 0;
    for(int i=0; i< level; i++){
        s += coef[i]*buffer[(*data_buffer_cnt + i) & (seq_length - 1)]; // &(level -1);
    }
    return s;
}

int length(float* array){
    // printf("Length of array: %d\n", (int)(sizeof(array)/sizeof(array[0])));
    // printf("%d", (int)(sizeof(array)));
    // printf("sizeof(array[0]): %d\n", (int)(sizeof(array[0])));
    return (int)(sizeof(array)/sizeof(array[0]));
}

float* FIR(float x_n, float * fir_coef , float * data_buffer, int * data_buffer_cnt)
{
    float* y_n  = (float*)malloc(seq_length*sizeof(float));

    data_buffer[*data_buffer_cnt] = x_n; 

    for(int i=0; i<seq_length; i++){
        y_n[i] = sum(fir_coef, data_buffer, fir_level, data_buffer_cnt);
        // Circular buffer index update
        *data_buffer_cnt = (*data_buffer_cnt - 1 + seq_length) & (seq_length - 1); 
    }
	return y_n;
}


// 我写的太通用，没必要
float* IIR_original(float x_n, float * iir_coef, int iir_coef_length, float * data_buffer, int * data_buffer_cnt, float gain)
{
    float* input = (float*)malloc(seq_length*sizeof(float)); 
    float* output;
    
    data_buffer[*data_buffer_cnt] = x_n;
    for(int i=0; i<seq_length; i++){
        input[i] = data_buffer[i];
    }
    
    // 注意input&output序列buffer的循环使用
    // 4 sos concatenated
    for(int i=0; i<iir_level/2 ; i++){        

        *data_buffer_cnt = 0;

        // malloc a new output buffer after each sos
        // float* output = (float*)malloc(seq_length*sizeof(float));
        output = (float*)malloc(seq_length*sizeof(float));
        for(int j=0; j<seq_length; j++){
            output[j] = 0;
        }
        for(int j=0; j< seq_length; j++){
            output[j] = sum(iir_coef+i*6, input, 3, data_buffer_cnt)-sum(iir_coef+i*6+4, output, 2, data_buffer_cnt);
            *data_buffer_cnt = (*data_buffer_cnt - 1 + seq_length) & (seq_length - 1);
        }

        if (i > 0) free(input);   // 第一轮的 input 是调用者给的，不能 free
        input = output;           // 本轮输出作为下一轮输入
        
        // y_n[i]= gain*(sum(iir_coef+j, data_buffer, 3, data_buffer_cnt))
        // - sum(iir_coef+j+4, y_n, 3, data_buffer_cnt);
        // for(int j=6; j< iir_coef_length; j+=6){
        //     y_n[i]= gain*(sum(iir_coef+j, data_buffer, 3, data_buffer_cnt)) // not y_n[i]+ =...
        //             - sum(iir_coef+j+4, y_n, 3, data_buffer_cnt);
        // }
        // Circular buffer index update
        //*data_buffer_cnt = (*data_buffer_cnt - 1 + seq_length) & (seq_length - 1); 
    }
    for(int i=0; i<seq_length; i++){
        output[i] = gain*output[i]; 
    }
	return output;
}

float* IIR(float x_n, float * iir_coef, int iir_coef_length,
           float * data_buffer, int * data_buffer_cnt, float gain)
{
    int N = seq_length;
    int n_sos = iir_coef_length / 6;

    // 把 x_n 写成脉冲序列：x[0] = x_n，其余为 0
    // （你原来的 data_buffer 现在只作为输入序列的临时存储）
    data_buffer[0] = x_n;
    for (int i = 1; i < N; i++) data_buffer[i] = 0;

    // 当前正在处理的信号，拷贝自 data_buffer，之后每级会替换
    float* buf = (float*)malloc(N * sizeof(float));
    float* next = (float*)malloc(N * sizeof(float));
    for (int i = 0; i < N; i++) buf[i] = data_buffer[i];

    // 逐级 SOS 处理
    for (int s = 0; s < n_sos; s++) {
        float b0 = iir_coef[s*6+0], b1 = iir_coef[s*6+1], b2 = iir_coef[s*6+2];
        // a0 = iir_coef[s*6+3] == 1，跳过
        float a1 = iir_coef[s*6+4], a2 = iir_coef[s*6+5];

        // 该级的历史：x[n-1], x[n-2] 和 y[n-1], y[n-2]
        float x1 = 0, x2 = 0;
        float y1 = 0, y2 = 0;

        for (int n = 0; n < N; n++) {
            float in  = buf[n];                       // 本级的输入
            float out = b0*in + b1*x1 + b2*x2         // 前馈
                      - a1*y1 - a2*y2;                // 反馈

            // 更新历史
            x2 = x1; x1 = in;
            y2 = y1; y1 = out;

            next[n] = out;                            // 本级输出
        }

        // 交换 buf 和 next，下一级从 next 的结果继续
        float* tmp = buf; buf = next; next = tmp;
    }

    free(next);   // 最后一次交换后，next 是没用的那块

    // 整体增益只乘一次
    for (int i = 0; i < N; i++) buf[i] *= gain;

    return buf;   // 调用者负责 free
}

// test modules 

void test_FIR(){
    float x_n = 1 ;
    float* y_n;
    float fir_coef[] = {   -0.0006,   -0.0018,   -0.0026,   -0.0016,    0.0024,    0.0083,    0.0118,    0.0068,   -0.0092,   -0.0297,   -0.0398,   -0.0223,    0.0310,    0.1111,    0.1925,    0.2437,    0.2437,    0.1925,    0.1111,     0.0310,   -0.0223,   -0.0398,   -0.0297,   -0.0092,    0.0068,    0.0118,    0.0083,    0.0024,   -0.0016,   -0.0026,   -0.0018,   -0.0006};
    float* data_buffer = (float*)malloc(seq_length*sizeof(float));
    for(int i=0; i<seq_length; i++){
        data_buffer[i] = 0;
    } 
    int data_buffer_cnt = 0;

    // test
    printf("y_n= [");
    // for(int i=0; i<seq_length; i++){
    //     data_buffer[(i-2+32)%32]=0;
    //     data_buffer[(i-1+32)%32]=1;    
    //     y_n = FIR(x_n, fir_coef, data_buffer, &data_buffer_cnt);
    //     printf("%f, ",y_n);
    // }
    // printf("]\n");
    // // for(int i=0; i<fir_level; i++){
    // //     printf("data_buffer[%d] = %f\n", i, data_buffer[i]);
    // // }   
    y_n = FIR(x_n, fir_coef, data_buffer, &data_buffer_cnt);
    for(int i=0; i<seq_length; i++){
        printf("%f, ", y_n[i]);
    }
    printf("]\n\n");

    free(y_n);
    free(data_buffer);
}

void test_IIR(){
    float x_n = 1 ;
    float* y_n;
    float* y_n_original;
    // float iir_coef_a[] = {1.0000, -3.9838, 7.5362, -8.5998, 6.4002, -3.1560, 1.0017, -0.1863, 0.0155};
    // float iir_coef_b[] = {0.0001, 0.0009, 0.0030, 0.0060, 0.0076, 0.0060, 0.0030, 0.0009, 0.0001};
    float sos[] = {
        1.0000,    2.0003,    0.9999,    1.0000,   -0.8351,    0.1810,
        1.0000,    2.0305,    1.0309,    1.0000,   -0.8906,    0.2595,
        1.0000,    1.9997,    1.0001,    1.0000,   -1.0153,    0.4359,
        1.0000,    1.9695,    0.9700,    1.0000,   -1.2428,    0.7575
    };
    float g = 1.0791e-04;
    int sos_length = (int)(sizeof(sos)/sizeof(sos[0]));
    float* data_buffer = (float*)malloc(seq_length*sizeof(float));
    for(int i=0; i<seq_length; i++){
        data_buffer[i] = 0;
    } 

    // test
    int data_buffer_cnt = 0;
    y_n_original = IIR_original(x_n, sos, sos_length, data_buffer, &data_buffer_cnt, g);
    y_n = IIR(x_n, sos, sos_length, data_buffer, &data_buffer_cnt, g);
    printf("y_n= [");
    for(int i=0; i<seq_length; i++){
        printf("%f, ", y_n[i]);
    }
    printf("]\n\n");
    printf("y_n_original= [");
    for(int i=0; i<seq_length; i++){
        printf("%f, ", y_n_original[i]);
    }
    printf("]\n\n");
    //printf("Length of sos array: %d\n", length(sos)); -> 2 数组退化为指针
    //printf("Length of sos array: %d\n", sos_length); // -> 24

    free(y_n);
    free(y_n_original);
    free(data_buffer);
}

int main(){

    test_FIR();
    test_IIR();

    return 0;
}
