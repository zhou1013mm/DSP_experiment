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


// ps:
//      1. 三项求和不用sum函数不然太绕了，一种调试方式是把自己看着都绕的地方试图简化，比如声明一些辅助变量
//      2. 注意循环队列的方向（时间前后）
float* IIR(float x_n, float * iir_coef, int iir_coef_length, float * data_buffer, int * data_buffer_cnt, float gain)
{
    float* buf = (float*)malloc(seq_length*sizeof(float));
    float* next = (float*)malloc(seq_length*sizeof(float));
    float b0, b1, b2, a1, a2;
    float x1, x2, y1, y2;
    float in, out;
    
    data_buffer[*data_buffer_cnt] = x_n;
    for(int i=0; i<seq_length; i++){
        buf[i] = data_buffer[i];
    }
    
    // 4 sos concatenated
    for(int i=0; i<iir_level/2 ; i++){
        b0 = iir_coef[i*6+0]; b1 = iir_coef[i*6+1]; b2 = iir_coef[i*6+2];
        a1 = iir_coef[i*6+4]; a2 = iir_coef[i*6+5];
        x1 = 0, x2 = 0;
        y1 = 0, y2 = 0;
            
        for(int k=0; k<seq_length; k++){
                next[k] = 0;
        }

        //for(int j=seq_length-1; j>=0; j--){
        for(int j=0; j<seq_length; j++){
            // if(j+1 >= seq_length) x1 = 0;
            // if(j+2 >= seq_length) x2 = 0;
            //buf[j] = b0*input[j] + b1*input[(j+1+seq_length)&(seq_length-1)] + b2*input[(j+2+seq_length)&(seq_length-1)]
            //            - a1*buf[(j+1+seq_length)&(seq_length-1)] - a2*buf[(j+2+seq_length)&(seq_length-1)];
            in = buf[j];
            out = b0*in + b1*x1 + b2*x2 - a1*y1 - a2*y2;
            
            // update history
            x2 = x1; x1 = in;
            y2 = y1; y1 = out;
            
            next[j] = out;
        }
        // update the next buf with this sos's result (named `next`) 
        for(int k=0; k<seq_length; k++){
                buf[k] = next[k];
        }
    }
    // apply gain
    for(int i=0; i<seq_length; i++){
        buf[i] *= gain; 
    }
    free(next);
    
	return buf;
}


// test modules 

void test_FIR(){
    float x_n = 1 ;
    float* y_n;
    float fir_coef[] = { -0.0014,   -0.0013,   -0.0011,   -0.0006,    0.0009,    0.0036,    0.0082,    0.0148,    0.0234,    0.0337,    0.0452,    0.0569,    0.0680,    0.0774,    0.0843,    0.0879,    0.0879,    0.0843,    0.0774,
                            0.0680,    0.0569,    0.0452,    0.0337,    0.0234,    0.0148,    0.0082,    0.0036,    0.0009,   -0.0006,   -0.0011,   -0.0013,   -0.0014};
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
    // float iir_coef_a[] = {1.0000, -3.9838, 7.5362, -8.5998, 6.4002, -3.1560, 1.0017, -0.1863, 0.0155};
    // float iir_coef_b[] = {0.0001, 0.0009, 0.0030, 0.0060, 0.0076, 0.0060, 0.0030, 0.0009, 0.0001};
    float sos[] = {1.0000,    2.0003,    0.9999,    1.0000,   -1.5407,    0.5951,
                    1.0000,    2.0305,    1.0309,    1.0000,   -1.5897,    0.6458,
                    1.0000,    1.9997,    1.0001,    1.0000,   -1.6890,    0.7486,
                    1.0000,    1.9695,    0.9700,    1.0000,   -1.8390,    0.9039};
    float g = 4.6020e-08;
    int sos_length = (int)(sizeof(sos)/sizeof(sos[0]));
    float* data_buffer = (float*)malloc(seq_length*sizeof(float));
    for(int i=0; i<seq_length; i++){
        data_buffer[i] = 0;
    } 

    // test
    int data_buffer_cnt = 0;
    y_n = IIR(x_n, sos, sos_length, data_buffer, &data_buffer_cnt, g);
    printf("y_n= [");
    for(int i=0; i<seq_length; i++){
        printf("%f, ", y_n[i]);
    }
    printf("]\n\n");
    //printf("Length of sos array: %d\n", length(sos)); -> 2 数组退化为指针
    //printf("Length of sos array: %d\n", sos_length); // -> 24

    free(y_n);
    free(data_buffer);
}

int main(){

    test_FIR();
    test_IIR();

    return 0;
}
