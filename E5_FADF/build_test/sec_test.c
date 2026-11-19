#include <stdint.h>
#pragma section(".bss")
volatile float big1[30000];
volatile float big2[100];
int main(void){ big1[0]=1.0f; big2[0]=2.0f; return 0; }
