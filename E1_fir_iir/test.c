#include <stdio.h>
#include <stdlib.h>
int main() {
    int* a = malloc(2 * sizeof(int));
    *(a+0) = 1;
    *(a+sizeof(int)) = 2;
    printf("%d\n", *(a+sizeof(int)/sizeof(int)));
    printf("%d", *(a+1));
    printf("int size:%zu\n",sizeof(int));
    printf("unsigned size:%zu\n",u);
    free(a);
    return 0;
}