#include<stdio.h>
int main(int argc, char* argv[])
{
    int k;
    printf("%d\n", argc);
    for (k = 0; k < argc; k++)
    {
        printf("%d:", k);
        puts(argv[k]);
    }
    getchar();
    return 0;
}
