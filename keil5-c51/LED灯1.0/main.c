#include<reg52.h>
sbit led0=P0^0;

void main()
{
	P0=0x00;
	led0=1;
	while(1)
	{}
}