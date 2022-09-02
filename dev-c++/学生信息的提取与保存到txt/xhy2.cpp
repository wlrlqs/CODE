#include<stdio.h>
#include<math.h>
int main()
{
	char a[20];
	int b[20];
	scanf("%s", &a);
	int i=0,m=0,n=0,k=-1,h=0,sum=0;
	while(a[i]!=',')
	{
		if(a[i]>='0'&&a[i]<='9')
		{
			
			k++;
			b[h]=a[i]-48;
			h++;
		}
		i++;
	}
	for(int c=0;c<=h;c++)
	{
		
		m+=b[c]*pow(10,k);
		k--;
	}
	k=-1;
	h=0;
	while(a[i]!='\0')
	{
		if(a[i]>='0'&&a[i]<='9')
		{
			
			k++;
			b[h]=a[i]-48;
			h++;
		}
		i++;
	}
	for(int c=0;c<=h;c++)
	{
		
		n+=b[c]*pow(10,k);
		k--;
	}
	printf("满足条件的数有：");
	for(h=m+1;h<=n;h++)
	{
		if(h%2==0)
		{
			if(h%4!=0)
			{
				printf("%d  ",h);
				sum+=h;
			}
		}
	}
	printf("\n总和为：%d",sum);
	return 0;
}
