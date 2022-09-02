#include<stdio.h>
int main()
{
	FILE* fp = NULL;
	fp = fopen("data.txt", "wt");
	fprintf(fp,"78 65\n80 89\n90 85\n95 90\n90 85\n75 80\n60 60\n95 80\n90 85\n75 75");
	fclose(fp);
	fp = fopen("data.txt", "r");
	if (fp == NULL)
	{
		printf("fail to open the file");
	}
	char ch[100];
	char* p = ch;
	char a=0;
	while (a != EOF)
	{
		a = fgetc(fp);
		if (a != ' ' && a != '\n')
		{
			*p = a;
			p++;
		}
	}
	fclose(fp);
	int ah[100];
	int b = 0, c = 0;
	for (int i = 0, k = 0; i < 40; i = i + 2, k++)
	{
		b = (ch[i] - '0') * 10 + (ch[i + 1] - '0');
		ah[k] = b;

	}
	float cj[100];
	fp = fopen("T_data.txt", "wt");
	for (int i = 0, k = 0; i < 20; i += 2, k++)
	{
		cj[k] = ah[i] * 0.2 + ah[i + 1] * 0.8;
		printf("%g\n", cj[k]);
		fprintf(fp, "%g\n", cj[k]);
	}
	fclose(fp);
	return 0;
}
