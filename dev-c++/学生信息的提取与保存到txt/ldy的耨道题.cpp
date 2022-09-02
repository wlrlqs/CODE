#include <stdio.h>
#pragma warning (disable:4996)
int main()
{
	int m, n, b, i, d, e, g, a;

	i = 0;

	g = 0;

	d = 1;

	scanf("%d %d", &m, &n);

	if (m == 0) { m = m + 1; }

	for (; m <= n; m++) {

		b = m * m; e = m;

		while (e != 0) { e = e / 10; i++; }

		for (a = 1; a <= i; a++) 
		{

			d = d * 10;	
					
		}

		if (b % d == m)

		{
			printf("%d\n", m);

			g = 1;
		}

		i = 0;
		d = 1;

	}
}
