#include <iostream>
#include <string>
int S[1000];

int main(void)
{
	int a = 10;
	int interval = 1;

	while (interval < a / 3)
		interval = interval * 3 + 1;

	std::cout << interval << std::endl;
}