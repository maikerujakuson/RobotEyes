#include <iostream>
#include <string>

using namespace std;

int main(void)
{
	// Variable for result
	unsigned char x = 0x00;
	int y = 1;

	// Bit operation of x and y
	if (y)
		x = x | 1;
	else
		x = x | 0;

	x = x << 2;

	cout << (int)x << endl;

	return 0;

}