#include <iostream>

using namespace std;

void trace(int A[], int n)
{
	for (int i = 0; i < n; i++) {
		if (i > 0) cout << " ";
		cout << A[i];
	}

	cout << endl;
}

int main()
{
	int n;
	int A[100];
	int key;

	// Input data
	cin >> n;
	for (int i = 0; i < n; i++) {
		cin >> A[i];
	}

	trace(A, n);
	int j;
	for (int i = 1; i < n; i++) {
		key = A[i];
		j = i - 1;
		while (j >= 0 && A[j] > key) {
			A[j + 1] = A[j];
			j--;
		}
		A[j + 1] = key;
		trace(A, n);
	}

	return 0;
}