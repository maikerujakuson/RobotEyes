#define NOMINMAX
#include <stdio.h>
#include <iostream>
#include <WinSock2.h>
#include <winsock2.h>

int
main()
{
	// Variable Socket	
	SOCKET sock;
	// Variable to store port and IP numbers
	struct sockaddr_in addr1;
	// ???
	fd_set fds, readfds;
	// Variable for received text
	char buf[2048];
	// ???
	WSADATA wsaData;

	// Start 
	WSAStartup(MAKEWORD(2, 0), &wsaData);

	// Initialize receive socekt	
	sock = socket(AF_INET, SOCK_DGRAM, 0);

	// ???
	addr1.sin_family = AF_INET;

	// Bind IP address
	addr1.sin_addr.S_un.S_addr = inet_addr("127.0.0.1");
	
	// Bind	port number 
	addr1.sin_port = htons(11111);

	// Bind sockaddr to sock 
	bind(sock, (struct sockaddr *)&addr1, sizeof(addr1));

	// Initialize readfds
	FD_ZERO(&readfds);

	// Register sock with readfds
	FD_SET(sock, &readfds);

	// Set timeout
	struct timeval timeout;
	timeout.tv_sec = 0;
	timeout.tv_usec = 0;

	// Reapt listen
	while (1) {
		// Clear fds
		memcpy(&fds, &readfds, sizeof(fd_set));

		// Wait untile fds becomes readable in timeout time
		select(0, &fds, NULL, NULL, &timeout);

		// Cheack readable data is in sock 
		if (FD_ISSET(sock, &fds)) {
			//	Recieve data from sock
			memset(buf, 0, sizeof(buf));
			recv(sock, buf, sizeof(buf), 0);
			// Print data
			printf("%s\n", buf);
		}
	}

	// Close sock 
	closesocket(sock);
	// Clean up socket communication
	WSACleanup();

	return 0;
}