#include <stdio.h>
#include <iostream>
#include <WinSock2.h>
#include <WS2tcpip.h>


// Port number 
#define PORT 9876 

int main() {
	int i;
	// ポート番号，ソケット
	int srcSocket;  // 自分
	int dstSocket;  // 相手

	// Structure to store IP address and port of server
	struct sockaddr_in srcAddr;
	// Structure to store IP address and port of client
	struct sockaddr_in dstAddr;
	// Variable for size of client sockaddr_in
	int dstAddrSize = sizeof(dstAddr);
	// Variable for status
	int status;
	// 
	int numrcv;
	char buffer[1024];

	// Structure to contain information about the Window socket implementation
	WSADATA data;
	// Initialize DLL to use WinSock API
	WSAStartup(MAKEWORD(2, 0), &data);
	// Initialize srcAddr with 0
	memset(&srcAddr, 0, sizeof(srcAddr));
	// Set port number to srcAddr
	srcAddr.sin_port = htons(PORT);
	// What are below two lines doing?
	srcAddr.sin_family = AF_INET;
	srcAddr.sin_addr.s_addr = htonl(INADDR_ANY);
	// Create endpoint for communication and Set endpoint to srcSocket
	srcSocket = socket(AF_INET, SOCK_STREAM, 0);
	// Assgin the address specified by addr to the socket
	bind(srcSocket, (struct sockaddr *) &srcAddr, sizeof(srcAddr));
	// Accept incoming connection request
	listen(srcSocket, 1);

	while (1) { 
		// Wait for connection from client
		printf("Wait connection...\nRun the client program.\n");
		dstSocket = accept(srcSocket, (struct sockaddr *) &dstAddr, &dstAddrSize);
		printf("Received the connection from %s\n", inet_ntoa(dstAddr.sin_addr));
		std::cout << "ochinchin" << std::endl;
		while (1) {
			// Receive packet
			numrcv = recv(dstSocket, buffer, sizeof(char) * 1024, 0);
			if (numrcv == 0 || numrcv == -1) {
				status = closesocket(dstSocket); break;
			}
			std::cout << "omanman" << std::endl;
			printf("変換前 %s", buffer);
			for (i = 0; i< numrcv; i++) { // bufの中の小文字を大文字に変換
										  //if(isalpha(buffer[i])) 
				buffer[i] = toupper(buffer[i]);
			}
			// パケットの送信
			send(dstSocket, buffer, sizeof(char) * 1024, 0);
			printf("→ 変換後 %s \n", buffer);
		}
	}
	// Do clean up 
	WSACleanup();

	return(0);
}
