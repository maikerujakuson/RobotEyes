#include <stdio.h>
//#include <winsock2.h>
#include <WinSock2.h>
int
main()
{
	SOCKET sock;
	struct sockaddr_in dest1;
	char buf[1024];
	WSADATA wsaData;

	WSAStartup(MAKEWORD(2, 0), &wsaData);

	sock = socket(AF_INET, SOCK_DGRAM, 0);

	dest1.sin_family = AF_INET;

	dest1.sin_addr.S_un.S_addr = inet_addr("127.0.0.1");

	dest1.sin_port = htons(11111);

	memset(buf, 0, sizeof(buf));
	_snprintf(buf, sizeof(buf), "data to port 11111");
	sendto(sock,
		buf, strlen(buf), 0, (struct sockaddr *)&dest1, sizeof(dest1));

	closesocket(sock);

	WSACleanup();

	return 0;
}