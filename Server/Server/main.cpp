#include <stdio.h>
#include <winsock2.h>
#include <Eigen\Dense>

int
main()
{
	// Socket for receiving
	SOCKET sock_recv;
	// Socket for sending
	SOCKET sock_send;
	// Structure to store socket adress
	struct sockaddr_in addr_recv, addr_send;
	// Variable to use select()
	fd_set fds, readfds;
	// Variable for text
	char buf[2048];
	// Structure to store Window socket information
	WSADATA wsaData;

	Eigen::Vector4f vec;
	vec.x() = vec.y() = vec.z() = 1.11111f;
	
	// ???
	WSAStartup(MAKEWORD(2, 0), &wsaData);

	// Make socket object
	sock_recv = socket(AF_INET, SOCK_DGRAM, 0);
	sock_send = socket(AF_INET, SOCK_DGRAM, 0);

	addr_recv.sin_family = AF_INET;
	addr_send.sin_family = AF_INET;

	addr_recv.sin_addr.S_un.S_addr = inet_addr("127.0.0.1");
	addr_send.sin_addr.S_un.S_addr = inet_addr("127.0.0.1");

	// Set different port number to each socket
	addr_recv.sin_port = htons(11111);
	addr_send.sin_port = htons(22222);

	// Bind receiver socket 
	bind(sock_recv, (struct sockaddr *)&addr_recv, sizeof(addr_recv));

	// fd_set�̏��������܂�
	FD_ZERO(&readfds);

	// select�ő҂ǂݍ��݃\�P�b�g�Ƃ���sock1��o�^���܂�
	FD_SET(sock_recv, &readfds);

	// Do listen untile recieving data
	while (1) {
		// Initialize fds
		memcpy(&fds, &readfds, sizeof(fd_set));

		// fds�ɐݒ肳�ꂽ�\�P�b�g���ǂݍ��݉\�ɂȂ�܂ő҂��܂�
		select(0, &fds, NULL, NULL, NULL);

		// sock1�ɓǂݍ��݉\�f�[�^������ꍇ
		if (FD_ISSET(sock_recv, &fds)) {
			// sock1����f�[�^����M���ĕ\�����܂�
			memset(buf, 0, sizeof(buf));
			recv(sock_recv, buf, sizeof(buf), 0);
			printf("%s\n", buf);
			break;
		}
	}

	// Sending data
	memset(buf, 0, sizeof(buf));
	sprintf(buf, "%f %f %f", vec.x(),vec.y(),vec.z());
	sendto(sock_send,
		buf, strlen(buf), 0, (struct sockaddr *)&addr_send, sizeof(addr_send));
	
	// ���̃T���v���ł́A�����ւ͓��B���܂���
	closesocket(sock_recv);
	closesocket(sock_send);

	WSACleanup();

	return 0;
}
