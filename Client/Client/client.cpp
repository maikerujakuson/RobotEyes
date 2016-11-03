#include <stdio.h>
#include <winsock2.h>
#include <Eigen\Dense>
#include <string>
#include <iostream>
#include <sstream>
int
main()
{
	// Socket for sending
	SOCKET sock_send;
	// Socket for receiving
	SOCKET sock_recv;
	struct sockaddr_in addr_send, addr_recv;
	char buf[2048];
	WSADATA wsaData;
	// Variable to use select()
	fd_set fds, readfds;

	// Vec
	Eigen::Vector4f vec;
	
	WSAStartup(MAKEWORD(2, 0), &wsaData);

	sock_recv = socket(AF_INET, SOCK_DGRAM, 0);
	sock_send = socket(AF_INET, SOCK_DGRAM, 0);

	addr_send.sin_family = AF_INET;
	addr_recv.sin_family = AF_INET;

	addr_send.sin_addr.S_un.S_addr = inet_addr("127.0.0.1");
	addr_recv.sin_addr.S_un.S_addr = inet_addr("127.0.0.1");

	addr_send.sin_port = htons(11111);
	addr_recv.sin_port = htons(22222);

	// Bind receiver socket 
	bind(sock_recv, (struct sockaddr *)&addr_recv, sizeof(addr_recv));

	memset(buf, 0, sizeof(buf));
	_snprintf(buf, sizeof(buf), "Give me object information!!!!");
	sendto(sock_send,
		buf, strlen(buf), 0, (struct sockaddr *)&addr_send, sizeof(addr_send));

	closesocket(sock_send);

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
			std::string s = buf;
			std::stringstream ss(s);
			std::string item;
			int i = 0;
			while (std::getline(ss, item, ' ') && !item.empty()) {
				vec(i++) = stof(item);
			}
			std::cout << vec.x() << std::endl;
			std::cout << vec.y() << std::endl;
			std::cout << vec.z() << std::endl;
			std::cout << s << std::endl;
			//printf("%s\n", buf);
			break;
		}
	}

	WSACleanup();

	return 0;
}