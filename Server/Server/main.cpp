
#include <stdio.h>
#include <iostream>
#include <winsock2.h>

int
main()
{
	SOCKET sock1;
	struct sockaddr_in addr1;
	fd_set fds, readfds;
	char buf[2048];
	WSADATA wsaData;

	WSAStartup(MAKEWORD(2, 0), &wsaData);

	// ��M�\�P�b�g��2���܂�
	sock1 = socket(AF_INET, SOCK_DGRAM, 0);

	addr1.sin_family = AF_INET;

	addr1.sin_addr.S_un.S_addr = inet_addr("127.0.0.1");

	// 2�̕ʁX�̃|�[�g�ő҂��߂ɕʂ̃|�[�g�ԍ������ꂼ��ݒ肵�܂�
	addr1.sin_port = htons(11111);

	// 2�̕ʁX�̃|�[�g�ő҂悤��bind���܂�
	bind(sock1, (struct sockaddr *)&addr1, sizeof(addr1));

	// fd_set�̏��������܂�
	FD_ZERO(&readfds);

	// select�ő҂ǂݍ��݃\�P�b�g�Ƃ���sock1��o�^���܂�
	FD_SET(sock1, &readfds);

	struct timeval timeout;
	timeout.tv_sec = 0;
	timeout.tv_usec = 0;
	// �������[�v�ł�
	// ���̃T���v���ł́A���̖������[�v�𔲂��܂���
	while (1) {
		std::cout << "Fuck you" << std::endl;
		// �ǂݍ��ݗpfd_set�̏�����
		// select��������e���㏑�����Ă��܂��̂ŁA���񏉊������܂�
		memcpy(&fds, &readfds, sizeof(fd_set));

		// fds�ɐݒ肳�ꂽ�\�P�b�g���ǂݍ��݉\�ɂȂ�܂ő҂��܂�
		select(0, &fds, NULL, NULL, &timeout);

		// sock1�ɓǂݍ��݉\�f�[�^������ꍇ
		if (FD_ISSET(sock1, &fds)) {
			// sock1����f�[�^����M���ĕ\�����܂�
			memset(buf, 0, sizeof(buf));
			recv(sock1, buf, sizeof(buf), 0);
			printf("%s\n", buf);
		}
	}

	// ���̃T���v���ł́A�����ւ͓��B���܂���
	closesocket(sock1);

	WSACleanup();

	return 0;
}