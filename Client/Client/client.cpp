#include <stdio.h>
#include <winsock2.h>
#include <ws2tcpip.h>

#define PORT 9876 //�T�[�o�[�v���O�����ƃ|�[�g�ԍ������킹�Ă�������

int main() {
	// IP �A�h���X�C�|�[�g�ԍ��C�\�P�b�g�Csockaddr_in �\����
	char destination[32];
	int dstSocket;
	struct sockaddr_in dstAddr;

	// �e��p�����[�^
	char buffer[1024];

	// Windows �̏ꍇ
	WSADATA data;
	WSAStartup(MAKEWORD(2, 0), &data);

	// �����A�h���X�̓��͂Ƒ��镶���̓���
	printf("�T�[�o�[�}�V����IP�́H:");
	scanf("%s", destination);

	// sockaddr_in �\���̂̃Z�b�g
	memset(&dstAddr, 0, sizeof(dstAddr));
	dstAddr.sin_port = htons(PORT);
	dstAddr.sin_family = AF_INET;
	dstAddr.sin_addr.s_addr = inet_addr(destination);

	// �\�P�b�g�̐���
	dstSocket = socket(AF_INET, SOCK_STREAM, 0);

	//�ڑ�
	if (connect(dstSocket, (struct sockaddr *) &dstAddr, sizeof(dstAddr))) {
		printf("%s�@�ɐڑ��ł��܂���ł���\n", destination);
		return(-1);
	}
	printf("%s �ɐڑ����܂���\n", destination);
	printf("�K���ȃA���t�@�x�b�g����͂��Ă�������\n");

	while (1) {
		scanf("%s", buffer);
		//�p�P�b�g�̑��M
		send(dstSocket, buffer, 1024, 0);
		//�p�P�b�g�̎�M
		recv(dstSocket, buffer, 1024, 0);
		printf("�� %s\n\n", buffer);
	}

	// Windows �ł̃\�P�b�g�̏I��
	closesocket(dstSocket);
	WSACleanup();
	return(0);
}