
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

	// 受信ソケットを2つ作ります
	sock1 = socket(AF_INET, SOCK_DGRAM, 0);

	addr1.sin_family = AF_INET;

	addr1.sin_addr.S_un.S_addr = inet_addr("127.0.0.1");

	// 2つの別々のポートで待つために別のポート番号をそれぞれ設定します
	addr1.sin_port = htons(11111);

	// 2つの別々のポートで待つようにbindします
	bind(sock1, (struct sockaddr *)&addr1, sizeof(addr1));

	// fd_setの初期化します
	FD_ZERO(&readfds);

	// selectで待つ読み込みソケットとしてsock1を登録します
	FD_SET(sock1, &readfds);

	struct timeval timeout;
	timeout.tv_sec = 0;
	timeout.tv_usec = 0;
	// 無限ループです
	// このサンプルでは、この無限ループを抜けません
	while (1) {
		std::cout << "Fuck you" << std::endl;
		// 読み込み用fd_setの初期化
		// selectが毎回内容を上書きしてしまうので、毎回初期化します
		memcpy(&fds, &readfds, sizeof(fd_set));

		// fdsに設定されたソケットが読み込み可能になるまで待ちます
		select(0, &fds, NULL, NULL, &timeout);

		// sock1に読み込み可能データがある場合
		if (FD_ISSET(sock1, &fds)) {
			// sock1からデータを受信して表示します
			memset(buf, 0, sizeof(buf));
			recv(sock1, buf, sizeof(buf), 0);
			printf("%s\n", buf);
		}
	}

	// このサンプルでは、ここへは到達しません
	closesocket(sock1);

	WSACleanup();

	return 0;
}