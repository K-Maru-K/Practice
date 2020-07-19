#define _USE_MATH_DEFINES
#include <iostream>
#include <thread>
#include <cstdio>
#include <cstdint>
#include <winsock.h>
#include <fstream>
#include <string>

#pragma warning(disable : 4996)

#include "rplidar.h"
#include <math.h>

#include <fstream>   // ifstream, ofstream
#include <sstream>   // istringstream

#include"matplotlibcpp.h"
#include<conio.h>

namespace plt = matplotlibcpp;
//using namespace std;
using namespace rp::standalone::rplidar;

int main() {

	std::ofstream ofs("C:\\Users\\sens\\Desktop\\test.txt", std::ios::app);

	WSAData wsaData;

	SOCKET sock;
	struct sockaddr_in addr, send_addr;

	char buf[2048];

	int err = 0;

	err = WSAStartup(MAKEWORD(2, 0), &wsaData);

	if (err != 0) {
		switch (err) {
		case WSASYSNOTREADY:
			printf("WSASYSNOTREADY\n");
			break;
		case WSAVERNOTSUPPORTED:
			printf("WSAVERNOTSUPPORTED\n");
			break;
		case WSAEINPROGRESS:
			printf("WSAEINPROGRESS\n");
			break;
		case WSAEPROCLIM:
			printf("WSAEPROCLIM\n");
			break;
		case WSAEFAULT:
			printf("WSAEFAULT\n");
			break;
		}
	}

	sock = socket(AF_INET, SOCK_DGRAM, 0);

	addr.sin_family = AF_INET;
	addr.sin_port = htons(52525);
	addr.sin_addr.S_un.S_addr = INADDR_ANY;
	//addr.sin_addr.S_un.S_addr = inet_addr(std::string("192.168.4.1").c_str());

	err = bind(sock, (struct sockaddr*)&addr, sizeof(addr));
	printf("Bind Error : %d\n", err);
	printf("Error : %d\n", WSAGetLastError());

	memset(buf, 0, sizeof(buf));
	while (1) {
		if (_kbhit()) {
			sendto(sock, "Re\n", strlen("Re\n") + 1, 0, (struct sockaddr*)&addr, sizeof(addr));
			printf("Reset send!\n");
			break;
		}
	}

	while (1) {

		if (_kbhit()) {
			sendto(sock, "St\n", strlen("St\n") + 1, 0, (struct sockaddr*)&addr, sizeof(addr));
			printf("Start send!\n");
			ofs << "----------------------------\n";
		}

		int len = recv(sock, buf, sizeof(buf), 0);

		if (len != 0) {
			ofs << buf << std::endl;
			printf("Get!\n");
		}
	}

	closesocket(sock);

	WSACleanup();

	return 0;
}
