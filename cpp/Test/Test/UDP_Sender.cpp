#define _USE_MATH_DEFINES
#include <iostream>
#include <thread>
#include <cstdio>
#include <cstdint>
#include <winsock.h>

#pragma warning(disable : 4996)

#include "rplidar.h"
#include <math.h>

#include <fstream>   // ifstream, ofstream
#include <sstream>   // istringstream

#include"matplotlibcpp.h"
#include<conio.h>

int main() {

	WSAData wsaData_pan, wsaData_tilt;

	SOCKET sock_pan, sock_tilt;
	struct sockaddr_in addr_pan, addr_tilt;

	int err_pan = 0, err_tilt = 0;

	err_pan = WSAStartup(MAKEWORD(2, 0), &wsaData_pan);
	err_tilt = WSAStartup(MAKEWORD(2, 0), &wsaData_tilt);

	if (err_pan != 0) {
		switch (err_pan) {
		case WSASYSNOTREADY:
			printf("Pan: WSASYSNOTREADY\n");
			break;
		case WSAVERNOTSUPPORTED:
			printf("Pan: WSAVERNOTSUPPORTED\n");
			break;
		case WSAEINPROGRESS:
			printf("Pan: WSAEINPROGRESS\n");
			break;
		case WSAEPROCLIM:
			printf("Pan: WSAEPROCLIM\n");
			break;
		case WSAEFAULT:
			printf("Pan: WSAEFAULT\n");
			break;
		}
	}

	if (err_tilt != 0) {
		switch (err_tilt) {
		case WSASYSNOTREADY:
			printf("Tilt: WSASYSNOTREADY\n");
			break;
		case WSAVERNOTSUPPORTED:
			printf("Tilt: WSAVERNOTSUPPORTED\n");
			break;
		case WSAEINPROGRESS:
			printf("Tilt: WSAEINPROGRESS\n");
			break;
		case WSAEPROCLIM:
			printf("Tilt: WSAEPROCLIM\n");
			break;
		case WSAEFAULT:
			printf("Tilt: WSAEFAULT\n");
			break;
		}
	}


	sock_pan = socket(AF_INET, SOCK_DGRAM, 0);

	addr_pan.sin_family = AF_INET;
	addr_pan.sin_port = htons(7777);
	addr_pan.sin_addr.S_un.S_addr = inet_addr("192.168.11.3");

	sock_tilt = socket(AF_INET, SOCK_DGRAM, 0);

	addr_tilt.sin_family = AF_INET;
	addr_tilt.sin_port = htons(7774);
	addr_tilt.sin_addr.S_un.S_addr = inet_addr("192.168.11.4");


	int count = 0;

	while (true) {

		if (_kbhit()) {
			sendto(sock_pan, "123.45", 6, 0, (struct sockaddr*)&addr_pan, sizeof(addr_pan));
			sendto(sock_tilt, "678.9", 5, 0, (struct sockaddr*)&addr_tilt, sizeof(addr_tilt));
			count++;

			printf("Count\n");

			if (count > 10)
				break;
		}

	}

	closesocket(sock_pan);
	closesocket(sock_tilt);

	WSACleanup();

}

//int main() {
//
//	WSAData wsaData;
//
//	SOCKET sock;
//	struct sockaddr_in addr;
//
//	char buf[2048];
//
//	int err = 0;
//
//	err = WSAStartup(MAKEWORD(2, 0), &wsaData);
//
//	if (err != 0) {
//		switch (err) {
//		case WSASYSNOTREADY:
//			printf("WSASYSNOTREADY\n");
//			break;
//		case WSAVERNOTSUPPORTED:
//			printf("WSAVERNOTSUPPORTED\n");
//			break;
//		case WSAEINPROGRESS:
//			printf("WSAEINPROGRESS\n");
//			break;
//		case WSAEPROCLIM:
//			printf("WSAEPROCLIM\n");
//			break;
//		case WSAEFAULT:
//			printf("WSAEFAULT\n");
//			break;
//		}
//	}
//
//	sock = socket(AF_INET, SOCK_DGRAM, 0);
//
//	addr.sin_family = AF_INET;
//	addr.sin_port = htons(7777);
//	addr.sin_addr.S_un.S_addr = inet_addr("192.168.11.3");
//
//	//err = bind(sock, (struct sockaddr*)&addr, sizeof(addr));
//	//printf("Bind Error : %d\n", err);
//
//	//memset(buf, 0, sizeof(buf));
//
//	//int saveLen = 0;
//	//unsigned char saveData[100] = "";
//	//char data = 0;
//
//	int count = 0;
//
//	while (true) {
//
//		if (_kbhit()) {
//			sendto(sock, "123.45", 6, 0, (struct sockaddr*)&addr, sizeof(addr));
//			count++;
//
//			printf("Count\n");
//
//			if (count > 10)
//				break;
//		}
//
//	}
//
//	closesocket(sock);
//
//	WSACleanup();
//
//}