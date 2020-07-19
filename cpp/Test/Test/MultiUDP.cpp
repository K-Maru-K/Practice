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

namespace plt = matplotlibcpp;
//using namespace std;
using namespace rp::standalone::rplidar;

float dist = 0;
float whillOri[3] = { 0,0,0 };
float headOri[3] = { 0,0,0 };

float projOri[2] = { 0,0 };

bool appRunning = true;

#pragma region Definision

#define HEADER 0xAF
#define REPLACE 0xAE

#define MAXSIZE 60

#define XSIGN 'x'
#define YSIGN 'y'
#define ZSIGN 'z'

#define PITCHSIGN 'p'
#define YAWSIGN 't'
#define ROLLSIGN 'r'

#define MEDIANSIZE 5

#define INDEX_DISTANCE 0

#define INDEX_ANG_DIFF 1

#define FACE_DIRECTION M_PI

const double RANGE = 10 * M_PI / 180.0;

//RPLIDAR A2 min angle resolution is 0.45 degree
const double MATCH_THREATHOLD = 0.45 * M_PI / 180.0;
#pragma endregion

typedef union bytetofloat {
	unsigned char bval[4];
	float fval;
} b_f;

typedef union floattobyte {
	float fval;
	unsigned char bval[4];
} f_b;

void KeiganSend() {

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

	while (appRunning) {
		char panData[64],tiltData[64];

		int retPan = snprintf(panData, sizeof panData, "%f", projOri[0]);
		int retTilt = snprintf(tiltData, sizeof tiltData, "%f", projOri[1]);

		if (retPan < sizeof panData && retTilt < sizeof tiltData) {

			sendto(sock_pan, panData, retPan,0, (struct sockaddr*)&addr_pan, sizeof(addr_pan));
			sendto(sock_tilt, tiltData, retTilt, 0, (struct sockaddr*)&addr_tilt, sizeof(addr_tilt));

		}
	}

	closesocket(sock_pan);
	closesocket(sock_tilt);

	WSACleanup();

}

void HeadOrientation() {

	WSAData wsaData;

	SOCKET sock;
	struct sockaddr_in addr;

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

	err = bind(sock, (struct sockaddr*)&addr, sizeof(addr));
	printf("Bind Error : %d\n", err);

	memset(buf, 0, sizeof(buf));

	int saveLen = 0;
	unsigned char saveData[100] = "";
	char data = 0;

	while (appRunning) {
		int len = recv(sock, buf, sizeof(buf), 0);

		//printf("len : %d\n", len);
		//printf("Error : %d\n", WSAGetLastError());

		for (int i = 0;i < len;i++) {
			data = buf[i];

			//printf("%x-", data);
			if ((data & 0xFF) == HEADER && saveLen != 0) {
				char conv_data[4] = "";

				for (int j = 0;j < saveLen;j++) {

					if (saveData[j] == PITCHSIGN) {
						j++;
						for (int k = 0;k < 4;k++) {
							if (saveData[j] == REPLACE) {
								j++;
								conv_data[k] = saveData[j] ^ 0xFF;
							}
							else {
								conv_data[k] = saveData[j];
							}
							j++;
						}
						j--;
						b_f bf;
						for (int i = 0;i < 4;i++) {
							bf.bval[i] = conv_data[i];
						}
						headOri[0] = bf.fval;
						projOri[1] = headOri[0];
					}
					else if (saveData[j] == YAWSIGN) {
						j++;
						for (int k = 0;k < 4;k++) {
							if (saveData[j] == REPLACE) {
								j++;
								conv_data[k] = saveData[j] ^ 0xFF;
							}
							else {
								conv_data[k] = saveData[j];
							}
							j++;
						}
						j--;
						b_f bf;
						for (int i = 0;i < 4;i++) {
							bf.bval[i] = conv_data[i];
						}
						headOri[1] = bf.fval;
					}
					else if (saveData[j] == ROLLSIGN) {
						j++;
						for (int k = 0;k < 4;k++) {
							if (saveData[j] == REPLACE) {
								j++;
								conv_data[k] = saveData[j] ^ 0xFF;
							}
							else {
								conv_data[k] = saveData[j];
							}
							j++;
						}
						j--;
						b_f bf;
						for (int i = 0;i < 4;i++) {
							bf.bval[i] = conv_data[i];
						}
						headOri[2] = bf.fval;
						projOri[0] = headOri[2];
					}
				}

				//printf("pitch : %f\nyaw : %f\nroll : %f\n", headOri[0], headOri[1], headOri[2]);
				saveLen = 0;
			}

			if ((data & 0xFF) == HEADER || saveLen != 0) {
				saveData[saveLen++] = (unsigned char)(data & 0xFF);
			}
		}

		printf("\n");
	}

	closesocket(sock);

	WSACleanup();
}

int main() {
	appRunning = true;

	std::thread th_keigan(KeiganSend);
	std::thread th_headori(HeadOrientation);
	//std::thread th_whillori(WhillOrientation);

	while (true) {

		printf("HeadOri : %f\n%f\n%f\n\n", headOri[0], headOri[1], headOri[2]);
		//printf("WhillOri : %f\n%f\n%f\n\n", whillOri[0], whillOri[1], whillOri[2]);

		if (_kbhit()) {
			appRunning = false;
			break;
		}
	}

	th_keigan.join();
	th_headori.join();
	//th_whillori.join();

	return 0;
}
