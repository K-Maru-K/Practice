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

#include "SerialWrapper.h"

#include <iostream>
#include <chrono>
#include <ctime>

namespace plt = matplotlibcpp;
//using namespace std;
using namespace rp::standalone::rplidar;

float dist = 0;
float whillOri[3] = { 0,0,0 };
float headOri[3] = { 0,0,0 };
float whillBias[3] = { 0,0,0 };
float headBias[3] = { 0,0,0 };

bool appRunning = true;

float preProjOri[2] = { 0,0 };

auto preTime = std::chrono::system_clock::now();

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

bool checkRPLIDARHealth(RPlidarDriver* drv)
{
	u_result     op_result;
	rplidar_response_device_health_t healthinfo;

	op_result = drv->getHealth(healthinfo);
	if (IS_OK(op_result)) { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
		printf("RPLidar health status : %d\n", healthinfo.status);
		if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
			fprintf(stderr, "Error, rplidar internal error detected. Please reboot the device to retry.\n");
			// enable the following code if you want rplidar to be reboot by software
			// drv->reset();
			return false;
		}
		else {
			return true;
		}

	}
	else {
		fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
		return false;
	}
}

#include <signal.h>
bool ctrl_c_pressed;
void ctrlc(int)
{
	ctrl_c_pressed = true;
}

double Arrange(double val, double min, double max) {
	double v = val;

	while (v > max) {
		v -= (max - min);
		//printf("-");
	}

	while (v < min) {
		v += (max - min);
		//printf("+");
	}

	return v;
}

void InsertSort(double array[MEDIANSIZE][2], int len, int num) {

	int j = 0;
	double k[2] = { 0,0 };

	for (int i = 1; i < len; i++) {
		k[0] = array[i][0];
		k[1] = array[i][1];
		j = i - 1;

		while ((j >= 0) && (array[j][num] > k[num])) {
			array[j + 1][0] = array[j][0];
			array[j + 1][1] = array[j][1];
			j--;
		}
		array[j + 1][0] = k[0];
		array[j + 1][1] = k[1];
	}

}

double Median(double array[MEDIANSIZE][2], int len) {
	double val = 0;

	InsertSort(array, len, INDEX_DISTANCE);

	if (len % 2 == 0) {
		val = (array[(len - 1) / 2][0] + array[(len - 1) / 2 + 1][0]) / 2;
	}
	else {
		val = array[len / 2][0];
	}

	return val;

}

void DetectDistance() {
	const char* opt_com_path = "COM4";
	_u32         baudrateArray[2] = { 115200, 256000 };
	_u32         opt_com_baudrate = 115200;
	u_result     op_result;
	int COUNTUP = 0;

	bool useArgcBaudrate = false;

	int showCount = 0;
	std::istream::int_type ch;

	// create the driver instance
	RPlidarDriver* drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
	if (!drv) {
		fprintf(stderr, "insufficent memory, exit\n");
		exit(-2);
	}

	rplidar_response_device_info_t devinfo;
	bool connectSuccess = false;
	// make connection...
	if (useArgcBaudrate)
	{
		if (!drv)
			drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
		if (IS_OK(drv->connect(opt_com_path, opt_com_baudrate)))
		{
			op_result = drv->getDeviceInfo(devinfo);

			if (IS_OK(op_result))
			{
				connectSuccess = true;
			}
			else
			{
				delete drv;
				drv = NULL;
			}
		}
	}
	else
	{
		size_t baudRateArraySize = (sizeof(baudrateArray)) / (sizeof(baudrateArray[0]));
		for (size_t i = 0; i < baudRateArraySize; ++i)
		{
			if (!drv)
				drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
			if (IS_OK(drv->connect(opt_com_path, baudrateArray[i])))
			{
				op_result = drv->getDeviceInfo(devinfo);

				if (IS_OK(op_result))
				{
					connectSuccess = true;
					break;
				}
				else
				{
					delete drv;
					drv = NULL;
				}
			}
		}
	}

	if (!connectSuccess) {

		fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n"
			, opt_com_path);
		goto on_finished;
	}

	// check health...
	if (!checkRPLIDARHealth(drv)) {
		goto on_finished;
	}

	signal(SIGINT, ctrlc);

	drv->startMotor();
	// start scan...
	drv->startScan(0, 1);

	// fetech result and print it out...
	while (true) {
		rplidar_response_measurement_node_hq_t nodes[8192];
		size_t   count = _countof(nodes);
		std::vector<double> x(8192), y(8192);
		std::vector<double > Dx(1), Dy(1);
		double  nearData[MEDIANSIZE][2] = {  };
		int graphCount = 0;

		double headDir = headOri[1] + FACE_DIRECTION;

		for (int i = 0;i < MEDIANSIZE;i++) {
			nearData[i][0] = 6;
			nearData[i][1] = RANGE;
		}

		graphCount = 0;
		op_result = drv->grabScanDataHq(nodes, count);
		if (IS_OK(op_result)) {
			drv->ascendScanData(nodes, count);
			for (int pos = 0; pos < (int)count; ++pos) {
				//unit is m
				double dis = nodes[pos].dist_mm_q2 / 4.0 / 1000;
				//unit is radian
				double arg = nodes[pos].angle_z_q14 * M_PI / 2.0 / (1 << 14);
				if (nodes[pos].quality != 0 && dis < 6 && dis > 0.015) {

					x.at(graphCount) = dis * cos(arg - M_PI / 2);
					y.at(graphCount++) = dis * sin(arg - M_PI / 2);

					if (abs(Arrange(arg - headDir, -M_PI, M_PI)) < nearData[MEDIANSIZE - 1][1]) {
						nearData[MEDIANSIZE - 1][0] = dis;
						nearData[MEDIANSIZE - 1][1] = abs(Arrange(arg - headDir, -M_PI, M_PI));
						InsertSort(nearData, MEDIANSIZE, INDEX_ANG_DIFF);
					}

				}
			}

			dist = Median(nearData, MEDIANSIZE);

			////printf("count : %d\n", graphCount);
			//plt::clf();
			//plt::xlim(-6.5, 6.5);
			//plt::ylim(-6.5, 6.5);
			//plt::scatter(x, y);
			//plt::draw();
			////plt::pause(0.1);


			//Dx.at(0) = dist * cos(headDir - M_PI / 2);
			//Dy.at(0) = dist * sin(headDir - M_PI / 2);
			//plt::scatter(Dx, Dy, 5);
			//plt::draw();
			//plt::pause(0.1);

		}

		std::vector<double>().swap(x);
		std::vector<double>().swap(y);
		std::vector<double>().swap(Dx);
		std::vector<double>().swap(Dy);

		if (!appRunning) {
			drv->stop();
			drv->stopMotor();
			break;
		}

	}

	// done!
	drv->stop();
	drv->stopMotor();
on_finished:
	RPlidarDriver::DisposeDriver(drv);
	drv = NULL;

}

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
		char panData[64], tiltData[64];

		float projOri[2];

		projOri[0] = headOri[1] - whillOri[1];
		projOri[1] = headOri[2];

		int retPan = snprintf(panData, sizeof panData, "%f", projOri[0]);
		int retTilt = snprintf(tiltData, sizeof tiltData, "%f", projOri[1]);

		if (retPan < sizeof panData && retTilt < sizeof tiltData) {
			if (abs(preProjOri[0] - projOri[0]) > 0.001 || abs(preProjOri[1] - projOri[1]) > 0.001) {
				auto t = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - preTime);
				std::cout << t.count() << std::endl;
				preTime = std::chrono::system_clock::now();
			}

			sendto(sock_pan, panData, retPan, 0, (struct sockaddr*)&addr_pan, sizeof(addr_pan));
			sendto(sock_tilt, tiltData, retTilt, 0, (struct sockaddr*)&addr_tilt, sizeof(addr_tilt));

		}

		preProjOri[0] = projOri[0];
		preProjOri[1] = projOri[1];

	}

	closesocket(sock_pan);
	closesocket(sock_tilt);

	WSACleanup();

}

/// <summary>
/// port is 52525
/// </summary>
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
					}
				}

				//printf("pitch : %f\nyaw : %f\nroll : %f\n", headOri[0], headOri[1], headOri[2]);
				saveLen = 0;
			}

			if ((data & 0xFF) == HEADER || saveLen != 0) {
				saveData[saveLen++] = (unsigned char)(data & 0xFF);
			}
		}

		//printf("\n");
	}

	closesocket(sock);

	WSACleanup();
}

/// <summary>
/// port is 52520
/// </summary>
void WhillOrientation() {
	SerialWrapper _serial(8);
	_serial.begin(115200);

	int saveLen = 0;
	unsigned char saveData[100] = "";
	char data = 0;

	while (appRunning) {
		int len = _serial.available();

		//printf("len : %d\n", len);
		//printf("Error : %d\n", WSAGetLastError());

		for (int i = 0;i < len;i++) {
			data = _serial.read();

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
						whillOri[0] = bf.fval;
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
						whillOri[1] = bf.fval;
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
						whillOri[2] = bf.fval;
					}
				}

				//printf("pitch : %f\nyaw : %f\nroll : %f\n", headOri[0], headOri[1], headOri[2]);
				saveLen = 0;
			}

			if ((data & 0xFF) == HEADER || saveLen != 0) {
				saveData[saveLen++] = (unsigned char)(data & 0xFF);
			}
		}

		//printf("\n");
	}

}

int main() {
	appRunning = true;

	std::thread th_dist(DetectDistance);
	std::thread th_headori(HeadOrientation);
	std::thread th_whillori(WhillOrientation);
	//std::thread th_Keigan(KeiganSend);

	while (true) {
		if (_kbhit() && _getch() == 's') {
			break;
		}
	}

	for (int i = 0;i < 3;i++) {
		headBias[i] = headOri[i];
		whillBias[i] = whillOri[i];
	}

#pragma region Keigan

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

#pragma endregion


	while (true) {

		//printf("Dist : %f\n", dist);
		//printf("HeadOri : %f\n%f\n%f\n\n", headOri[0], headOri[1], headOri[2]);
		//printf("WhillOri : %f\n%f\n%f\n\n", whillOri[0], whillOri[1], whillOri[2]);

#pragma region Keigan_loop

		char panData[64], tiltData[64];

		float projOri[2];

		projOri[0] = headOri[1] - whillOri[1];
		projOri[1] = headOri[2];

		int retPan = snprintf(panData, sizeof panData, "%f", projOri[0]);
		int retTilt = snprintf(tiltData, sizeof tiltData, "%f", projOri[1]);

		if (retPan < sizeof panData && retTilt < sizeof tiltData) {
			if (abs(preProjOri[0] - projOri[0]) > 0.001 || abs(preProjOri[1] - projOri[1]) > 0.001) {
				auto t = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - preTime);
				std::cout << t.count() << std::endl;
				preTime = std::chrono::system_clock::now();
			}

			sendto(sock_pan, panData, retPan, 0, (struct sockaddr*)&addr_pan, sizeof(addr_pan));
			sendto(sock_tilt, tiltData, retTilt, 0, (struct sockaddr*)&addr_tilt, sizeof(addr_tilt));

		}

		preProjOri[0] = projOri[0];
		preProjOri[1] = projOri[1];

#pragma endregion

		if (_kbhit()) {
			appRunning = false;
			break;
		}
	}

#pragma region Keigan_end
	closesocket(sock_pan);
	closesocket(sock_tilt);

	WSACleanup();
#pragma endregion


	th_dist.join();
	th_headori.join();
	th_whillori.join();
	//th_Keigan.join();

	return 0;
}