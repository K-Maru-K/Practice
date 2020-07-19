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
		printf("-");
	}

	while (v < min) {
		v += (max - min);
		printf("+");
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
		//std::vector<double> x(8192), y(8192);
		//std::vector<double > Dx(1), Dy(1);
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

					//x.at(graphCount) = dis * cos(arg - M_PI / 2);
					//y.at(graphCount++) = dis * sin(arg - M_PI / 2);

					if (abs(Arrange(arg - headDir, -M_PI, M_PI)) < nearData[MEDIANSIZE - 1][1]) {
						nearData[MEDIANSIZE - 1][0] = dis;
						nearData[MEDIANSIZE - 1][1] = abs(Arrange(arg - headDir, -M_PI, M_PI));
						InsertSort(nearData, MEDIANSIZE, INDEX_ANG_DIFF);
					}

				}
			}

			dist = Median(nearData, MEDIANSIZE);

//#pragma region Graph show
//			//printf("count : %d\n", graphCount);
//			plt::clf();
//			plt::xlim(-6.5, 6.5);
//			plt::ylim(-6.5, 6.5);
//			plt::scatter(x, y);
//			plt::draw();
//			//plt::pause(0.1);
//
//
//			Dx.at(0) = dist * cos(headDir - M_PI / 2);
//			Dy.at(0) = dist * sin(headDir - M_PI / 2);
//			plt::scatter(Dx, Dy, 5);
//			plt::draw();
//			plt::pause(0.1);
//#pragma endregion

		}

		//std::vector<double>().swap(x);
		//std::vector<double>().swap(y);
		//std::vector<double>().swap(Dx);
		//std::vector<double>().swap(Dy);

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

		printf("\n");
	}

	closesocket(sock);

	WSACleanup();
}

/// <summary>
/// port is 52520
/// </summary>
void WhillOrientation() {

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
	addr.sin_port = htons(52520);
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

		printf("\n");
	}

	closesocket(sock);

	WSACleanup();
}

//int main() {
//	//thread th_headori(HeadOrientation);
//
//	const char* opt_com_path = "COM4";
//	_u32         baudrateArray[2] = { 115200, 256000 };
//	_u32         opt_com_baudrate = 115200;
//	u_result     op_result;
//	int COUNTUP = 0;
//
//	bool useArgcBaudrate = false;
//
//	//string filename = "3DPointData_20200625_3.txt";
//
//	//ofstream writing_file;
//	//writing_file.open(filename, ios::app);
//
//	int showCount = 0;
//	istream::int_type ch;
//	// create the driver instance
//	RPlidarDriver* drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
//	if (!drv) {
//		fprintf(stderr, "insufficent memory, exit\n");
//		exit(-2);
//	}
//
//	rplidar_response_device_info_t devinfo;
//	bool connectSuccess = false;
//	// make connection...
//	if (useArgcBaudrate)
//	{
//		if (!drv)
//			drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
//		if (IS_OK(drv->connect(opt_com_path, opt_com_baudrate)))
//		{
//			op_result = drv->getDeviceInfo(devinfo);
//
//			if (IS_OK(op_result))
//			{
//				connectSuccess = true;
//			}
//			else
//			{
//				delete drv;
//				drv = NULL;
//			}
//		}
//	}
//	else
//	{
//		size_t baudRateArraySize = (sizeof(baudrateArray)) / (sizeof(baudrateArray[0]));
//		for (size_t i = 0; i < baudRateArraySize; ++i)
//		{
//			if (!drv)
//				drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
//			if (IS_OK(drv->connect(opt_com_path, baudrateArray[i])))
//			{
//				op_result = drv->getDeviceInfo(devinfo);
//
//				if (IS_OK(op_result))
//				{
//					connectSuccess = true;
//					break;
//				}
//				else
//				{
//					delete drv;
//					drv = NULL;
//				}
//			}
//		}
//	}
//
//	if (!connectSuccess) {
//
//		fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n"
//			, opt_com_path);
//		goto on_finished;
//	}
//
//	// check health...
//	if (!checkRPLIDARHealth(drv)) {
//		goto on_finished;
//	}
//
//	signal(SIGINT, ctrlc);
//
//	drv->startMotor();
//	// start scan...
//	drv->startScan(0, 1);
//
//	// fetech result and print it out...
//	while (1) {
//		rplidar_response_measurement_node_hq_t nodes[8192];
//		size_t   count = _countof(nodes);
//		vector<double> x(8192), y(8192);
//		vector<double > Dx(1), Dy(1);
//		double  nearData[MEDIANSIZE][2] = {  };
//		int graphCount = 0;
//
//		double headDir = headOri[1] + FACE_DIRECTION;
//
//		for (int i = 0;i < MEDIANSIZE;i++) {
//			nearData[i][0] = 6;
//			nearData[i][1] = RANGE;
//		}
//
//		graphCount = 0;
//		op_result = drv->grabScanDataHq(nodes, count);
//		if (IS_OK(op_result)) {
//			drv->ascendScanData(nodes, count);
//			for (int pos = 0; pos < (int)count; ++pos) {
//				double dis = nodes[pos].dist_mm_q2 / 4.0 / 1000.0;
//				double arg = nodes[pos].angle_z_q14 * M_PI / 2.0 / (1 << 14);
//				if (nodes[pos].quality != 0 && dis < 6 && dis > 0.015 && graphCount < 8192) {
//
//					x.at(graphCount) = dis * cos(arg - M_PI / 2);
//					y.at(graphCount++) = dis * sin(arg - M_PI / 2);
//
//					if (abs(Arrange(arg - headDir, -M_PI, M_PI)) < nearData[MEDIANSIZE - 1][1]) {
//						nearData[MEDIANSIZE - 1][0] = dis;
//						nearData[MEDIANSIZE - 1][1] = abs(Arrange(arg - headDir, -M_PI, M_PI));
//						InsertSort(nearData, MEDIANSIZE, INDEX_ANG_DIFF);
//
//						//for (int i = 0;i < MEDIANSIZE;i++) {
//						//	printf("nearData[%d] : dist -> %f ang -> %f\n", i, nearData[i][0], nearData[i][1]);
//						//}
//
//					}
//				}
//
//			}
//
//			dist = Median(nearData, MEDIANSIZE);
//
//			//printf("count : %d\n", graphCount);
//			plt::clf();
//			plt::xlim(-6.5, 6.5);
//			plt::ylim(-6.5, 6.5);
//			plt::scatter(x, y);
//			plt::draw();
//			//plt::pause(0.1);
//
//
//			Dx.at(0) = dist * cos(headDir - M_PI / 2);
//			Dy.at(0) = dist * sin(headDir - M_PI / 2);
//			plt::scatter(Dx, Dy, 5);
//			plt::draw();
//			plt::pause(0.1);
//
//		}
//
//		vector<double>().swap(x);
//		vector<double>().swap(y);
//		vector<double>().swap(Dx);
//		vector<double>().swap(Dy);
//
//		if (_kbhit()) {
//			if (cin.get() == '\n') {
//				drv->stop();
//				drv->stopMotor();
//				break;
//			}
//		}
//	}
//
//	// done!
//	drv->stop();
//	drv->stopMotor();
//on_finished:
//	RPlidarDriver::DisposeDriver(drv);
//	drv = NULL;
//
//	return 0;
//}

int main() {
	appRunning = true;

	std::thread th_dist(DetectDistance);
	std::thread th_headori(HeadOrientation);
	//std::thread th_whillori(WhillOrientation);

	while (true) {

		printf("Dist : %f\n", dist);
		printf("HeadOri : %f\n%f\n%f\n\n", headOri[0], headOri[1], headOri[2]);
		//printf("WhillOri : %f\n%f\n%f\n\n", whillOri[0], whillOri[1], whillOri[2]);

		if (_kbhit()) {
			appRunning = false;
			break;
		}
	}

	th_dist.join();
	th_headori.join();
	//th_whillori.join();

	return 0;
}