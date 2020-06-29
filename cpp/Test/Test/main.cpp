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
using namespace std;
using namespace rp::standalone::rplidar;

#define MEDIANSIZE 5

#define INDEX_DISTANCE 0

#define INDEX_ANG_DIFF 1

#define FACE_DIRECTION M_PI

const double RANGE = 10 * M_PI / 180.0;

//RPLIDAR A2 min angle resolution is 0.45 degree
const double MATCH_THREATHOLD = 0.45 * M_PI / 180.0;

float dist = 0;
float whillOri[3] = { 0,0,0 };

float headOri[3] = { 0,0,0 };

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

int main() {
	//thread th_headori(HeadOrientation);

	const char* opt_com_path = "COM4";
	_u32         baudrateArray[2] = { 115200, 256000 };
	_u32         opt_com_baudrate = 115200;
	u_result     op_result;
	int COUNTUP = 0;

	bool useArgcBaudrate = false;

	//string filename = "3DPointData_20200625_3.txt";

	//ofstream writing_file;
	//writing_file.open(filename, ios::app);

	int showCount = 0;
	istream::int_type ch;
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
	while (1) {
		rplidar_response_measurement_node_hq_t nodes[8192];
		size_t   count = _countof(nodes);
		vector<double> x(8192), y(8192);
		vector<double > Dx(1), Dy(1);
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
				double dis = nodes[pos].dist_mm_q2 / 4.0 / 1000.0;
				double arg = nodes[pos].angle_z_q14 * M_PI / 2.0 / (1 << 14);
				if (nodes[pos].quality != 0 && dis < 6 && dis > 0.015 && graphCount < 8192) {

					x.at(graphCount) = dis * cos(arg - M_PI / 2);
					y.at(graphCount++) = dis * sin(arg - M_PI / 2);

					if (abs(Arrange(arg - headDir, -M_PI, M_PI)) < nearData[MEDIANSIZE - 1][1]) {
						nearData[MEDIANSIZE - 1][0] = dis;
						nearData[MEDIANSIZE - 1][1] = abs(Arrange(arg - headDir, -M_PI, M_PI));
						InsertSort(nearData, MEDIANSIZE, INDEX_ANG_DIFF);

						//for (int i = 0;i < MEDIANSIZE;i++) {
						//	printf("nearData[%d] : dist -> %f ang -> %f\n", i, nearData[i][0], nearData[i][1]);
						//}

					}
				}

			}

			dist = Median(nearData, MEDIANSIZE);

			//printf("count : %d\n", graphCount);
			plt::clf();
			plt::xlim(-6.5, 6.5);
			plt::ylim(-6.5, 6.5);
			plt::scatter(x, y);
			plt::draw();
			//plt::pause(0.1);


			Dx.at(0) = dist * cos(headDir - M_PI / 2);
			Dy.at(0) = dist * sin(headDir - M_PI / 2);
			plt::scatter(Dx, Dy, 5);
			plt::draw();
			plt::pause(0.1);

		}

		vector<double>().swap(x);
		vector<double>().swap(y);
		vector<double>().swap(Dx);
		vector<double>().swap(Dy);

		if (_kbhit()) {
			if (cin.get() == '\n') {
				drv->stop();
				drv->stopMotor();
				break;
			}
		}
	}

	// done!
	drv->stop();
	drv->stopMotor();
on_finished:
	RPlidarDriver::DisposeDriver(drv);
	drv = NULL;

	return 0;
}

//int main() {
//	cout << "matplotlib-cpp sample start" << endl;
//
//	int n = 5000;
//	vector<double> x(n), y(n), z(n);
//
//	for (int i = 0; i < 1000; ++i) {
//		x.at(i) = i;
//		y.at(i) = sin(2 * M_PI * i / 360.0);
//	}
//	plt::scatter(x, y);
//	plt::draw();
//	plt::pause(0.5);
//	plt::clf();
//
//	for (int i = 0; i < 1000; ++i) {
//		x.at(i) = i;
//		y.at(i) = sin(2 * M_PI * (i + 1000) / 360.0);
//	}
//	plt::scatter(x, y);
//	plt::draw();
//	plt::pause(0.5);
//	plt::clf();
//
//	for (int i = 0; i < 1000; ++i) {
//		x.at(i) = i;
//		y.at(i) = sin(2 * M_PI * (i + 2000) / 360.0);
//	}
//	plt::scatter(x, y);
//	plt::draw();
//	plt::pause(0.5);
//	plt::clf();
//
//
//	for (int i = 0; i < 1000; ++i) {
//		x.at(i) = i;
//		y.at(i) = sin(2 * M_PI * (i + 3000) / 360.0);
//	}
//	plt::scatter(x, y);
//	plt::draw();
//	plt::pause(0.5);
//
//
//	for (int i = 0; i < 1000; ++i) {
//		x.at(i) = i;
//		y.at(i) = sin(2 * M_PI * (i + 4000) / 360.0);
//	}
//	plt::scatter(x, y);
//
//	plt::draw();
//	plt::pause(1);
//
//	return 0;
//}