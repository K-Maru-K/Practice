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

#define MEDIANSIZE 50

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
		int graphCount = 0;

		graphCount = 0;
		op_result = drv->grabScanDataHq(nodes, count);
		if (IS_OK(op_result)) {
			drv->ascendScanData(nodes, count);
			for (int pos = 0; pos < (int)count; ++pos) {
				double dis = nodes[pos].dist_mm_q2 / 4.0;
				double arg = nodes[pos].angle_z_q14 * M_PI / 2.0 / (1 << 14);
				if (nodes[pos].quality != 0 && dis < 6000 && dis > 15 && graphCount < 8192) {

					x.at(graphCount) = dis * cos(arg) * 0.001;
					y.at(graphCount++) = dis * sin(arg) * 0.001;

					//char str[100];
					//sprintf(str, "%f\t%f\n",
					//	dis, arg);

					//writing_file << str;
				}
			}

			//printf("count : %d\n", graphCount);
			plt::clf();
			plt::xlim(-6, 6);
			plt::ylim(-6, 6);
			plt::scatter(x, y);
			plt::draw();
			plt::pause(0.1);
		}

		//if (showCount++ > 10) {
			//for (int i = 0;i < graphCount;i++) {
			//	printf("X: %f, Y: %f\n", x.at(i), y.at(i));
			//}
		//}

		if (_kbhit()) {
			if (cin.get() == '\n') {
				drv->stop();
				drv->stopMotor();
				break;
			}
		}

		//if ((COUNTUP++) > 100) {
		//	drv->stop();
		//	drv->stopMotor();
		//	break;
		//}

		//std::printf("Count up : %d\n", COUNTUP);
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