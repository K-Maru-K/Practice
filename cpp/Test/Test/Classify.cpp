#include <iostream>  // for debug writing
#include <string>    // useful for reading and writing
#include <stdio.h>
#include <fstream>   // ifstream, ofstream
#include <sstream>   // istringstream

#define FILESIZE 23278

#define THREASHOLD 1000000
#define COUNTTHREASHOLD 500

static float data[FILESIZE][2];
static float result[FILESIZE][2];
static int classify[FILESIZE] = {};
static int classNum[FILESIZE] = {};
int sumclass = 0;

int indexReturn(int index, int size) {
	int val = index;

	while (val >= size) {
		val -= size;
	}

	while (val < 0) {
		val += size;
	}

	return val;
}

// 最小二乗法の計算 
int lsm(float d[FILESIZE][2], int N, float* a0, float* a1, int classstart)
{
	float A00 = 0, A01 = 0, A02 = 0, A11 = 0, A12 = 0;

	for (int i = 0;i < N;i++) {
		float x = 0, y = 0;
		x = d[classstart + i][0] / 1000 * cosf(d[classstart + i][1]);
		y = d[classstart + i][0] / 1000 * sinf(d[classstart + i][1]);

		A00 += 1.0;
		A01 += x;
		A02 += y;
		A11 += x * x;
		A12 += x * y;
	}

	*a0 = (A02 * A11 - A01 * A12) / (A00 * A11 - A01 * A01);
	*a1 = (A00 * A12 - A01 * A02) / (A00 * A11 - A01 * A01);

	return N + classstart;
}

int lsm_init(float d[FILESIZE][2], int N, float* a0, float* a1)
{
	float A00 = 0, A01 = 0, A02 = 0, A11 = 0, A12 = 0;
	int count = 0;

	for (int i = 0;i < N;i++) {
		if (classify[i] == classify[0]) {
			float x = 0, y = 0;
			x = d[i][0] / 1000 * cosf(d[i][1]);
			y = d[i][0] / 1000 * sinf(d[i][1]);

			A00 += 1.0;
			A01 += x;
			A02 += y;
			A11 += x * x;
			A12 += x * y;
			count++;
		}
		else {
			break;
		}
	}

	for (int i = 0;i < N - count;i++) {
		if (classify[indexReturn(-1 - i, FILESIZE)] == classify[0]) {
			float x = 0, y = 0;
			x = d[indexReturn(-1 - i, FILESIZE)][0] / 1000 * cosf(d[indexReturn(-1 - i, FILESIZE)][1]);
			y = d[indexReturn(-1 - i, FILESIZE)][0] / 1000 * sinf(d[indexReturn(-1 - i, FILESIZE)][1]);

			A00 += 1.0;
			A01 += x;
			A02 += y;
			A11 += x * x;
			A12 += x * y;
		}
		else {
			break;
		}
	}

	*a0 = (A02 * A11 - A01 * A12) / (A00 * A11 - A01 * A01);
	*a1 = (A00 * A12 - A01 * A02) / (A00 * A11 - A01 * A01);

	return count;
}

void swap(float* a, float* b)
{
	float t = *a;
	*a = *b;
	*b = t;
}

int partition(float array[FILESIZE][2], int l, int r) {
	float pivot = array[r][1];
	int i = (l - 1);

	for (int j = l; j <= r - 1; j++) {
		if (array[j][1] <= pivot) {
			i++;
			swap(&array[i][0], &array[j][0]);
			swap(&array[i][1], &array[j][1]);
		}
	}
	swap(&array[i + 1][0], &array[r][0]);
	swap(&array[i + 1][1], &array[r][1]);
	return (i + 1);
}

void quickSort(float array[FILESIZE][2], int l, int r) {
	if (l < r) {
		int pivot = partition(array, l, r);
		quickSort(array, l, pivot - 1);
		quickSort(array, pivot + 1, r);
	}
}

//int single_partition(float array[], int l, int r) {
//	float pivot = array[r];
//	int i = (l - 1);
//
//	for (int j = l; j <= r - 1; j++) {
//		if (array[j] <= pivot) {
//			i++;
//			swap(&array[i], &array[j]);
//		}
//	}
//	swap(&array[i + 1], &array[r]);
//	return (i + 1);
//}
//
//void single_quickSort(float array[], int l, int r) {
//	if (l < r) {
//		int pivot = single_partition(array, l, r);
//		single_quickSort(array, l, pivot - 1);
//		single_quickSort(array, pivot + 1, r);
//	}
//}

int main() {

	std::ifstream ifs("C:\\Users\\sens\\Documents\\Research\\Program\\Open\\Practice\\cpp\\Test\\Test\\3DPointData_20200625_1.txt");
	if (!ifs) {
		std::cerr << "ファイルオープンに失敗" << std::endl;
		std::exit(1);
	}

	for (int i = 0;i < FILESIZE;i++) {

		std::string buf;
		ifs >> buf;

		if (!ifs) {
			std::cerr << "読み込みに失敗" << std::endl;
			std::exit(1);
		}

		data[i][0] = std::stof(buf);

		ifs >> buf;

		if (!ifs) {
			std::cerr << "読み込みに失敗" << std::endl;
			std::exit(1);
		}

		data[i][1] = std::stof(buf);
	}

	quickSort(data, 0, FILESIZE - 1);

	classify[0] = 1;
	classNum[1]++;
	for (int i = 1;i < FILESIZE;i++) {
		if (data[i - 1][0] * data[i - 1][0] + data[i][0] * data[i][0] - 2 * data[i - 1][0] * data[i][0] * cos(abs(data[i - 1][1] - data[i][1])) < THREASHOLD) {
			classify[i] = classify[i - 1];
		}
		else {
			classify[i] = classify[i - 1] + 1;
		}
		classNum[classify[i]]++;
		sumclass = classify[i];
	}

	bool mergeFlag = false;
	if (classify[0] != classify[FILESIZE - 1] && data[0][0] * data[0][0] + data[FILESIZE - 1][0] * data[FILESIZE - 1][0] - 2 * data[0][0] * data[FILESIZE - 1][0] * cos(abs(data[0][1] - data[FILESIZE - 1][1])) < THREASHOLD) {
		for (int i = 1;i < FILESIZE;i++) {
			if (classify[i] == classify[FILESIZE - 1]) {
				classify[i] = classify[0];
				mergeFlag = true;
			}
		}
	}

	if (mergeFlag) {
		classNum[classify[0]] += classNum[classify[FILESIZE - 1]];
		classNum[classify[FILESIZE - 1]] = 0;
		sumclass--;
	}

	std::ofstream of_s("C:\\Users\\sens\\Documents\\Research\\Program\\Open\\Practice\\cpp\\Test\\Test\\3DClassData_20200625_2_coe.txt");
	if (!of_s) {
		std::cerr << "ファイルオープンに失敗" << std::endl;
		std::exit(1);
	}

	float a = 0, b = 0;
	int nowIndex = lsm_init(data, classNum[1], &a, &b);
	//if (classNum[1] > COUNTTHREASHOLD) {
	char eqStr[30];
	sprintf_s(eqStr, "%f\t%f\n", a, b);
	of_s << eqStr;
	//}

	for (int i = 1;i < sumclass;i++) {
		nowIndex = lsm(data, classNum[i + 1], &a, &b, nowIndex);
		//if (classNum[i + 1] > COUNTTHREASHOLD)
		//{
		char eqStr_2[30];
		sprintf_s(eqStr_2, "%f\t%f\n", a, b);
		of_s << eqStr_2;
		//}
	}

	std::ofstream ofs("C:\\Users\\sens\\Documents\\Research\\Program\\Open\\Practice\\cpp\\Test\\Test\\3DClassData_20200625_2.txt");
	if (!ofs) {
		std::cerr << "ファイルオープンに失敗" << std::endl;
		std::exit(1);
	}

	float past_arg = 0;
	for (int i = 0;i < FILESIZE;i++) {
		if (abs(past_arg - data[i][1]) > 0.000001) {
			char resString[30];
			sprintf_s(resString, "%d\t%f\t%f\n", classify[i], data[i][0], data[i][1]);
			ofs << resString;
			past_arg = data[i][1];
		}
	}

	return 0;
}