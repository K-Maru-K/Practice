#include <iostream>  // for debug writing
#include <string>    // useful for reading and writing
#include <stdio.h>
#include <fstream>   // ifstream, ofstream
#include <sstream>   // istringstream

#define FILESIZE 23278
#define MEDIANSIZE 880

static float data[FILESIZE][2];
static float result[FILESIZE][2];

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

int single_partition(float array[], int l, int r) {
	float pivot = array[r];
	int i = (l - 1);

	for (int j = l; j <= r - 1; j++) {
		if (array[j] <= pivot) {
			i++;
			swap(&array[i], &array[j]);
		}
	}
	swap(&array[i + 1], &array[r]);
	return (i + 1);
}

void single_quickSort(float array[], int l, int r) {
	if (l < r) {
		int pivot = single_partition(array, l, r);
		single_quickSort(array, l, pivot - 1);
		single_quickSort(array, pivot + 1, r);
	}
}

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

float Median(float array[FILESIZE][2], int start) {

	int index = start;
	float val[MEDIANSIZE];
	float res = 0;

	for (int i = 0;i < MEDIANSIZE;i++) {
		val[i] = array[indexReturn(index + i, FILESIZE)][0];
	}

	single_quickSort(val, 0, MEDIANSIZE - 1);

	if (MEDIANSIZE % 2 == 0) {
		res = (val[(MEDIANSIZE - 1) / 2] + val[(MEDIANSIZE - 1) / 2 + 1]) / 2;
	}
	else {
		res = val[MEDIANSIZE / 2];
	}

	return res;

}

int MedianFilter() {

	std::ifstream ifs("C:\\Users\\sens\\Documents\\Research\\Program\\Open\\Practice\\cpp\\Test\\Test\\3DPointData.txt");
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

	for (int i = 0;i < FILESIZE;i++) {
		result[i][0] = Median(data, i);
		result[i][1] = data[i][1];
	}

	std::ofstream ofs("C:\\Users\\sens\\Documents\\Research\\Program\\Open\\Practice\\cpp\\Test\\Test\\3DModifiedData880.txt");
	if (!ofs) {
		std::cerr << "ファイルオープンに失敗" << std::endl;
		std::exit(1);
	}

	float past_data = 0;
	for (int i = 0;i < FILESIZE;i++) {
		if (abs(past_data - result[i][1]) > 0.000001) {
			char resString[30];
			//sprintf_s(resString, "%f\t%f\n", data[i][0], data[i][1]);
			sprintf_s(resString, "%f\t%f\n", result[i][0], result[i][1]);
			ofs << resString;
			past_data = result[i][1];
		}
	}

	return 0;
}