#include "WifiUDPSender.h"

typedef union f_b {
	float fval;
	char bval[4];
} fb;

typedef union b_f {
	char bval[4];
	float fval;
} bf;

/// <summary>
/// Return length, and put value in data[]
/// </summary>
/// <param name="data">strage of value</param>
/// <param name="startIndex">Index of input</param>
/// <param name="value">data value</param>
/// <returns></returns>
int dataConverter(char data[], int startIndex, float value) {
	int length = 0;
	fb converter;

	converter.fval = value;

	for (int i = 0; i < 4; i++)
	{
		if (converter.bval[i] == HEADER || converter.bval[i] == REPLACE) {
			data[startIndex + length++] = REPLACE;
			data[startIndex + length++] = converter.bval[i] ^ 0xFF;
		}
		else {
			data[startIndex + length++] = converter.bval[i];
		}
	}

	return length;
}

/// <summary>
/// Arrange the data for sending, and return length
/// Need more than 9 length in data array
/// </summary>
/// <param name="data">storage of arranged data</param>
/// <param name="value">a data of float value</param>
/// <returns></returns>
int Form(char data[], float value) {
	int length = 0;

	data[length++] = HEADER;

	length += dataConverter(data, length, value);

	return length;
}

int Form_data(char data[], float value) {
	int length = 0;

	data[length++] = HEADER;

	data[length++] = DATASIGN;
	length += dataConverter(data, length, value);

	return length;
}

int Form_Pos(char data[], float x, float y, float z) {
	int length = 0;

	data[length++] = HEADER;

	data[length++] = XSIGN;
	length += dataConverter(data, length, x);
	data[length++] = YSIGN;
	length += dataConverter(data, length, y);
	data[length++] = ZSIGN;
	length += dataConverter(data, length, z);

	return length;
}

int Form_Ori(char data[], float pitch, float yaw, float roll) {
	int length = 0;

	data[length++] = HEADER;

	data[length++] = PITCHSIGN;
	length += dataConverter(data, length, pitch);
	data[length++] = YAWSIGN;
	length += dataConverter(data, length, yaw);
	data[length++] = ROLLSIGN;
	length += dataConverter(data, length, roll);

	return length;
}

int Form_PosOri(char data[], float x, float y, float z, float pitch, float yaw, float roll) {
	int length = 0;

	data[length++] = HEADER;

	data[length++] = XSIGN;
	length += dataConverter(data, length, x);
	data[length++] = YSIGN;
	length += dataConverter(data, length, y);
	data[length++] = ZSIGN;
	length += dataConverter(data, length, z);

	data[length++] = PITCHSIGN;
	length += dataConverter(data, length, pitch);
	data[length++] = YAWSIGN;
	length += dataConverter(data, length, yaw);
	data[length++] = ROLLSIGN;
	length += dataConverter(data, length, roll);

	return length;
}

//int Rec(char data[], int len, float value[]) {
//
//}
//
//int Rec_data(char data[], int len, float value[]) {
//
//}
//
//int Rec_Pos(char data[], int len, float pos[]) {
//
//}
//
//int Rec_Ori(char data[], int len, float ori[]) {
//
//}
//
//int Rec_PosOri(char data[], int len, float pos[], float ori[]) {
//
//}
