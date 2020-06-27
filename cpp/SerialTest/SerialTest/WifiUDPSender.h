#ifndef _WifiUDPSender_h
#define _WifiUDPSender_h

#define HEADER 0xAF
#define REPLACE 0xAE

#define XSIGN 'x'
#define YSIGN 'y'
#define ZSIGN 'z'

#define PITCHSIGN 'p'
#define YAWSIGN 't'
#define ROLLSIGN 'r'

#define DATASIGN 'd'

#define MAXSIZE 60

int Form(char data[], float value);
int Form_data(char data[], float value);
int Form_Pos(char data[], float x, float y, float z);
int Form_Ori(char data[], float pitch, float yaw, float roll);
int Form_PosOri(char data[], float x, float y, float z, float pitch, float yaw, float roll);

//int Rec(char data[], int len, float value[]);
//int Rec_data(char data[], int len, float value[]);
//int Rec_Pos(char data[], int len, float pos[]);
//int Rec_Ori(char data[], int len, float ori[]);
//int Rec_PosOri(char data[], int len, float pos[], float ori[]);

int dataConverter(char data[], int startIndex, float value);

#endif
