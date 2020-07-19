#include <WiFi.h>
#include <SPI.h>
#include <WiFiUDP.h>
#include <WiFiClient.h>
#include <WiFiAP.h>
#include <M5Stack.h>
#include "log_util.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

double pitch_bias = 0, yaw_bias = 0, roll_bias = 0;
int count = 0;

bool flag = false;
int sendCount = 0;

WiFiServer server(80);

static WiFiUDP wifiUdp;
IPAddress ip(192, 168, 4, 2); //Node static IP
IPAddress gateway(192, 168, 11, 1);
IPAddress subnet(255, 255, 255, 0);
static const char *kRemoteIpadr = "192.168.4.2";
static const int kRmoteUdpPort = 52525;
//static const int kRmoteUdpPort = 52520;
static const char ssid[] = "ExtendedHand-G";
static const char pass[] = "machikanehomare";

int sign(double x) {
  int value = 0;
  if (x > 0)
    value = 1;
  else if (x < 0)
    value = -1;

  return value;
}

void M5_Screen_clear() {
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.fillScreen(BLACK);
}

void M5_setup() {
  M5.begin();
  displayWelcome();
  delay(1000);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setTextColor(GREEN , BLACK);
  M5_Screen_clear();
}

static void WiFi_setup()
{
  static const int kLocalPort = 7777;

  // You can remove the password parameter if you want the AP to be open.
  WiFi.softAP(ssid, pass);

  WiFi.config(ip, ip, subnet);
  server.begin();

  Serial.println("WiFi SERVER started");

  M5.Lcd.println("");
  M5.Lcd.println(WiFi.localIP());
  wifiUdp.begin(kLocalPort);

  IPAddress myIP = WiFi.softAPIP();

  Serial.print("AP IP: ");
  Serial.println(myIP);
}

static void Serial_setup()
{
  Serial.begin(115200);
  M5_setup();
}

static void Sensor_setup()
{
  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  delay(1000);

  bno.setExtCrystalUse(true);
}

void setup() {
  Serial_setup();
  WiFi_setup();
  Sensor_setup();

  pitch_bias = yaw_bias = roll_bias = 0;

  for (int i = 0; i < 10; i++) {
    imu::Quaternion quat = bno.getQuat();
    double ysqr = quat.y() * quat.y();

    // pitch (x-axis rotation)
    double t0 = +2.0 * (quat.w() * quat.x() + quat.y() * quat.z());
    double t1 = +1.0 - 2.0 * (quat.x() * quat.x() + ysqr);
    pitch_bias += std::atan2(t0, t1) / 10;

    // roll (y-axis rotation)
    double t2 = +2.0 * (quat.w() * quat.y() - quat.x() * quat.z());
    t2 = t2 > 1.0 ? 1.0 : t2;
    t2 = t2 < -1.0 ? -1.0 : t2;
    roll_bias += std::asin(t2) / 10;

    // yaw (z-axis rotation)
    double t3 = +2.0 * (quat.w() * quat.z() + quat.x() * quat.y());
    double t4 = +1.0 - 2.0 * (ysqr + quat.z() * quat.z());
    yaw_bias += std::atan2(t3, t4) / 10;

    delay(100);
  }
  
  char sendData[100];
  int len = sprintf(sendData, "%f\t%f\t%f\n", roll_bias, pitch_bias, yaw_bias);

  wifiUdp.beginPacket(kRemoteIpadr, kRmoteUdpPort);
  wifiUdp.write((uint8_t *)"-------\n", 10);
  wifiUdp.write((uint8_t *)sendData, len);
  wifiUdp.write((uint8_t *)"-------\n\n", 10);
  wifiUdp.endPacket();

  flag = false;

}

void loop()
{
  /* Get a new sensor event */
  imu::Quaternion quat = bno.getQuat();
  double ysqr = quat.y() * quat.y();

  // pitch (x-axis rotation)
  double t0 = +2.0 * (quat.w() * quat.x() + quat.y() * quat.z());
  double t1 = +1.0 - 2.0 * (quat.x() * quat.x() + ysqr);
  double pitch = std::atan2(t0, t1);

  // roll (y-axis rotation)
  double t2 = +2.0 * (quat.w() * quat.y() - quat.x() * quat.z());
  t2 = t2 > 1.0 ? 1.0 : t2;
  t2 = t2 < -1.0 ? -1.0 : t2;
  double roll = std::asin(t2);

  // yaw (z-axis rotation)
  double t3 = +2.0 * (quat.w() * quat.z() + quat.x() * quat.y());
  double t4 = +1.0 - 2.0 * (ysqr + quat.z() * quat.z());
  double yaw = std::atan2(t3, t4);

  pitch -= pitch_bias;
  yaw -= yaw_bias;
  roll -= roll_bias;

  if (!flag) {
    M5.Lcd.fillScreen(TFT_RED);
    String Data = "";
    char c[10] = "000000000";

    if (wifiUdp.parsePacket() > 0) {
      wifiUdp.read(c, 10);
      wifiUdp.flush();
    }

    Data = String(c);
    if (Data.indexOf("St\n") != -1) {
      flag = true;
      wifiUdp.beginPacket(kRemoteIpadr, kRmoteUdpPort);
      wifiUdp.write((uint8_t *)"-------\n\n", 10);
      wifiUdp.endPacket();
      sendCount = 0;
    }
    else if (Data.indexOf("Re\n") != -1) {
      pitch_bias += pitch;
      yaw_bias += yaw;
      roll_bias += roll;
      
      char sendData[100];
      int len = sprintf(sendData, "%f\t%f\t%f\n", roll_bias, pitch_bias, yaw_bias);

      wifiUdp.beginPacket(kRemoteIpadr, kRmoteUdpPort);
      wifiUdp.write((uint8_t *)"-------\n", 10);
      wifiUdp.write((uint8_t *)sendData, len);
      wifiUdp.write((uint8_t *)"-------\n\n", 10);
      wifiUdp.endPacket();
    }

  } else {
    M5.Lcd.fillScreen(TFT_BLUE);
    char sendData[100];
    int len = sprintf(sendData, "%f\t%f\t%f\n", roll, pitch, yaw);

    wifiUdp.beginPacket(kRemoteIpadr, kRmoteUdpPort);
    wifiUdp.write((uint8_t *)sendData, len);
    wifiUdp.endPacket();

//    Serial.printf("roll: %f\npitch: %f\nyaw: %f\n", roll, pitch, yaw);
//    Serial.printf("sendCount: %d\n\n", sendCount);

    sendCount++;

    if (sendCount >= 20) {
      flag = false;
    }

  }

  /* Wait the specified delay before requesting next data */
  //  delay(BNO055_SAMPLERATE_DELAY_MS);
  delay(200);

  M5.update();
}
