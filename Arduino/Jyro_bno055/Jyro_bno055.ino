#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

//double roll_bias = 0;
double pitch_bias = 0;
double yaw_bias = 0;

int sign(double x) {
  int value = 0;
  if (x > 0)
    value = 1;
  else if (x < 0)
    value = -1;

  return value;
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
  Serial.begin(115200);
  Sensor_setup();

  /* Get a new sensor event */
  imu::Quaternion quat = bno.getQuat();
  double ysqr = quat.y() * quat.y();

  //  // roll (x-axis rotation)
  //  double t0 = +2.0 * (quat.w() * quat.x() + quat.y() * quat.z());
  //  double t1 = +1.0 - 2.0 * (quat.x() * quat.x() + ysqr);
  //  roll_bias = std::atan2(t0, t1) * 180 / PI;

  // pitch (y-axis rotation)
  double t2 = +2.0 * (quat.w() * quat.y() - quat.x() * quat.z());
  t2 = t2 > 1.0 ? 1.0 : t2;
  t2 = t2 < -1.0 ? -1.0 : t2;
  pitch_bias = std::asin(t2) * 180 / PI;

  // yaw (z-axis rotation)
  double t3 = +2.0 * (quat.w() * quat.z() + quat.x() * quat.y());
  double t4 = +1.0 - 2.0 * (ysqr + quat.z() * quat.z());
  yaw_bias = std::atan2(t3, t4) * 180 / PI;
}

void loop()
{
  /* Get a new sensor event */
  imu::Quaternion quat = bno.getQuat();
  double ysqr = quat.y() * quat.y();

  //  // roll (x-axis rotation)
  //  double t0 = +2.0 * (quat.w() * quat.x() + quat.y() * quat.z());
  //  double t1 = +1.0 - 2.0 * (quat.x() * quat.x() + ysqr);
  //  double roll = std::atan2(t0, t1) * 180 / PI;

  // pitch (y-axis rotation)
  double t2 = +2.0 * (quat.w() * quat.y() - quat.x() * quat.z());
  t2 = t2 > 1.0 ? 1.0 : t2;
  t2 = t2 < -1.0 ? -1.0 : t2;
  double pitch = std::asin(t2) * 180 / PI;

  // yaw (z-axis rotation)
  double t3 = +2.0 * (quat.w() * quat.z() + quat.x() * quat.y());
  double t4 = +1.0 - 2.0 * (ysqr + quat.z() * quat.z());
  double yaw = std::atan2(t3, t4) * 180 / PI;

  //  roll -= roll_bias;
  pitch -= pitch_bias;
  yaw -= yaw_bias;

  // //If you wanna adjust from -180 to 180, uncomment the following code
  //
  //  while (abs(roll) > 180) {
  //    roll -= sign(roll) * 360;
  //  }
  //
  //  while (abs(pitch) > 180) {
  //    pitch -= sign(pitch) * 360;
  //  }
  //
  //  while (abs(yaw) > 180) {
  //    yaw -= sign(yaw) * 360;
  //  }

  /* New line for the next sample */
  Serial.println("");

  //  Serial.printf("roll: %f\n", roll);
  Serial.printf("pitch: %f\n", pitch);
  Serial.printf("yaw: %f\n", yaw);

  /* Wait the specified delay before requesting next data */
  delay(BNO055_SAMPLERATE_DELAY_MS);

  M5.update();
}
