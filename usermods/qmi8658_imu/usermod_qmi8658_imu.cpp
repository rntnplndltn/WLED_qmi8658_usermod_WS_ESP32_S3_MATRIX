#include "usermod_qmi8658_imu.h"

void QMI8658Usermod::setup() {
  Wire.begin(SDA_PIN, SCL_PIN);
  if (!imu.begin(Wire, QMI8658_L_SLAVE_ADDRESS, SDA_PIN, SCL_PIN)) {
    DEBUG_PRINTLN(F("QMI8658C: Init fehlgeschlagen!"));
    return;
  }
  imu.configAccelerometer(
    SensorQMI8658::ACC_RANGE_4G,
    SensorQMI8658::ACC_ODR_1000Hz,
    SensorQMI8658::LPF_MODE_0
  );
  imu.configGyroscope(
    SensorQMI8658::GYR_RANGE_64DPS,
    SensorQMI8658::GYR_ODR_896_8Hz,
    SensorQMI8658::LPF_MODE_3
  );
  imu.enableAccelerometer();
  imu.enableGyroscope();
  initialized = true;
  DEBUG_PRINTLN(F("QMI8658C: Bereit!"));
}

void QMI8658Usermod::loop() {
  if (!initialized) return;
  if (!imu.getDataReady()) return;
  imu.getAccelerometer(ax, ay, az);
  imu.getGyroscope(gx, gy, gz);
  temp = imu.getTemperature_C();
}

void QMI8658Usermod::addToJsonInfo(JsonObject& root) {
  if (!initialized) return;
  JsonObject user = root["u"];
  if (user.isNull()) user = root.createNestedObject("u");

  JsonArray arrA = user.createNestedArray("IMU Accel (g)");
  arrA.add(serialized(String(ax, 3)));
  arrA.add(serialized(String(ay, 3)));
  arrA.add(serialized(String(az, 3)));

  JsonArray arrG = user.createNestedArray("IMU Gyro (dps)");
  arrG.add(serialized(String(gx, 2)));
  arrG.add(serialized(String(gy, 2)));
  arrG.add(serialized(String(gz, 2)));

  JsonArray arrT = user.createNestedArray("IMU Temp (C)");
  arrT.add(serialized(String(temp, 1)));
}

static QMI8658Usermod qmi8658_instance;
REGISTER_USERMOD(qmi8658_instance);