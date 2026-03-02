#pragma once
#include "wled.h"
#include <Wire.h>
#include "SensorQMI8658.hpp"

class QMI8658Usermod : public Usermod {
private:
  SensorQMI8658 imu;
  bool initialized = false;

  static constexpr int8_t SDA_PIN = 11;
  static constexpr int8_t SCL_PIN = 12;

  float ax = 0, ay = 0, az = 0;
  float gx = 0, gy = 0, gz = 0;
  float temp = 0;

public:
  void setup() override;
  void loop() override;
  void addToJsonInfo(JsonObject& root) override;
  uint16_t getId() override { return USERMOD_ID_IMU; }
};