#pragma once
#include "wled.h"
#include <Wire.h>
#include "SensorQMI8658.hpp"

class QMI8658Usermod : public Usermod {
private:
  static const char _name[];
  static const char* const modeNames[];

  SensorQMI8658 imu;
  bool initialized = false;
  bool initDone    = false;

  static constexpr int8_t SDA_PIN  = 11;
  static constexpr int8_t SCL_PIN  = 12;

  float ax = 0, ay = 0, az = 0;
  float gx = 0, gy = 0, gz = 0;
  float temp = 0;

  static constexpr uint8_t FIFO_BUF_SIZE = 16;
  IMUdata fifoBufAccel[FIFO_BUF_SIZE];
  IMUdata fifoBufGyro[FIFO_BUF_SIZE];

  uint8_t imuMode = 4;

  void applyMode();
  void handleTap();

public:
  void setup() override;
  void loop() override;
  void addToJsonInfo(JsonObject& root) override;
  void addToConfig(JsonObject& root) override;
  bool readFromConfig(JsonObject& root) override;
  uint16_t getId() override { return USERMOD_ID_IMU; }
};
