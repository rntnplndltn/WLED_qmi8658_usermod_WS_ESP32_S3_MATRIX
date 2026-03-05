#include "usermod_qmi8658_imu.h"

const char QMI8658Usermod::_name[] PROGMEM = "QMI8658";

const char* const QMI8658Usermod::modeNames[] PROGMEM = {
  "Standby",           // 0
  "Wake-on-Motion",    // 1
  "Neigung",           // 2
  "HW-Tap DSP",        // 3
  "6DOF Standard",     // 4
  "Stromsparend",      // 5
  "FIFO",              // 6
  "Tap+6DOF",          // 7
  "Hohe Bandbreite",   // 8
  "High-G Praezision"  // 9
};

// ─────────────────────────────────────────────
//  handleTap
// ─────────────────────────────────────────────
void QMI8658Usermod::handleTap() {
  if (!(imu.update() & SensorQMI8658::STATUS1_TAP_MOTION)) return;
  switch (imu.getTapStatus()) {
    case SensorQMI8658::SINGLE_TAP:
      DEBUG_PRINTLN(F("QMI8658: Single-Tap -> Effektwechsel"));
      effectCurrent = (effectCurrent + 1) % strip.getModeCount();
      stateUpdated(CALL_MODE_DIRECT_CHANGE);
      break;
    case SensorQMI8658::DOUBLE_TAP:
      DEBUG_PRINTLN(F("QMI8658: Double-Tap -> Palettenwechsel"));
      effectPalette = (effectPalette + 1) % 64;
      stateUpdated(CALL_MODE_DIRECT_CHANGE);
      break;
    default: break;
  }
}

// ─────────────────────────────────────────────
//  applyMode
// ─────────────────────────────────────────────
void QMI8658Usermod::applyMode() {
  if (!initialized) return;

  imu.disableAccelerometer();
  imu.disableGyroscope();
  imu.disableTap();

  switch (imuMode) {

    // ── 0: Standby ───────────────────────────────────────────────────
    case 0:
      DEBUG_PRINTLN(F("QMI8658: Modus 0 - Standby"));
      break;

    // ── 1: Wake-on-Motion ────────────────────────────────────────────
    // configWakeOnMotion() ruft intern reset() + CTRL9 + enableAccel + enableINT auf
    case 1:
      imu.configWakeOnMotion(
        200,                                    // Schwellwert 200mg
        SensorQMI8658::ACC_ODR_LOWPOWER_128Hz,  // ODR Lowpower
        SensorQMI8658::INTERRUPT_PIN_1,         // INT1
        1,                                      // InitialValue high
        0x20                                    // BlankingTime
      );
      DEBUG_PRINTLN(F("QMI8658: Modus 1 - Wake-on-Motion"));
      break;

    // ── 2: Neigung – nur Accel ───────────────────────────────────────
    case 2:
      imu.configAccelerometer(
        SensorQMI8658::ACC_RANGE_8G,
        SensorQMI8658::ACC_ODR_250Hz,
        SensorQMI8658::LPF_MODE_2
      );
      imu.enableAccelerometer();
      DEBUG_PRINTLN(F("QMI8658: Modus 2 - Neigung"));
      break;

    // ── 3: Hardware-Tap DSP ──────────────────────────────────────────
    // enableTap(INT1) setzt intern configActivityInterruptMap() + enableINT()
    case 3:
      imu.configAccelerometer(
        SensorQMI8658::ACC_RANGE_4G,
        SensorQMI8658::ACC_ODR_500Hz,
        SensorQMI8658::LPF_MODE_0
      );
      imu.enableAccelerometer();
      imu.configTap(
        SensorQMI8658::PRIORITY0,
        20, 50, 250,
        0.0625f, 0.25f, 0.8f, 0.4f
      );
      imu.enableTap(SensorQMI8658::INTERRUPT_PIN_1);
      DEBUG_PRINTLN(F("QMI8658: Modus 3 - HW-Tap DSP"));
      break;

    // ── 4: 6DOF Standard ─────────────────────────────────────────────
    case 4:
      imu.configAccelerometer(
        SensorQMI8658::ACC_RANGE_8G,
        SensorQMI8658::ACC_ODR_1000Hz,
        SensorQMI8658::LPF_MODE_2
      );
      imu.configGyroscope(
        SensorQMI8658::GYR_RANGE_512DPS,
        SensorQMI8658::GYR_ODR_896_8Hz,
        SensorQMI8658::LPF_MODE_2
      );
      imu.enableAccelerometer();
      imu.enableGyroscope();
      DEBUG_PRINTLN(F("QMI8658: Modus 4 - 6DOF Standard"));
      break;

    // ── 5: Stromsparend ──────────────────────────────────────────────
    case 5:
      imu.configAccelerometer(
        SensorQMI8658::ACC_RANGE_4G,
        SensorQMI8658::ACC_ODR_31_25Hz,
        SensorQMI8658::LPF_MODE_1
      );
      imu.configGyroscope(
        SensorQMI8658::GYR_RANGE_64DPS,
        SensorQMI8658::GYR_ODR_56_05Hz,
        SensorQMI8658::LPF_MODE_1
      );
      imu.enableAccelerometer();
      imu.enableGyroscope();
      DEBUG_PRINTLN(F("QMI8658: Modus 5 - Stromsparend"));
      break;

    // ── 6: FIFO-gepuffert ────────────────────────────────────────────
    case 6:
      imu.configAccelerometer(
        SensorQMI8658::ACC_RANGE_8G,
        SensorQMI8658::ACC_ODR_1000Hz,
        SensorQMI8658::LPF_MODE_2
      );
      imu.configGyroscope(
        SensorQMI8658::GYR_RANGE_512DPS,
        SensorQMI8658::GYR_ODR_896_8Hz,
        SensorQMI8658::LPF_MODE_2
      );
      imu.enableAccelerometer();
      imu.enableGyroscope();
      imu.configFIFO(
        SensorQMI8658::FIFO_MODE_FIFO,
        SensorQMI8658::FIFO_SAMPLES_16,
        SensorQMI8658::INTERRUPT_PIN_DISABLE,
        16
      );
      DEBUG_PRINTLN(F("QMI8658: Modus 6 - FIFO"));
      break;

    // ── 7: Tap + 6DOF ────────────────────────────────────────────────
    case 7:
      imu.configAccelerometer(
        SensorQMI8658::ACC_RANGE_4G,
        SensorQMI8658::ACC_ODR_500Hz,
        SensorQMI8658::LPF_MODE_0
      );
      imu.configGyroscope(
        SensorQMI8658::GYR_RANGE_512DPS,
        SensorQMI8658::GYR_ODR_448_4Hz,
        SensorQMI8658::LPF_MODE_2
      );
      imu.enableAccelerometer();
      imu.enableGyroscope();
      imu.configTap(
        SensorQMI8658::PRIORITY0,
        20, 50, 250,
        0.0625f, 0.25f, 0.8f, 0.4f
      );
      imu.enableTap(SensorQMI8658::INTERRUPT_PIN_1);
      DEBUG_PRINTLN(F("QMI8658: Modus 7 - Tap+6DOF"));
      break;

    // ── 8: Hohe Bandbreite ───────────────────────────────────────────
    case 8:
      imu.configAccelerometer(
        SensorQMI8658::ACC_RANGE_8G,
        SensorQMI8658::ACC_ODR_1000Hz,
        SensorQMI8658::LPF_MODE_3
      );
      imu.configGyroscope(
        SensorQMI8658::GYR_RANGE_512DPS,
        SensorQMI8658::GYR_ODR_3587_2Hz,
        SensorQMI8658::LPF_MODE_3
      );
      imu.enableAccelerometer();
      imu.enableGyroscope();
      DEBUG_PRINTLN(F("QMI8658: Modus 8 - Hohe Bandbreite"));
      break;

    // ── 9: High-G Praezision ─────────────────────────────────────────
    case 9:
      imu.configAccelerometer(
        SensorQMI8658::ACC_RANGE_16G,
        SensorQMI8658::ACC_ODR_1000Hz,
        SensorQMI8658::LPF_MODE_3
      );
      imu.configGyroscope(
        SensorQMI8658::GYR_RANGE_1024DPS,
        SensorQMI8658::GYR_ODR_896_8Hz,
        SensorQMI8658::LPF_MODE_3
      );
      imu.enableAccelerometer();
      imu.enableGyroscope();
      DEBUG_PRINTLN(F("QMI8658: Modus 9 - High-G Praezision"));
      break;

    default:
      imuMode = 4;
      applyMode();
      break;
  }
}

// ─────────────────────────────────────────────
//  setup
// ─────────────────────────────────────────────
void QMI8658Usermod::setup() {
  // SensorCommon::begin() ruft intern Wire.begin(sda, scl) auf – kein manuelles Wire.begin() noetig
  if (!imu.begin(Wire, QMI8658_L_SLAVE_ADDRESS, SDA_PIN, SCL_PIN)) {
    DEBUG_PRINTLN(F("QMI8658: Init fehlgeschlagen!"));
    initDone = true;
    return;
  }
  initialized = true;
  applyMode();
  DEBUG_PRINTLN(F("QMI8658: Bereit!"));
  initDone = true;
}

// ─────────────────────────────────────────────
//  loop
// ─────────────────────────────────────────────
void QMI8658Usermod::loop() {
  if (!initialized) return;

  switch (imuMode) {

    case 0:
      break;

    case 1: // Wake-on-Motion
      if (imu.update() & SensorQMI8658::STATUS1_WOM_MOTION) {
        DEBUG_PRINTLN(F("QMI8658: WoM ausgeloest!"));
        if (bri == 0) { bri = briLast; stateUpdated(CALL_MODE_DIRECT_CHANGE); }
      }
      break;

    case 2: // Neigung
      if (imu.getDataReady()) {
        imu.getAccelerometer(ax, ay, az);
        temp = imu.getTemperature_C();
      }
      break;

    case 3: // HW-Tap
      handleTap();
      if (imu.getDataReady()) {
        imu.getAccelerometer(ax, ay, az);
        temp = imu.getTemperature_C();
      }
      break;

    case 4: // 6DOF Standard
    case 5: // Stromsparend
    case 8: // Hohe Bandbreite
    case 9: // High-G
      if (imu.getDataReady()) {
        imu.getAccelerometer(ax, ay, az);
        imu.getGyroscope(gx, gy, gz);
        temp = imu.getTemperature_C();
      }
      break;

    case 6: // FIFO
      {
        uint16_t samples = imu.readFromFifo(
          fifoBufAccel, FIFO_BUF_SIZE,
          fifoBufGyro,  FIFO_BUF_SIZE
        );
        if (samples > 0) {
          ax = fifoBufAccel[samples - 1].x;
          ay = fifoBufAccel[samples - 1].y;
          az = fifoBufAccel[samples - 1].z;
          gx = fifoBufGyro[samples - 1].x;
          gy = fifoBufGyro[samples - 1].y;
          gz = fifoBufGyro[samples - 1].z;
          temp = imu.getTemperature_C();
        }
      }
      break;

    case 7: // Tap + 6DOF
      handleTap();
      if (imu.getDataReady()) {
        imu.getAccelerometer(ax, ay, az);
        imu.getGyroscope(gx, gy, gz);
        temp = imu.getTemperature_C();
      }
      break;
  }
}

// ─────────────────────────────────────────────
//  addToJsonInfo
// ─────────────────────────────────────────────
void QMI8658Usermod::addToJsonInfo(JsonObject& root) {
  JsonObject user = root["u"];
  if (user.isNull()) user = root.createNestedObject("u");

  if (!initialized) {
    JsonArray arr = user.createNestedArray(FPSTR(_name));
    arr.add(F("nicht verbunden"));
    return;
  }

  JsonArray arrT = user.createNestedArray(F("IMU Temp (C)"));
  arrT.add(serialized(String(temp, 1)));

  JsonArray arrM = user.createNestedArray(F("IMU Modus"));
  arrM.add(FPSTR(modeNames[imuMode]));
}

// ─────────────────────────────────────────────
//  addToConfig
// ─────────────────────────────────────────────
void QMI8658Usermod::addToConfig(JsonObject& root) {
  JsonObject top = root.createNestedObject(FPSTR(_name));
  top["Modus"] = imuMode;
  top["_Legende"] =
    "0=Standby "
    "1=WakeOnMotion "
    "2=Neigung "
    "3=HW-Tap "
    "4=6DOF-Standard "
    "5=Stromsparend "
    "6=FIFO "
    "7=Tap+6DOF "
    "8=HoheBandbreite "
    "9=HighG";
}

// ─────────────────────────────────────────────
//  readFromConfig
// ─────────────────────────────────────────────
bool QMI8658Usermod::readFromConfig(JsonObject& root) {
  JsonObject top = root[FPSTR(_name)];
  if (top.isNull()) return false;
  uint8_t oldMode = imuMode;
  bool ok = getJsonValue(top["Modus"], imuMode, (uint8_t)4);
  if (initDone && initialized && imuMode != oldMode) {
    applyMode();
    DEBUG_PRINTF("QMI8658: Modus -> %s\n", modeNames[imuMode]);
  }
  return ok;
}

// ─────────────────────────────────────────────
//  Registrierung
// ─────────────────────────────────────────────
static QMI8658Usermod qmi8658_instance;
REGISTER_USERMOD(qmi8658_instance);
