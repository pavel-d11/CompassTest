#include "QMC5883LCompass.h"

// Constructor
QMC5883LCompass::QMC5883LCompass(TwoWire* wire, uint8_t address) {
  // Initialize variables
  _wire = wire;  // Set the Wire object
  QMC5883L_ADDR = address;  // Set the I2C address
  magXmin = magXmax = 0;
  magYmin = magYmax = 0;
  magXoffset = magYoffset = 0;
  magXscale = magYscale = 1.0f;
  _isCalibrated = false;
  i2cError = false;
}

// Function to write to QMC5883L register
bool QMC5883LCompass::writeRegister(uint8_t reg, uint8_t value) {
  _wire->beginTransmission(QMC5883L_ADDR);
  _wire->write(reg);
  _wire->write(value);
  uint8_t error = _wire->endTransmission();
  i2cError = (error != 0);
  return !i2cError;
}

// Function to read from QMC5883L register
uint8_t QMC5883LCompass::readRegister(uint8_t reg) {
  _wire->beginTransmission(QMC5883L_ADDR);
  _wire->write(reg);
  _wire->endTransmission(false);
  _wire->requestFrom((uint8_t)QMC5883L_ADDR, (uint8_t)1);
  if (_wire->available()) {
    return _wire->read();
  }

  i2cError = true;
  return 0;
}

// Function to read 16-bit value from QMC5883L
int16_t QMC5883LCompass::readRegister16(uint8_t reg) {
  _wire->beginTransmission(QMC5883L_ADDR);
  _wire->write(reg);
  _wire->endTransmission(false);
  _wire->requestFrom((uint8_t)QMC5883L_ADDR, (uint8_t)2);
  if (_wire->available() >= 2) {
    int16_t value = _wire->read();
    value |= _wire->read() << 8;
    return value;
  }

  i2cError = true;
  return 0;
}

// Function to initialize QMC5883L
bool QMC5883LCompass::begin(uint32_t clockSpeed) {
  // Set I2C clock speed
  _wire->setClock(clockSpeed);
  delay(100); // Give sensor time to initialize

  // Check if device responds
  _wire->beginTransmission(QMC5883L_ADDR);
  uint8_t error = _wire->endTransmission();
  if (error != 0) {
    i2cError = true;
    return false;
  }
  
  // Perform a soft reset to ensure a clean state
  if (!writeRegister(QMC5883L_REG_CONFIG_2, 0x80)) {
    return false;
  }
  delay(100);

  // --- STAGE 1: Configure sensor parameters in STANDBY mode ---

  // Set SET/RESET period - crucial for some clones
  if (!writeRegister(QMC5883L_REG_SET_RESET, 0x01)) {
    return false;
  }
  
  // Configure Control Register 2: Disable interrupt pin
  if (!writeRegister(QMC5883L_REG_CONFIG_2, 0x01)) {
    return false;
  }

  // Configure Control Register 1: Set ODR, Range, OSR, but keep in STANDBY mode
  byte control1_standby = QMC5883L_MODE_STANDBY | QMC5883L_ODR_50HZ | QMC5883L_RNG_8G | QMC5883L_OSR_512;
  if (!writeRegister(QMC5883L_REG_CONFIG_1, control1_standby)) {
    return false;
  }

  // --- STAGE 2: Wake up the sensor by switching to CONTINUOUS mode ---
  
  byte control1_continuous = QMC5883L_MODE_CONTINUOUS | QMC5883L_ODR_10HZ | QMC5883L_RNG_8G | QMC5883L_OSR_512;
  if (!writeRegister(QMC5883L_REG_CONFIG_1, control1_continuous)) {
    return false;
  }

  delay(10); // Give sensor time to start measuring

  return true;
}

// Function to read compass data
// NOTE: Due to a bug in this specific sensor clone, the DRDY status flag
// is unreliable. This function performs a direct read of the data registers.
// It is the responsibility of the calling code to ensure this function is
// not called faster than the Output Data Rate (e.g., 10Hz -> every 100ms).
bool QMC5883LCompass::readCompass(int16_t &x, int16_t &y, int16_t &z) {
  // Direct read of data registers, bypassing status check.
  x = readRegister16(QMC5883L_REG_OUT_X_L);
  y = readRegister16(QMC5883L_REG_OUT_Y_L);
  z = readRegister16(QMC5883L_REG_OUT_Z_L);

  if (i2cError) {
    return false; // Check if the read operations failed
  }

  // A basic check to see if the sensor is stuck on zero
  if (x == 0 && y == 0 && z == 0) {
    return false;
  }

  // Update calibration if in progress
  if (_isCalibrating) {
    updateCalibration(x, y);
  }

  return true;
}

// Function to check calibration quality
bool QMC5883LCompass::hasGoodCalibration() const {
  if (!_isCalibrated) {
    return false;
  }

  // 1. Check for sensor saturation
  // If min/max values are at the 16-bit integer limits, it means the
  // magnetic field was stronger than the sensor's range.
  const int16_t limit = 32760; // Use a value slightly lower than 32767
  if (abs(magXmin) >= limit || abs(magXmax) >= limit ||
      abs(magYmin) >= limit || abs(magYmax) >= limit) {
    return false; // Saturation detected
  }

  // 2. Check for reasonable scale values
  // Scales should be positive and not excessively small.
  if (magXscale < 100.0f || magYscale < 100.0f) {
    return false; // Scale is too small, likely a failed calibration
  }

  // 3. Check for ellipticity
  // For a good calibration, the scales of X and Y axes should be similar.
  // If one is drastically larger than the other, the calibration is skewed.
  float ratio = magXscale / magYscale;
  if (ratio < 0.33f || ratio > 3.0f) {
    return false; // Ellipticity is too high
  }

  return true;
}

// Function to get compass heading
float QMC5883LCompass::getHeading() {
  int16_t x, y, z;
  if (!readCompass(x, y, z)) {
    return -1; // Error reading sensor
  }

  float heading;
  float calibratedX, calibratedY;
  
  if (_isCalibrated && hasGoodCalibration()) {
    // Apply calibration using offset and scale factors
    calibratedX = (x - magXoffset) / magXscale;
    calibratedY = (y - magYoffset) / magYscale;
  } else {
    // Use raw values
    calibratedX = x;
    calibratedY = y;
  }
  // Calculate heading: angle clockwise from North.
  // Swapped X and Y in the formula to match the actual sensor axis orientation.
  heading = atan2(-calibratedX, calibratedY);

  // Add declination angle correction (convert declination from degrees to radians)
  heading += declinationAngle * PI / 180.0f;

  // Normalize to 0-2π
  if (heading < 0) {
    heading += 2 * PI;
  }
  if (heading > 2 * PI) {
    heading -= 2 * PI;
  }

  // Convert to degrees
  float headingDegrees = heading * 180.0f / PI;
  
  return headingDegrees;
}

// Function to start calibration
void QMC5883LCompass::startCalibration() {
  // Initialize min/max values with first reading
  int16_t x, y, z;
  if (readCompass(x, y, z)) {
    magXmin = magXmax = x;
    magYmin = magYmax = y;
    _isCalibrating = true;
  }
}

// Function to update calibration with current readings
void QMC5883LCompass::updateCalibration(int16_t x, int16_t y) {
  if (!_isCalibrating) {
    return; // Not calibrating
  }

  // Update min/max values
  if (x < magXmin) magXmin = x;
  if (x > magXmax) magXmax = x;
  if (y < magYmin) magYmin = y;
  if (y > magYmax) magYmax = y;
}

// Function to stop calibration and calculate factors
void QMC5883LCompass::stopCalibration() {
  if (!_isCalibrating) {
    return; // Not calibrating
  }

  // Calculate offset and scale factors
  magXoffset = (magXmax + magXmin) / 2.0f;
  magYoffset = (magYmax + magYmin) / 2.0f;
  magXscale = (magXmax - magXmin) / 2.0f;
  magYscale = (magYmax - magYmin) / 2.0f;

  _isCalibrated = true;
  _isCalibrating = false;
}

// Function to reset calibration
void QMC5883LCompass::resetCalibration() {
  _isCalibrated = false;
  _isCalibrating = false;
  magXmin = magXmax = 0;
  magYmin = magYmax = 0;
  magXoffset = magYoffset = 0;
  magXscale = magYscale = 1.0f;
}

// Function to get calibration data
CalibrationData QMC5883LCompass::getCalibrationData() const {
  CalibrationData data;
  data.xMin = magXmin;
  data.xMax = magXmax;
  data.yMin = magYmin;
  data.yMax = magYmax;
  data.xOffset = magXoffset;
  data.yOffset = magYoffset;
  data.xScale = magXscale;
  data.yScale = magYscale;
  data.isValid = _isCalibrated && hasGoodCalibration();
  return data;
}

// Function to set calibration data
void QMC5883LCompass::setCalibrationData(const CalibrationData& data) {
  if (data.isValid) {
    magXmin = data.xMin;
    magXmax = data.xMax;
    magYmin = data.yMin;
    magYmax = data.yMax;
    magXoffset = data.xOffset;
    magYoffset = data.yOffset;
    magXscale = data.xScale;
    magYscale = data.yScale;
    _isCalibrated = true;
    _isCalibrating = false;
  } else {
    resetCalibration();
  }
}