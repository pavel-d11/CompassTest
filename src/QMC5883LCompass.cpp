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
  
  // Configure QMC5883L
  // Control 1: Continuous mode, 200Hz ODR, 8G range, 512 oversampling
  byte control1 = QMC5883L_MODE_CONTINUOUS | QMC5883L_ODR_200HZ | QMC5883L_RNG_8G | QMC5883L_OSR_512;
  if (!writeRegister(QMC5883L_REG_CONFIG_1, control1)) {
    return false;
  }

  // Control 2: Enable interrupt pin
  if (!writeRegister(QMC5883L_REG_CONFIG_2, 0x01)) {
    return false;
  }

  return true;
}

// Function to read compass data
bool QMC5883LCompass::readCompass(int16_t &x, int16_t &y, int16_t &z) {
  // Wait for data to be ready (DRDY bit in status register)
  unsigned long timeout = millis() + 1000;
  bool dataReady = false;
Serial.println(0);
  while (!dataReady && millis() <= timeout) {
    uint8_t status = readRegister(QMC5883L_REG_STATUS);
    Serial.println(1);
    if (i2cError) {
      return false; // Exit on I2C error
    }
    
    Serial.println(2);
    
    if (status & 0x01) {
      dataReady = true; // Data is ready, exit loop
    } else {
      delay(10); // Wait a bit before polling again
    }
  }
  Serial.println(3);
  if (!dataReady) {
    i2cError = true; // Set error flag on timeout
    return false;
  }

  Serial.println("Data ready");

  // Read data registers
  x = readRegister16(QMC5883L_REG_OUT_X_L);
  y = readRegister16(QMC5883L_REG_OUT_Y_L);
  z = readRegister16(QMC5883L_REG_OUT_Z_L);
  Serial.println("Data read");

  // Check if reading was successful
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
  if (!_isCalibrated) return false;

  // Check if calibration range is reasonable
  float xRange = magXmax - magXmin;
  float yRange = magYmax - magYmin;
  
  // For QMC5883L with 8G range, typical values are:
  // - Raw values: -32768 to +32767 (16-bit)
  // - Typical magnetic field strength: ±2000-4000 LSB
  // - A good calibration should have a range > 500 LSB
  float minRange = 500;  // Minimum reasonable range
  float maxRange = 10000; // Maximum reasonable range

  if (xRange < minRange || yRange < minRange) {
    return false; // Range too small - poor calibration
  }

  if (xRange > maxRange || yRange > maxRange) {
    return false; // Range too large - possible sensor error
  }

  // Check if scales are reasonable (should be > 250 for good calibration)
  if (magXscale < 250 || magYscale < 250) {
    return false; // Scale too small - poor calibration
  }

  // Check if scales are not too large (should be < 5000)
  if (magXscale > 5000 || magYscale > 5000) {
    return false; // Scale too large - possible error
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
  // With X=North and Y=West, the formula is atan2(-y, x).
  heading = atan2(-calibratedY, calibratedX);

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

// Function to get tilt angle (total tilt from horizontal)
float QMC5883LCompass::getTiltAngle() {
  int16_t x, y, z;
  if (!readCompass(x, y, z)) {
    return -1; // Error reading sensor
  }

  // Apply calibration if available
  float calX = x, calY = y;
  if (_isCalibrated) {
    calX = (x - magXoffset) / magXscale;
    calY = (y - magYoffset) / magYscale;
  }

  // Calculate tilt angle using DFRobot approach
  // AngleXZ = angle between X-axis and Z-axis (tilt in X-Z plane)
  float tiltAngle = atan2((double)z, sqrt((double)calX * calX + (double)calY * calY)) * 180.0f / PI;
  
  return tiltAngle;
}

// Function to get roll angle (rotation around X-axis)
float QMC5883LCompass::getRollAngle() {
  int16_t x, y, z;
  if (!readCompass(x, y, z)) {
    return -1; // Error reading sensor
  }

  // Apply calibration if available
  float calY = y;
  if (_isCalibrated) {
    calY = (y - magYoffset) / magYscale;
  }

  // Roll is rotation around X-axis (Y-Z plane) - using DFRobot approach
  float rollAngle = atan2((double)z, (double)calY) * 180.0f / PI;
  
  return rollAngle;
}

// Function to get pitch angle (rotation around Y-axis)
float QMC5883LCompass::getPitchAngle() {
  int16_t x, y, z;
  if (!readCompass(x, y, z)) {
    return -1; // Error reading sensor
  }

  // Apply calibration if available
  float calX = x;
  if (_isCalibrated) {
    calX = (x - magXoffset) / magXscale;
  }

  // Pitch is rotation around Y-axis (X-Z plane) - using DFRobot approach
  float pitchAngle = atan2((double)z, (double)calX) * 180.0f / PI;
  
  return pitchAngle;
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