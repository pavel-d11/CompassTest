#ifndef QMC5883L_COMPASS_H
#define QMC5883L_COMPASS_H

#include <Arduino.h>
#include <Wire.h>

// Structure to hold calibration data
struct CalibrationData {
  float xMin, xMax, yMin, yMax;
  float xOffset, yOffset;
  float xScale, yScale;
  bool isValid;
  
  CalibrationData() : xMin(0), xMax(0), yMin(0), yMax(0), 
                      xOffset(0), yOffset(0), xScale(1.0f), yScale(1.0f), 
                      isValid(false) {}
};

class QMC5883LCompass {
private:
  // I2C configuration
  TwoWire* _wire;  // Pointer to Wire object
  uint8_t QMC5883L_ADDR;  // Configurable address
  static const uint8_t QMC5883L_REG_OUT_X_L = 0x00;
  static const uint8_t QMC5883L_REG_OUT_Y_L = 0x02;
  static const uint8_t QMC5883L_REG_OUT_Z_L = 0x04;
  static const uint8_t QMC5883L_REG_STATUS = 0x06;
  static const uint8_t QMC5883L_REG_CONFIG_1 = 0x09;
  static const uint8_t QMC5883L_REG_CONFIG_2 = 0x0A;

// Control Register 1 bits

  #define QMC5883L_MODE_CONTINUOUS 0x01 // Continuous mode
  #define QMC5883L_ODR_200HZ 0x03 // 10 Hz output data rate
  #define QMC5883L_RNG_8G 0x01 // 2 gauss full scale
  #define QMC5883L_OSR_512 0x00 // 512 samples per measurement

  // Calibration variables
  int16_t magXmin = 0, magXmax = 0;
  int16_t magYmin = 0, magYmax = 0;
  float magXoffset = 0, magYoffset = 0;
  float magXscale = 1.0f, magYscale = 1.0f;
  bool _isCalibrated = false;
  bool _isCalibrating = false;
  
  // Magnetic declination
  float declinationAngle = 0.0f;  // Default declination angle

  // I2C error handling
  bool i2cError = false;

  // Private methods
  bool writeRegister(uint8_t reg, uint8_t value);
  uint8_t readRegister(uint8_t reg);
  int16_t readRegister16(uint8_t reg);
  void updateCalibration(int16_t x, int16_t y);
  bool isCalibrating() const { return _isCalibrating; }
  bool isCalibrated() const { return this->_isCalibrated; }

public:
  // Constructor
  QMC5883LCompass(TwoWire* wire, uint8_t address);  // Wire and address must be specified

  // Initialization
  bool begin(uint32_t clockSpeed = 100000);

  // Data reading
  // Orientation based on PCB marking: X-axis points North, Y-axis points West.
  // Heading is measured clockwise from North (0° = North).
  bool readCompass(int16_t &x, int16_t &y, int16_t &z);
  float getHeading();
  float getTiltAngle();
  float getRollAngle();
  float getPitchAngle();

  // Calibration
  void startCalibration();
  void stopCalibration();
  bool hasGoodCalibration() const;
  void resetCalibration();
  
  // Calibration data management
  CalibrationData getCalibrationData() const;
  void setCalibrationData(const CalibrationData& data);
  

  // Status and configuration
  bool hasError() const { return i2cError; }
  void clearError() { i2cError = false; }
  uint8_t getAddress() const { return QMC5883L_ADDR; }
  void setAddress(uint8_t address) { QMC5883L_ADDR = address; }
  TwoWire* getWire() const { return _wire; }
  void setWire(TwoWire* wire) { _wire = wire; }
  
  // Magnetic declination
  void setDeclinationAngle(float angle) { declinationAngle = angle; }
  float getDeclinationAngle() const { return declinationAngle; }
  
};

#endif // QMC5883L_COMPASS_H 