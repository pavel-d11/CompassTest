#include <Arduino.h>
#include "QMC5883LCompass.h"

float headingDegrees = 0;
unsigned long lastUpdate = 0;
const unsigned long UPDATE_INTERVAL = 1000;
QMC5883LCompass compass(&Wire, 0x2C);

void printHelp() {
  Serial.println("Команды:");
  Serial.println("  c - Калибровка компаса");
  Serial.println("  s - Остановить калибровку");
  Serial.println("  r - Сброс калибровки");
  Serial.println("  a - Показать адрес компаса");
  Serial.println("  d - Показать/установить склонение");
  Serial.println("  p - Информация о калибровке");
  Serial.println("  h - Помощь");
}

void setup() {
  Serial.begin(115200);
  Serial.println("QMC5883L Compass Test");
  Wire.begin(21, 22);
  Serial.print("Initializing QMC5883L at address 0x");
  Serial.print(compass.getAddress(), HEX);
  Serial.println("...");
  if (!compass.begin(100000)) {
    Serial.println("Failed to initialize QMC5883L! Check wiring:");
    while (1) {
      delay(1000);
      if (compass.begin(100000)) {
        Serial.println("QMC5883L initialization successful!");
        break;
      }
    }
  }
  Serial.println("QMC5883L initialized successfully!");
  Serial.println("");
  Serial.println("Нажмите 'c' для калибровки компаса или любую другую клавишу для пропуска...");
  unsigned long startWait = millis();
  while (millis() - startWait < 5000) {
    if (Serial.available()) {
      char input = Serial.read();
      if (input == 'c' || input == 'C') {
        compass.startCalibration();
        Serial.println("Калибровка начата. Вращайте модуль во всех направлениях.");
      } else {
        Serial.println("Калибровка пропущена.");
      }
      break;
    }
    delay(100);
  }
  Serial.println("Старт измерений компаса...");
  printHelp();
  Serial.println("========================================");
}

void printCalibrationInfo() {
  CalibrationData data = compass.getCalibrationData();
  if (data.xScale <= 1.0f && data.yScale <= 1.0f) {
    Serial.println("Компас не откалиброван");
    return;
  }
  float xRange = data.xMax - data.xMin;
  float yRange = data.yMax - data.yMin;
  Serial.println("=== Калибровка ===");
  Serial.print("X: "); Serial.print(data.xMin); Serial.print(" .. "); Serial.print(data.xMax);
  Serial.print(" (диапазон: "); Serial.print(xRange); Serial.println(")");
  Serial.print("Y: "); Serial.print(data.yMin); Serial.print(" .. "); Serial.print(data.yMax);
  Serial.print(" (диапазон: "); Serial.print(yRange); Serial.println(")");
  Serial.print("Смещение X: "); Serial.print(data.xOffset, 1);
  Serial.print(", масштаб: "); Serial.println(data.xScale, 1);
  Serial.print("Смещение Y: "); Serial.print(data.yOffset, 1);
  Serial.print(", масштаб: "); Serial.println(data.yScale, 1);
  bool isGood = compass.hasGoodCalibration();
  Serial.print("Качество калибровки: ");
  Serial.println(isGood ? "ХОРОШЕЕ" : "ПЛОХОЕ");
  Serial.println("==================");
}

void loop() {
  unsigned long currentTime = millis();
  if (currentTime - lastUpdate >= UPDATE_INTERVAL) {
    headingDegrees = compass.getHeading();
    if (headingDegrees < 0) {
      Serial.println("Error reading compass sensor!");
      lastUpdate = currentTime;
      return;
    }
    int16_t x, y, z;
    if (!compass.readCompass(x, y, z)) {
      Serial.println("Error reading raw sensor data!");
      lastUpdate = currentTime;
      return;
    }

    Serial.print("Азимут: ");
    Serial.print(headingDegrees, 1);
    Serial.print("° | Raw: X=");
    Serial.print(x);
    Serial.print(" Y=");
    Serial.print(y);
    Serial.print(" Z=");
    Serial.print(z);
    if (compass.hasGoodCalibration()) {
      Serial.print(" [Калиброван]");
    } else {
      CalibrationData data = compass.getCalibrationData();
      if (data.xScale > 1.0f || data.yScale > 1.0f) {
        Serial.print(" [Плохая калибровка]");
      }
    }
    Serial.println();
    lastUpdate = currentTime;
  }
  if (Serial.available()) {
    char input = Serial.read();
    if (input == 'c' || input == 'C') {
      compass.startCalibration();
      Serial.println("Калибровка начата. Вращайте модуль во всех направлениях. Для завершения нажмите 's'.");
    } else if (input == 's' || input == 'S') {
      compass.stopCalibration();
      Serial.println("Калибровка завершена!");
      printCalibrationInfo();
    } else if (input == 'r' || input == 'R') {
      compass.resetCalibration();
      Serial.println("Калибровка сброшена");
    } else if (input == 'a' || input == 'A') {
      Serial.print("Текущий адрес компаса: 0x");
      Serial.println(compass.getAddress(), HEX);
    } else if (input == 'd' || input == 'D') {
      Serial.print("Текущее склонение: ");
      Serial.print(compass.getDeclinationAngle());
      Serial.println("°");
      Serial.println("Введите новое склонение в градусах (или Enter для пропуска):");
      unsigned long startWait = millis();
      while (millis() - startWait < 10000) {
        if (Serial.available()) {
          String inputStr = Serial.readString();
          float angle = inputStr.toFloat();
          if (angle != 0.0f) {
            compass.setDeclinationAngle(angle);
            Serial.print("Склонение установлено: ");
            Serial.print(angle, 1);
            Serial.println("°");
          }
          break;
        }
        delay(100);
      }
    } else if (input == 'p' || input == 'P') {
      printCalibrationInfo();
    } else if (input == 'h' || input == 'H') {
      printHelp();
    }
  }
}