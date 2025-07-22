# QMC5883L Compass Test

Проект для работы с магнитным компасом QMC5883L на ESP32 с использованием PlatformIO.

## Описание

Этот проект демонстрирует работу с датчиком QMC5883L для получения точных показаний азимута. Код организован в виде модульного класса `QMC5883LCompass`, который инкапсулирует всю логику работы с датчиком.

## Структура проекта

```
CompassTest/
├── src/
│   ├── main.cpp              # Основной файл с setup() и loop()
│   ├── QMC5883LCompass.h     # Заголовочный файл класса
│   └── QMC5883LCompass.cpp   # Реализация класса
├── platformio.ini            # Конфигурация PlatformIO
└── README.md                 # Документация
```

## Подключение

Подключите QMC5883L к ESP32 следующим образом:

- **SDA** → GPIO21
- **SCL** → GPIO22  
- **VCC** → 3.3V
- **GND** → GND

## Класс QMC5883LCompass

### Структура данных калибровки
```cpp
struct CalibrationData {
  float xMin, xMax, yMin, yMax;     // Минимальные и максимальные значения
  float xOffset, yOffset;           // Смещения
  float xScale, yScale;             // Масштабные коэффициенты
  bool isValid;                     // Валидность данных
};
```

### Основные методы

#### Инициализация
```cpp
// Инициализация I2C (в основном коде)
Wire.begin(21, 22);  // SDA=21, SCL=22

// Создание объекта с обязательным указанием Wire и адреса
QMC5883LCompass compass(&Wire, 0x2C);  // Использовать стандартный Wire и QMC5883L адрес
// QMC5883LCompass compass(&Wire1, 0x0D);  // Использовать Wire1 и альтернативный адрес
bool success = compass.begin(100000);  // 100kHz clock speed

// Получение и изменение параметров
uint8_t addr = compass.getAddress();
compass.setAddress(0x0D);
TwoWire* wire = compass.getWire();
compass.setWire(&Wire1);

// Магнитная деклинация
compass.setDeclinationAngle(11.5 * PI / 180.0f);  // Москва: 11.5°
float declination = compass.getDeclinationAngle();
```

#### Получение данных
```cpp
float heading = compass.getHeading();           // Азимут в градусах
bool success = compass.readCompass(x, y, z);    // Сырые данные
```

#### Калибровка
```cpp
// Начать калибровку
compass.startCalibration();

// Калибровка обновляется автоматически при каждом чтении данных
// (в getHeading(), getTiltAngle(), readCompass() и т.д.)

// Остановить калибровку и рассчитать коэффициенты
compass.stopCalibration();

// Проверка качества калибровки
if (compass.hasGoodCalibration()) {
  Serial.println("Calibration quality is good");
}

// Сброс калибровки
compass.resetCalibration();

// Работа с данными калибровки
CalibrationData data = compass.getCalibrationData();
compass.setCalibrationData(data);

// Пример сохранения в EEPROM
#include <EEPROM.h>
CalibrationData calData = compass.getCalibrationData();
EEPROM.put(0, calData);
EEPROM.commit();

// Пример загрузки из EEPROM
CalibrationData loadedData;
EEPROM.get(0, loadedData);
if (loadedData.isValid) {
  compass.setCalibrationData(loadedData);
}

#### Вспомогательные функции
```cpp
String cardinal = QMC5883LCompass::getCardinalDirection(degrees);
String detailed = QMC5883LCompass::getDetailedDirection(degrees);
```

### Особенности реализации

1. **Точная калибровка**: Использует смещение и масштабирование для компенсации магнитных помех
2. **Проверка качества**: Автоматическая проверка разумности калибровочных данных
3. **Деклинация**: Учитывает магнитную деклинацию для Москвы (11.5°)
4. **Обработка ошибок**: Встроенная диагностика I2C ошибок

## Использование

### Компиляция и загрузка
```bash
pio run --target upload
```

### Мониторинг
```bash
pio device monitor
```

### Команды через Serial Monitor

- **c/C** - Начать калибровку компаса
- **s/S** - Остановить калибровку
- **r/R** - Сброс калибровки
- **t/T** - Тест ориентации
- **f/F** - Тест всех формул
- **i/I** - Тест углов наклона
- **a/A** - Показать адрес компаса
- **d/D** - Показать угол деклинации
- **p/P** - Показать информацию о калибровке
- **g/G** - Получить данные калибровки
- **l/L** - Загрузить демо-данные калибровки
- **h/H** - Показать справку

## Пример вывода

```
QMC5883L Compass Test
=====================
QMC5883L initialized successfully!

Starting compass readings...
Format: Azimuth (degrees) | Cardinal | Detailed | Raw X, Y, Z
Commands: c=calibrate, r=reset, h=help
=============================================================
Azimuth: 45.2° | NE | Northeast | Raw: X=1234 Y=567 Z=890 [Calibrated]
```

## Калибровка

Для получения точных показаний рекомендуется провести калибровку:

1. Отправьте команду `c` через Serial Monitor для начала калибровки
2. Вращайте компас во всех направлениях (включая наклоны)
3. Отправьте команду `s` для остановки калибровки
4. Избегайте металлических предметов и магнитных помех

**Важно**: Калибровка обновляется в реальном времени. Чем дольше вы вращаете компас, тем лучше будет калибровка.

## Технические характеристики

- **Датчик**: QMC5883L
- **Интерфейс**: I2C
- **Частота обновления**: 10 Гц
- **Диапазон**: ±2G
- **Разрешение**: 16 бит
- **Деклинация**: Настраиваемая (по умолчанию 0°)

## Ориентация осей

- **X-ось**: указывает на **Север** (North)
- **Y-ось**: указывает на **Восток** (East) 
- **Z-ось**: указывает вверх (Up)

## Система координат

**Heading измеряется по часовой стрелке относительно севера:**
- **0°** = Север (North)
- **90°** = Восток (East)
- **180°** = Юг (South)
- **270°** = Запад (West)

**Важно**: В коде используется формула `atan2(y, -x)` для корректного расчета азимута в стандартной системе координат компаса.

## Лицензия

Этот проект распространяется под лицензией MIT. 