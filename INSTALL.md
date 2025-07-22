# Инструкции по установке и настройке

## Установка PlatformIO

### Вариант 1: Через VS Code (рекомендуется)
1. Установите Visual Studio Code
2. Откройте VS Code и перейдите в раздел Extensions (Ctrl+Shift+X)
3. Найдите "PlatformIO IDE" и установите расширение
4. Перезапустите VS Code
5. Откройте папку проекта в VS Code

### Вариант 2: Через командную строку
```bash
# Установка Python (если не установлен)
# Скачайте с https://www.python.org/downloads/

# Установка PlatformIO Core
pip install platformio

# Или через pip3
pip3 install platformio
```

### Вариант 3: Через Arduino IDE
1. Скачайте Arduino IDE с https://www.arduino.cc/en/software
2. Установите библиотеки через Library Manager:
   - Adafruit BusIO
   - Adafruit HMC5883 Unified
3. Скопируйте код из `src/main.cpp` в Arduino IDE

## Проверка установки

После установки PlatformIO выполните:

```bash
# Проверка версии
pio --version

# Компиляция проекта
pio run

# Загрузка на устройство
pio run --target upload

# Мониторинг серийного порта
pio device monitor
```

## Альтернативная компиляция через Arduino IDE

Если PlatformIO недоступен, используйте Arduino IDE:

1. Откройте Arduino IDE
2. Выберите плату: Tools → Board → ESP32 Arduino → UPEsy Wroom
3. Установите библиотеки через Library Manager:
   - Adafruit BusIO
   - Adafruit HMC5883 Unified
4. Скопируйте код из `src/main.cpp`
5. Нажмите Upload (Ctrl+U)

## Настройка драйверов

Для Windows может потребоваться установка драйверов CP210x или CH340 для работы с серийным портом ESP32.

## Устранение проблем

1. **"pio не является командой"** - переустановите PlatformIO или используйте Arduino IDE
2. **Ошибки компиляции** - проверьте установку библиотек
3. **Проблемы с загрузкой** - проверьте подключение USB и драйверы 