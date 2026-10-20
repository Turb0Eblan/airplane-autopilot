// Прошивка для Arduino Nano (на основе PilotExtended)
//
// Почему раньше дёргались сервы:
//   SoftwareSerial на время приёма КАЖДОГО байта (~1 мс на 9600) запрещает
//   прерывания. Библиотека Servo формирует импульс 1–2 мс в прерывании Timer1,
//   и если это прерывание опоздало — импульс становится длиннее, серва дёргается.
//   GPS шлёт пачку NMEA раз в секунду -> рывок раз в секунду.
//   Отключение серв (detach) убирало рывки, но и управление на это время.
//
// Что сделано:
//   1. SoftwareSerial заменён на NeoSWSerial — он не держит прерывания
//      запрещёнными, обработчик короткий, поэтому импульсы Servo остаются ровными.
//      Проводка не меняется: GPS всё так же на пинах 3 (RX) / 2 (TX).
//   2. GPS читается непрерывно в loop() без блокирующих окон по 100 мс
//      и без ss.begin()/ss.end() каждый раз.
//   3. Убран bno.begin() из controlServos(): он вызывался каждые 20 мс и
//      каждый раз перезапускал BNO055 (сброс + задержки сотни мс).
//   4. SD.begin() вызывается один раз в setup(), а не при каждой записи.
//   5. В PID добавлено ограничение интеграла (anti-windup).

#include <Arduino.h>
#include <TinyGPS++.h>
#include <NeoSWSerial.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Servo.h>

// Конфигурация пинов
static const int RXPin = 3, TXPin = 2;
static const uint32_t GPSBaud = 9600;  // NeoSWSerial поддерживает 9600/19200/31250/38400
const int chipSelect = 4;

// Начальные координаты (точка старта)
static const double START_LAT = 43.23728, START_LON = 76.93131;

// Константы для управления
const unsigned long GPS_TIMEOUT = 3000;          // Нет свежих данных GPS дольше — руль в нейтраль
const unsigned long SERVO_UPDATE_INTERVAL = 20;  // Обновление сервоприводов каждые 20 мс
const unsigned long LOG_INTERVAL = 5000;         // Логирование каждые 5 секунд
const int RUDDER_NEUTRAL = 90;                  // Нейтральное положение руля
const int RUDDER_LIMIT = 30;                    // Максимальное отклонение руля от нейтрали
const float PID_I_LIMIT = 1000;                 // Ограничение накопленной ошибки (anti-windup)

// Объекты для устройств
TinyGPSPlus gps;
NeoSWSerial ss(RXPin, TXPin);
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29, &Wire);
Servo aileronLeft, aileronRight, rudder;

// Переменные состояния
unsigned long lastGpsFixTime = 0;
unsigned long lastServoUpdateTime = 0;
unsigned long lastLogTime = 0;
int rudderAngle = RUDDER_NEUTRAL;
bool gpsValid = false;
bool sdReady = false;
double currentLat = 0, currentLon = 0;
float currentAlt = 0, currentSpeed = 0, currentCourse = 0;

// PID-регулятор для руля направления
float Kp = 0.5, Ki = 0.01, Kd = 0.1;
float pidErrorSum = 0, lastPidError = 0;

void readGPSData();
void calculateRudderAngle();
void controlServos();
void setNeutralPosition();
void logData();

void setup() {
  Serial.begin(115200);

  // Инициализация сервоприводов
  aileronLeft.attach(7);
  aileronRight.attach(6);
  rudder.attach(8);

  // Инициализация IMU
  if (!bno.begin()) {
    Serial.println(F("No BNO055 detected"));
    while (1);
  }
  bno.setExtCrystalUse(true);

  // Инициализация SD-карты
  Serial.print(F("Initializing SD card..."));
  sdReady = SD.begin(chipSelect);
  // Продолжаем работу и без SD-карты
  Serial.println(sdReady ? F("initialization done.") : F("initialization failed"));

  // Установка сервоприводов в нейтральное положение
  setNeutralPosition();

  ss.begin(GPSBaud);
}

void loop() {
  unsigned long currentTime = millis();

  // Чтение данных GPS — каждый проход, без блокировки
  readGPSData();

  // Управление сервоприводами
  if (currentTime - lastServoUpdateTime >= SERVO_UPDATE_INTERVAL) {
    lastServoUpdateTime = currentTime;
    controlServos();
  }

  // Логирование данных
  if (currentTime - lastLogTime >= LOG_INTERVAL && gpsValid) {
    lastLogTime = currentTime;
    logData();
  }
}

void readGPSData() {
  while (ss.available()) {
    gps.encode(ss.read());
  }

  if (gps.location.isUpdated()) {
    gpsValid = gps.location.isValid() && gps.altitude.isValid() &&
               gps.speed.isValid() && gps.course.isValid();

    if (gpsValid) {
      lastGpsFixTime = millis();
      currentLat = gps.location.lat();
      currentLon = gps.location.lng();
      currentAlt = gps.altitude.meters();
      currentSpeed = gps.speed.kmph();
      currentCourse = gps.course.deg();

      // Рассчитываем угол отклонения руля с помощью PID-регулятора
      calculateRudderAngle();
    }
  }

  // Если GPS давно не давал координат, устанавливаем руль в нейтральное положение
  if (gpsValid && millis() - lastGpsFixTime > GPS_TIMEOUT) {
    gpsValid = false;
  }
  if (!gpsValid) {
    rudderAngle = RUDDER_NEUTRAL;
    pidErrorSum = 0;
    lastPidError = 0;
  }
}

void calculateRudderAngle() {
  if (!gpsValid) return;

  // Рассчитываем курс к точке старта
  float targetCourse = TinyGPSPlus::courseTo(currentLat, currentLon, START_LAT, START_LON);
  float error = targetCourse - currentCourse;

  // Нормализуем ошибку в диапазон [-180, 180]
  while (error > 180) error -= 360;
  while (error < -180) error += 360;

  // PID-регулятор
  pidErrorSum = constrain(pidErrorSum + error, -PID_I_LIMIT, PID_I_LIMIT);
  float dError = error - lastPidError;
  lastPidError = error;

  float pidOutput = Kp * error + Ki * pidErrorSum + Kd * dError;

  // Преобразуем выход PID в угол сервопривода
  rudderAngle = RUDDER_NEUTRAL + constrain(pidOutput, -RUDDER_LIMIT, RUDDER_LIMIT);
}

void controlServos() {
  sensors_event_t event;
  bno.getEvent(&event);

  // Управление элеронами на основе крена
  float roll = event.orientation.y; // Крен
  float pitch = event.orientation.z; // Тангаж

  // Простое управление элеронами (можно улучшить)
  int aileronLeftPos = 90 + roll - 0.5*pitch;
  int aileronRightPos = 90 - roll - 0.5*pitch;

  // Ограничиваем углы
  aileronLeftPos = constrain(aileronLeftPos, 45, 135);
  aileronRightPos = constrain(aileronRightPos, 45, 135);

  // Управление сервоприводами
  aileronLeft.write(aileronLeftPos);
  aileronRight.write(aileronRightPos);
  rudder.write(rudderAngle);
}

void setNeutralPosition() {
  aileronLeft.write(90);
  aileronRight.write(90);
  rudder.write(RUDDER_NEUTRAL);
  delay(500); // Даем время сервам занять позицию
}

void logData() {
  if (!gpsValid) return;

  // Вывод в Serial
  Serial.print(F("Lat: ")); Serial.print(currentLat, 6);
  Serial.print(F(", Lon: ")); Serial.print(currentLon, 6);
  Serial.print(F(", Alt: ")); Serial.print(currentAlt);
  Serial.print(F(", Speed: ")); Serial.print(currentSpeed);
  Serial.print(F(", Course: ")); Serial.print(currentCourse);
  Serial.print(F(", Rudder: ")); Serial.println(rudderAngle);

  // Запись на SD-карту
  if (!sdReady) return;
  File dataFile = SD.open("datalog.txt", FILE_WRITE);
  if (dataFile) {
    dataFile.print(millis());
    dataFile.print(", ");
    dataFile.print(currentLat, 6);
    dataFile.print(", ");
    dataFile.print(currentLon, 6);
    dataFile.print(", ");
    dataFile.print(currentAlt);
    dataFile.print(", ");
    dataFile.print(currentSpeed);
    dataFile.print(", ");
    dataFile.print(currentCourse);
    dataFile.print(", ");
    dataFile.print(rudderAngle);

    // Добавляем данные IMU
    sensors_event_t event;
    bno.getEvent(&event);
    dataFile.print(", ");
    dataFile.print(event.orientation.x, 2);
    dataFile.print(", ");
    dataFile.print(event.orientation.y, 2);
    dataFile.print(", ");
    dataFile.println(event.orientation.z, 2);

    dataFile.close();
  }
}
