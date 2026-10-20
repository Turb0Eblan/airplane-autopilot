#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Servo.h>

// Конфигурация пинов
static const int RXPin = 3, TXPin = 2;
static const uint32_t GPSBaud = 9600;
const int chipSelect = 4;

// Начальные координаты (точка старта)
static const double START_LAT = 43.23728, START_LON = 76.93131;

// Константы для управления
#define BNO055_SAMPLERATE_DELAY_MS (100)
const unsigned long GPS_READ_INTERVAL = 1000;    // Чтение GPS каждую секунду
const unsigned long SERVO_UPDATE_INTERVAL = 20;  // Обновление сервоприводов каждые 20 мс
const unsigned long LOG_INTERVAL = 5000;         // Логирование каждые 5 секунд
const int RUDDER_NEUTRAL = 90;                  // Нейтральное положение руля
const int RUDDER_LIMIT = 30;                    // Максимальное отклонение руля от нейтрали

// Объекты для устройств
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29, &Wire);
Servo aileronLeft, aileronRight, rudder;  // Переименовал для ясности

// Переменные состояния
unsigned long lastGpsReadTime = 0;
unsigned long lastServoUpdateTime = 0;
unsigned long lastLogTime = 0;
int rudderAngle = RUDDER_NEUTRAL;
bool gpsValid = false;
double currentLat = 0, currentLon = 0;
float currentAlt = 0, currentSpeed = 0, currentCourse = 0;

// PID-регулятор для руля направления
float Kp = 0.5, Ki = 0.01, Kd = 0.1;
float pidErrorSum = 0, lastPidError = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  
  // Инициализация сервоприводов
  aileronLeft.attach(7);
  aileronRight.attach(6);
  rudder.attach(8);
  
  // Инициализация IMU
  if (!bno.begin()) {
    Serial.println("No BNO055 detected");
    while (1);
  }
  bno.setExtCrystalUse(true);
  
  // Инициализация SD-карты
  Serial.print("Initializing SD card...");
  if (!SD.begin(chipSelect)) {
    Serial.println("initialization failed");
    // Продолжаем работу без SD-карты
  } else {
    Serial.println("initialization done.");
  }
  
  // Установка сервоприводов в нейтральное положение
  setNeutralPosition();
}

void loop() {
  unsigned long currentTime = millis();
  
  // Чтение данных GPS
  if (currentTime - lastGpsReadTime >= GPS_READ_INTERVAL) {
    lastGpsReadTime = currentTime;
    readGPSData();
  }
  
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
  bool newData = false;
  
  // Чтение данных GPS в течение 100 мс
  ss.begin(GPSBaud);
  unsigned long start = millis();
  while (millis() - start < 100) {
    while (ss.available()) {
      char c = ss.read();
      if (gps.encode(c)) newData = true;
    }
  }
  ss.end();
  
  if (newData) {
    gpsValid = gps.location.isValid() && gps.altitude.isValid() && 
               gps.speed.isValid() && gps.course.isValid();
    
    if (gpsValid) {
      currentLat = gps.location.lat();
      currentLon = gps.location.lng();
      currentAlt = gps.altitude.meters();
      currentSpeed = gps.speed.kmph();
      currentCourse = gps.course.deg();
      
      // Рассчитываем угол отклонения руля с помощью PID-регулятора
      calculateRudderAngle();
    }
  } else {
    gpsValid = false;
    rudderAngle = RUDDER_NEUTRAL; // Если нет GPS, устанавливаем руль в нейтральное положение
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
  pidErrorSum += error;
  float dError = error - lastPidError;
  lastPidError = error;
  
  float pidOutput = Kp * error + Ki * pidErrorSum + Kd * dError;
  
  // Преобразуем выход PID в угол сервопривода
  rudderAngle = RUDDER_NEUTRAL + constrain(pidOutput, -RUDDER_LIMIT, RUDDER_LIMIT);
}

void controlServos() {
  if (!bno.begin()) return; // Проверка IMU
  
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
  Serial.print("Lat: "); Serial.print(currentLat, 6);
  Serial.print(", Lon: "); Serial.print(currentLon, 6);
  Serial.print(", Alt: "); Serial.print(currentAlt);
  Serial.print(", Speed: "); Serial.print(currentSpeed);
  Serial.print(", Course: "); Serial.print(currentCourse);
  Serial.print(", Rudder: "); Serial.println(rudderAngle);
  
  // Запись на SD-карту
  if (SD.begin(chipSelect)) {
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
}