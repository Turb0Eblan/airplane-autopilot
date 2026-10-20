#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Servo.h>

static const int RXPin = 3, TXPin = 2;
static const uint32_t GPSBaud = 9600;
const int chipSelect = 4;
int posy = 0; 
int posz = 0; 

#define BNO055_SAMPLERATE_DELAY_MS (100)

TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29, &Wire);
Servo servo1; 
Servo servo2; 
Servo servo3; 

// Временные интервалы для задач
const unsigned long gpsReadInterval = 10000;  // Чтение GPS каждые 10 секунд
const unsigned long servoControlInterval = 100;  // Управление сервоприводом каждые 100 мс

// Переменные для хранения временных меток
unsigned long previousGPSReadTime = 0;
unsigned long previousServoControlTime = 0;

// Начальные координаты (точка старта)
static const double START_LAT = 43.23728, START_LON = 76.93131;

// Угол для сервопривода руля направления
int rudderAngle = 90;  // Нейтральное положение

void setup()
{
  servo1.attach(7);
  servo2.attach(6);
  servo3.attach(8);
  
  Serial.begin(115200);

  while (!Serial);

  if (!bno.begin())
  {
    Serial.print("No BNO055 detected");
    while (1);
  }

  Serial.print("Initializing SD card...");
  if (!SD.begin(chipSelect)) {
    Serial.println("initialization failed");
    while (true);
  }
  Serial.println("initialization done.");
  
  bno.setExtCrystalUse(true);
}

void loop() {
  unsigned long currentTime = millis();

  // Задача 1: Чтение данных GPS
  if (currentTime - previousGPSReadTime >= gpsReadInterval) {
    previousGPSReadTime = currentTime;  // Обновляем временную метку
    readGPSData();
  }

  // Задача 2: Управление сервоприводом
  if (currentTime - previousServoControlTime >= servoControlInterval) {
    previousServoControlTime = currentTime;  // Обновляем временную метку
    controlServo();
  }
}

// Функция для чтения данных GPS
void readGPSData() {
  // Отключаем сервоприводы на время чтения данных GPS
  detachServos();

  // Включаем SoftwareSerial для чтения данных GPS
  ss.begin(GPSBaud);
  bool gpsDataReceived = false;
  unsigned long startTime = millis();

  // Чтение данных GPS в течение 500 мс
  while (millis() - startTime < 500) {
    while (ss.available()) {
      char c = ss.read();
      if (gps.encode(c)) {
        gpsDataReceived = true;
      }
    }
  }
  ss.end();  // Отключаем SoftwareSerial после чтения данных

  // Подключаем сервоприводы обратно
  attachServos();

  if (gpsDataReceived) {
    // Вывод данных GPS в Serial Monitor (в старом формате)
    Serial.print(gps.satellites.value());  // Информация о спутниках
    Serial.print(", ");
    Serial.print(gps.hdop.hdop());  // Точность
    Serial.print(", ");
    Serial.print(gps.location.lat(), 6);  // Широта
    Serial.print(", ");
    Serial.print(gps.location.lng(), 6);  // Долгота
    Serial.print(", ");
    Serial.print(gps.altitude.meters());  // Высота
    Serial.print(", ");
    Serial.print(gps.course.deg());  // Направление движения
    Serial.print(", ");
    Serial.print(gps.speed.kmph());  // Скорость
    Serial.print(", ");

    // Расстояние до старта
    unsigned long distanceKmToSTART = (unsigned long)TinyGPSPlus::distanceBetween(
      gps.location.lat(), gps.location.lng(), START_LAT, START_LON);
    Serial.print(distanceKmToSTART);
    Serial.print(", ");

    // Курс к старту
    double courseToSTART = TinyGPSPlus::courseTo(
      gps.location.lat(), gps.location.lng(), START_LAT, START_LON);
    Serial.print(courseToSTART);
    Serial.print(", ");

    // Направление к старту
    const char *cardinalToSTART = TinyGPSPlus::cardinal(courseToSTART);
    Serial.print(cardinalToSTART);
    Serial.print(", ");

    // Направления (x, y, z)
    sensors_event_t event;
    bno.getEvent(&event);
    Serial.print(event.orientation.x, 4);
    Serial.print(", ");
    Serial.print(event.orientation.y, 4);
    Serial.print(", ");
    Serial.print(event.orientation.z, 4);
    Serial.println(", ");

    // Управление рулём направления
    rudderAngle = map(courseToSTART, 0, 360, 45, 135);  // Преобразуем курс в угол сервопривода
    rudderAngle = constrain(rudderAngle, 45, 135);  // Ограничиваем угол в пределах 45-135 градусов

    // Запись данных в файл
    File dataFile = SD.open("datalog.txt", FILE_WRITE);
    if (dataFile) {
      dataFile.print(gps.location.lat(), 6);
      dataFile.print(", ");
      dataFile.print(gps.location.lng(), 6);
      dataFile.print(", ");
      dataFile.print(gps.altitude.meters());
      dataFile.print(", ");
      dataFile.print(gps.course.deg());
      dataFile.print(", ");
      dataFile.print(gps.speed.kmph());
      dataFile.print(", ");
      dataFile.print(distanceKmToSTART);
      dataFile.print(", ");
      dataFile.print(courseToSTART);
      dataFile.print(", ");
      dataFile.print(event.orientation.x, 4);
      dataFile.print(", ");
      dataFile.print(event.orientation.y, 4);
      dataFile.print(", ");
      dataFile.print(event.orientation.z, 4);
      dataFile.println(", ");

      dataFile.close();
    } else {
      Serial.println("Error opening datalog.txt");
    }
  } else {
    // Если данные GPS не получены, устанавливаем руль в нейтральное положение
    rudderAngle = 90;
    Serial.println("No GPS data received");
  }
}

// Функция для управления сервоприводом
void controlServo() {
  sensors_event_t event;
  bno.getEvent(&event);

  // Управление сервоприводами на основе данных с датчика
  posy = event.orientation.y; 
  posz = event.orientation.z;

  servo1.write(90 + posy - posz);
  servo2.write(90 - posy - posz);

  servo3.write(rudderAngle);  // Поворачиваем руль на заданный угол

  //Serial.print(90 + posy - posz);
  //Serial.print(", ");
  //Serial.print(90 - posy - posz);
  //Serial.print(", ");
  //Serial.print(rudderAngle);
  //Serial.println(", ");

}

// Функция для отключения сервоприводов
void detachServos() {
  servo1.detach();
  servo2.detach();
  servo3.detach();
}

// Функция для подключения сервоприводов
void attachServos() {
  servo1.attach(7);
  servo2.attach(6);
  servo3.attach(8);
}