// Прошивка для STM32F103C8 Blue Pill (на основе STM32Plane)
//
// Подключение:
//   GPS          — USART2: PA3 (RX, к TX модуля), PA2 (TX)
//   Телеметрия   — USART3: PB11 (RX), PB10 (TX) — строка данных «в Ардуино»
//   Отладка      — Serial (USART1): PA9 (TX), PA10 (RX)
//   BNO055       — I2C1: PB7 (SDA), PB6 (SCL)
//   SD-карта     — SPI1: PA5 (SCK), PA6 (MISO), PA7 (MOSI), PA4 (CS)
//   Сервы        — PB13 (левый элерон), PB14 (правый элерон), PB15 (руль)
//
// Отличия от исходного STM32Plane:
//   - GPS и телеметрия раньше были на одном USART2 (PA3/PA2) — телеметрия
//     перенесена на USART3.
//   - Объявлены rudderAngle/posz (без них скетч не собирался).
//   - Руль считается PID по ошибке курса, как в PilotExtended
//     (раньше map(courseToSTART, 0, 360, ...) не учитывал текущий курс).
//   - Убраны while(!Serial) и зависание при отсутствии SD — без USB/SD
//     самолёт раньше просто не стартовал.
//   - Задачи разнесены по таймерам вместо smartDelay(100) в конце loop().

#include <Arduino.h>
#include <TinyGPS++.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Servo.h>

static const uint32_t GPSBaud = 9600;
const uint32_t chipSelect = PA4;

// Начальные координаты (точка старта)
static const double START_LAT = 43.23728, START_LON = 76.93131;

const unsigned long GPS_TIMEOUT = 3000;          // Нет свежих данных GPS дольше — руль в нейтраль
const unsigned long SERVO_UPDATE_INTERVAL = 20;  // Обновление сервоприводов каждые 20 мс
const unsigned long TELEMETRY_INTERVAL = 100;    // Вывод телеметрии каждые 100 мс
const unsigned long LOG_INTERVAL = 1000;         // Запись на SD каждую секунду
const int RUDDER_NEUTRAL = 90;
const int RUDDER_LIMIT = 30;
const float PID_I_LIMIT = 1000;

TinyGPSPlus gps;
// В ядре STM32duino 4.x класс называется Uart (в старых версиях — HardwareSerial)
Uart ss(USART2);         // PA3 (RX), PA2 (TX)
Uart telemetry(USART3);  // PB11 (RX), PB10 (TX)
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29, &Wire);
Servo servo1;
Servo servo2;
Servo servo3;

unsigned long lastGpsFixTime = 0;
unsigned long lastServoUpdateTime = 0;
unsigned long lastTelemetryTime = 0;
unsigned long lastLogTime = 0;
int posy = 0;
int posz = 0;
int rudderAngle = RUDDER_NEUTRAL;
bool gpsValid = false;
bool sdReady = false;

// PID-регулятор для руля направления
float Kp = 0.5, Ki = 0.01, Kd = 0.1;
float pidErrorSum = 0, lastPidError = 0;

void readGPSData();
void calculateRudderAngle();
void controlServos();
void sendTelemetry();
void logData();

void setup()
{
  servo1.attach(PB13);
  servo2.attach(PB14);
  servo3.attach(PB15);
  servo1.write(90);
  servo2.write(90);
  servo3.write(RUDDER_NEUTRAL);

  Serial.begin(115200);
  telemetry.begin(115200);
  ss.begin(GPSBaud);

  if (!bno.begin())
  {
    Serial.print("No BNO055 detected");
    telemetry.print("No BNO055 detected");
    while (1);
  }
  bno.setExtCrystalUse(true);

  Serial.print("Initializing SD card...");
  telemetry.print("Initializing SD card...");
  sdReady = SD.begin(chipSelect);
  // Продолжаем работу и без SD-карты
  Serial.println(sdReady ? "initialization done." : "initialization failed");
  telemetry.println(sdReady ? "initialization done." : "initialization failed");
}

void loop()
{
  unsigned long currentTime = millis();

  readGPSData();

  if (currentTime - lastServoUpdateTime >= SERVO_UPDATE_INTERVAL) {
    lastServoUpdateTime = currentTime;
    controlServos();
  }

  if (currentTime - lastTelemetryTime >= TELEMETRY_INTERVAL) {
    lastTelemetryTime = currentTime;
    sendTelemetry();
  }

  if (currentTime - lastLogTime >= LOG_INTERVAL) {
    lastLogTime = currentTime;
    logData();
  }

  if (currentTime > 5000 && gps.charsProcessed() < 10) {
    static bool warned = false;
    if (!warned) Serial.println(F("Данные GPS не получены: проверьте соединение"));
    warned = true;
  }
}

void readGPSData()
{
  while (ss.available())
    gps.encode(ss.read());

  if (gps.location.isUpdated()) {
    gpsValid = gps.location.isValid() && gps.course.isValid();
    if (gpsValid) {
      lastGpsFixTime = millis();
      calculateRudderAngle();
    }
  }

  if (gpsValid && millis() - lastGpsFixTime > GPS_TIMEOUT) {
    gpsValid = false;
  }
  if (!gpsValid) {
    rudderAngle = RUDDER_NEUTRAL;
    pidErrorSum = 0;
    lastPidError = 0;
  }
}

void calculateRudderAngle()
{
  float targetCourse = TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), START_LAT, START_LON);
  float error = targetCourse - gps.course.deg();

  // Нормализуем ошибку в диапазон [-180, 180]
  while (error > 180) error -= 360;
  while (error < -180) error += 360;

  pidErrorSum = constrain(pidErrorSum + error, -PID_I_LIMIT, PID_I_LIMIT);
  float dError = error - lastPidError;
  lastPidError = error;

  float pidOutput = Kp * error + Ki * pidErrorSum + Kd * dError;
  rudderAngle = RUDDER_NEUTRAL + constrain(pidOutput, -RUDDER_LIMIT, RUDDER_LIMIT);
}

void controlServos()
{
  sensors_event_t event;
  bno.getEvent(&event);

  posy = event.orientation.y;
  posz = event.orientation.z;

  servo1.write(constrain(90 + posy, 45, 135));
  servo2.write(constrain(90 - posy, 45, 135));
  servo3.write(rudderAngle);
}

//---------------Вывод в Ардуино--------------------------------------------------------------------------------------------------------------------
void sendTelemetry()
{
  sensors_event_t event;
  bno.getEvent(&event);

  unsigned long distanceKmToSTART = (unsigned long)TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), START_LAT, START_LON);
  double courseToSTART = TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), START_LAT, START_LON);

  //Спутники
  telemetry.print(gps.satellites.value());
  telemetry.print(", ");
  //Точность по горизонту
  telemetry.print(gps.hdop.hdop());
  telemetry.print(", ");
  //Широта и долгота
  telemetry.print(gps.location.lat(), 6);
  telemetry.print(", ");
  telemetry.print(gps.location.lng(), 6);
  telemetry.print(", ");
  //Высота над уровнем моря
  telemetry.print(gps.altitude.meters());
  telemetry.print(", ");
  //Направление движения (компас)
  telemetry.print(gps.course.deg());
  telemetry.print(", ");
  //Скорость в кмч
  telemetry.print(gps.speed.kmph());
  telemetry.print(", ");
  //расстояние до старта
  telemetry.print(distanceKmToSTART);
  telemetry.print(", ");
  //курс к старту
  telemetry.print(courseToSTART);
  telemetry.print(", ");
  //Направление к старту
  telemetry.print(TinyGPSPlus::cardinal(courseToSTART));
  telemetry.print(", ");
  //Направления x, y, z
  telemetry.print(event.orientation.x, 4);
  telemetry.print(", ");
  telemetry.print(event.orientation.y, 4);
  telemetry.print(", ");
  telemetry.print(event.orientation.z, 4);
  telemetry.print(", ");
  //Руль
  telemetry.println(rudderAngle);
}

void logData()
{
  if (!sdReady) return;

  File dataFile = SD.open("datalog.txt", FILE_WRITE);
  if (!dataFile) {
    Serial.println("error opening datalog.txt");
    return;
  }

  sensors_event_t event;
  bno.getEvent(&event);

  unsigned long distanceKmToSTART = (unsigned long)TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), START_LAT, START_LON);
  double courseToSTART = TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), START_LAT, START_LON);

  dataFile.print(millis());
  dataFile.print(", ");
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
  dataFile.print(", ");
  dataFile.println(rudderAngle);

  dataFile.close();
}
