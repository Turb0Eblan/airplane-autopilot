#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Servo.h>

static const int RXPin = PA3, TXPin = PA2;
static const uint32_t GPSBaud = 9600;
const int chipSelect = 4;
int posy = 0; 

#define BNO055_SAMPLERATE_DELAY_MS (100)

TinyGPSPlus gps;
HardwareSerial Serial2(USART2);
HardwareSerial ss(RXPin, TXPin);
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29, &Wire);
Servo servo1; 
Servo servo2; 
Servo servo3; 

void setup()
{
  servo1.attach(PB13);
  servo2.attach(PB14);
  servo3.attach(PB15);
  
  Serial.begin(115200);
  Serial2.begin(115200);
  ss.begin(GPSBaud);

  while (!Serial);

  if (!bno.begin())
  {
    Serial.print("No BNO055 detected");
    Serial2.print("No BNO055 detected");
    while (1);
  }

  Serial.print("Initializing SD card...");
  Serial2.print("Initializing SD card...");
  if (!SD.begin(chipSelect)) {
    Serial.println("initialization failed");
    Serial2.println("initialization failed");
    while (true);
  }
  Serial.println("initialization done.");
  Serial2.println("initialization done.");
  
  bno.setExtCrystalUse(true);
  
}

void loop(){
  
  String dataString = "";

  File dataFile = SD.open("datalog.txt", FILE_WRITE);

  sensors_event_t event;
  bno.getEvent(&event);

  static const double START_LAT = 43.23728, START_LON = 76.93131;

//---------------Вывод в Ардуино--------------------------------------------------------------------------------------------------------------------
 
  //Спутники
  Serial2.print(gps.satellites.value());          //- информация о спутниках
  Serial2.print(", ");
  //Точность по горизонту
  Serial2.print(gps.hdop.hdop());                  // - Точность
  Serial2.print(", ");
  //Широта и долгота
  Serial2.print(gps.location.lat(), 6);
  Serial2.print(", ");
  Serial2.print(gps.location.lng(), 6);
  Serial2.print(", ");
  //Высота над уровнем моря
  Serial2.print(gps.altitude.meters());
  Serial2.print(", ");
  //Направление движения (компас)
  Serial2.print(gps.course.deg());
  Serial2.print(", ");
  //Скорость в кмч
  Serial2.print(gps.speed.kmph());
  Serial2.print(", ");
  
  //расстояние до старта
  unsigned long distanceKmToSTART = (unsigned long)TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), START_LAT,  START_LON);
  Serial2.print(distanceKmToSTART);
  Serial2.print(", ");

  //курс к старту
  double courseToSTART = TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), START_LAT, START_LON);
  Serial2.print(courseToSTART);
  Serial2.print(", ");

  //Направление к старту
  const char *cardinalToSTART = TinyGPSPlus::cardinal(courseToSTART);
  Serial2.print(cardinalToSTART);
  Serial2.print(", ");

  //Направления (x)
  Serial2.print(event.orientation.x, 4);
  Serial2.print(", ");
  //y and z
  Serial2.print(event.orientation.y, 4);
  Serial2.print(" , ");
  Serial2.print(event.orientation.z, 4);
  Serial2.println(", ");

//-----------------------------------------------------------------------------------------------------------------------------------

  rudderAngle = map(courseToSTART, 0, 360, 45, 135);  // Преобразуем курс в угол сервопривода
  rudderAngle = constrain(rudderAngle, 45, 135);  // Ограничиваем угол в пределах 45-135 градусов

  posy = event.orientation.y; 
  posz = event.orientation.z;

  servo1.write(90+posy);
  servo2.write(90-posy);
  servo3.write(rudderAngle);

//-----------------------------------------------------------------------------------------------------------------------------------
  smartDelay(100);

  if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.println(F("Данные GPS не получены: проверьте соединение"));

  if (dataFile) {
    dataFile.print(gps.location.lat(), 6);
    dataFile.print(", ");
    dataFile.print(gps.location.lng(), 6);
    dataFile.println(", ");
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
    dataFile.print(" , ");
    dataFile.print(event.orientation.z, 4);
    dataFile.println(", ");

    dataFile.close();
  }
  else {
    Serial.println("error opening datalog.txt");
  }

}


static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}