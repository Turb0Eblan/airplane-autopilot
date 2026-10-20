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
int rudderAngle = 90;

#define BNO055_SAMPLERATE_DELAY_MS (100)

TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29, &Wire);
Servo servo1; 
Servo servo2; 
Servo servo3; 

void setup()
{
  servo1.attach(7);
  servo2.attach(6);
  servo3.attach(8);
  pinMode(LED_BUILTIN, OUTPUT);
  
  Serial.begin(115200);
  ss.begin(GPSBaud);

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

void loop(){

  digitalWrite(LED_BUILTIN, HIGH);
  
  
  String dataString = "";

  File dataFile = SD.open("datalog.txt", FILE_WRITE);

  sensors_event_t event;
  bno.getEvent(&event);

  static const double START_LAT = 43.23728, START_LON = 76.93131;

//---------------Вывод в Ардуино--------------------------------------------------------------------------------------------------------------------
 
  //Спутники
  Serial.print(gps.satellites.value());          //- информация о спутниках
  Serial.print(", ");
  //Точность по горизонту
  Serial.print(gps.hdop.hdop());                  // - Точность
  Serial.print(", ");
  //Широта и долгота
  Serial.print(gps.location.lat(), 6);
  Serial.print(", ");
  Serial.print(gps.location.lng(), 6);
  Serial.print(", ");
  //Высота над уровнем моря
  Serial.print(gps.altitude.meters());
  Serial.print(", ");
  //Направление движения (компас)
  Serial.print(gps.course.deg());
  Serial.print(", ");
  //Скорость в кмч
  Serial.print(gps.speed.kmph());
  Serial.print(", ");
  
  //расстояние до старта
  unsigned long distanceKmToSTART = (unsigned long)TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), START_LAT,  START_LON);
  Serial.print(distanceKmToSTART);
  Serial.print(", ");

  //курс к старту
  double courseToSTART = TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), START_LAT, START_LON);
  Serial.print(courseToSTART);
  Serial.print(", ");

  //Направление к старту
  const char *cardinalToSTART = TinyGPSPlus::cardinal(courseToSTART);
  Serial.print(cardinalToSTART);
  Serial.print(", ");

  //Направления (x)
  Serial.print(event.orientation.x, 4);
  Serial.print(", ");
  //y and z
  Serial.print(event.orientation.y, 4);
  Serial.print(" , ");
  Serial.print(event.orientation.z, 4);
  Serial.println(", ");

//-----------------------------------------------------------------------------------------------------------------------------------

  rudderAngle = map(courseToSTART, 0, 360, 45, 135);  // Преобразуем курс в угол сервопривода
  rudderAngle = constrain(rudderAngle, 45, 135);  // Ограничиваем угол в пределах 45-135 градусов

  posy = event.orientation.y; 
  posz = event.orientation.z;

  servo1.write(90+posy);
  servo2.write(90-posy);
  servo3.write(rudderAngle);

//-----------------------------------------------------------------------------------------------------------------------------------
  smartDelay(1000);

  digitalWrite(LED_BUILTIN, LOW);

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