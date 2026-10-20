#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <SD.h>

static const int RXPin = 3, TXPin = 2;
static const uint32_t GPSBaud = 9600;
const int chipSelect = 4;

TinyGPSPlus gps;

SoftwareSerial ss(RXPin, TXPin);

void setup()
{
  Serial.begin(115200);
  ss.begin(GPSBaud);

  while (!Serial);

  Serial.print("Initializing SD card...");
  if (!SD.begin(chipSelect)) {
    Serial.println("initialization failed");
    while (true);
  }
  Serial.println("initialization done.");
}

void loop(){

  String dataString = "";

  File dataFile = SD.open("datalog.txt", FILE_WRITE);
  
  static const double START_LAT = 56.337582, START_LON = 38.370631;

  //Спутники
  printInt(gps.satellites.value(), gps.satellites.isValid(), 5);          //- информация о спутниках
  Serial.print(", ");
  //Точность по горизонту
  printFloat(gps.hdop.hdop(), gps.hdop.isValid(), 6, 1);                  // - Точность
  Serial.print(", ");
  //Широта и долгота
  printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
  Serial.print(", ");
  printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);
  Serial.print(", ");
    //Высота над уровнем моря
  printFloat(gps.altitude.meters(), gps.altitude.isValid(), 7, 2);
  Serial.print(", ");
  //Направление движения (компас)
  printFloat(gps.course.deg(), gps.course.isValid(), 7, 2);
  Serial.print(", ");
  //Скорость в кмч
  printFloat(gps.speed.kmph(), gps.speed.isValid(), 6, 2);
  Serial.print(", ");
  //Направление
  printStr(gps.course.isValid() ? TinyGPSPlus::cardinal(gps.course.deg()) : "*** ", 6);
  Serial.print(", ");

  //расстояние до старта
  unsigned long distanceKmToSTART = (unsigned long)TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), START_LAT,  START_LON) / 1000;
  printInt(distanceKmToSTART, gps.location.isValid(), 9);

  double courseToSTART = TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), START_LAT, START_LON);
  printFloat(courseToSTART, gps.location.isValid(), 7, 2);
  Serial.print(", ");

  //Направление к старту
  const char *cardinalToSTART = TinyGPSPlus::cardinal(courseToSTART);
  printStr(gps.location.isValid() ? cardinalToSTART : "*** ", 6);
  Serial.println();
//  printInt(gps.charsProcessed(), true, 6);
//  printInt(gps.sentencesWithFix(), true, 10);
//  printInt(gps.failedChecksum(), true, 9);


//-----------------------------------------------------------------------------------------------------------------------------------
  
  smartDelay(1000); // Пауза для вывода

  if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.println(F("Данные GPS не получены: проверьте соединение"));

  if (dataFile) {
    dataFile.printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
    dataFile.print(", ");
    dataFile.printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);
    dataFile.print(", ");
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  }
}

// This custom version of delay() ensures that the gps object
// is being "fed".
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}

static void printFloat(float val, bool valid, int len, int prec)
{
  if (!valid)
  {
    while (len-- > 1)
      Serial.print('*');
    Serial.print(' ');
  }
  else
  {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i=flen; i<len; ++i)
      Serial.print(' ');
  }
  smartDelay(0);
}

static void printInt(unsigned long val, bool valid, int len)
{
  char sz[32] = "*****************";
  if (valid)
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if (len > 0) 
    sz[len-1] = ' ';
  Serial.print(sz);
  smartDelay(0);
}

static void printStr(const char *str, int len)
{
  int slen = strlen(str);
  for (int i=0; i<len; ++i)
    Serial.print(i<slen ? str[i] : ' ');
  smartDelay(0);
}
