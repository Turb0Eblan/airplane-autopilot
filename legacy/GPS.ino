#include <TinyGPS++.h>
#include <SoftwareSerial.h>

static const int RXPin = 3, TXPin = 2;
static const uint32_t GPSBaud = 9600;

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

void setup()
{
  Serial.begin(115200);
  ss.begin(GPSBaud);
}

void loop()
{
  static const double MOSCOW_LAT = 56.337582, MOSCOW_LON = 38.370631;
Serial.print("Спутников - \t\t\t\t");
printInt(gps.satellites.value(), gps.satellites.isValid(), 5);          //- информация о спутниках
Serial.println();
Serial.print("Точность по горизон. - \t\t\t");
printFloat(gps.hdop.hdop(), gps.hdop.isValid(), 6, 1);                  // - Точность
Serial.println();
Serial.print("Широта и долгота - \t\t\t");
  printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
  Serial.print(" / ");
  printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);
Serial.println();
Serial.print("Дата и время - \t\t\t\t");
printDateTime(gps.date, gps.time);
Serial.println();
Serial.print("Высота над уровнем моря - \t\t");
printFloat(gps.altitude.meters(), gps.altitude.isValid(), 7, 2);
Serial.println();
Serial.print("Направление движения (компас) - \t");
printFloat(gps.course.deg(), gps.course.isValid(), 7, 2);
Serial.println();
Serial.print("Скорость в км. - \t\t\t");
printFloat(gps.speed.kmph(), gps.speed.isValid(), 6, 2);
Serial.println();
Serial.print("Направление - \t\t\t\t");
  printStr(gps.course.isValid() ? TinyGPSPlus::cardinal(gps.course.deg()) : "*** ", 6);
Serial.println();
Serial.print("расстояние до дачи - \t\t\t");
  unsigned long distanceKmToMoscow =
    (unsigned long)TinyGPSPlus::distanceBetween(
      gps.location.lat(),
      gps.location.lng(),
      MOSCOW_LAT, 
      MOSCOW_LON) / 1000;
  printInt(distanceKmToMoscow, gps.location.isValid(), 9);

  double courseToMoscow =
    TinyGPSPlus::courseTo(
      gps.location.lat(),
      gps.location.lng(),
      MOSCOW_LAT, 
      MOSCOW_LON);

  printFloat(courseToMoscow, gps.location.isValid(), 7, 2);

Serial.println();
Serial.print("Направление к даче - \t\t\t");  

  const char *cardinalToMoscow = TinyGPSPlus::cardinal(courseToMoscow);

  printStr(gps.location.isValid() ? cardinalToMoscow : "*** ", 6);
Serial.println();
//  printInt(gps.charsProcessed(), true, 6);
//  printInt(gps.sentencesWithFix(), true, 10);
//  printInt(gps.failedChecksum(), true, 9);
  Serial.println("------------------------------------------------------------------");
  
  smartDelay(10000);                                                      // Пауза для вывода

  if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.println(F("Данные GPS не получены: проверьте соединение"));
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

static void printDateTime(TinyGPSDate &d, TinyGPSTime &t)
{
  if (!d.isValid())
  {
    Serial.print(F("********** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
    Serial.print(sz);
  }
  
  if (!t.isValid())
  {
    Serial.print(F("******** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
    Serial.print(sz);
  }

  printInt(d.age(), d.isValid(), 5);
  smartDelay(0);
}

static void printStr(const char *str, int len)
{
  int slen = strlen(str);
  for (int i=0; i<len; ++i)
    Serial.print(i<slen ? str[i] : ' ');
  smartDelay(0);
}
