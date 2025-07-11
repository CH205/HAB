#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <SD.h>
const int solenoidPin = 7;//solonoid inclusion
const int rttyPin = 9;//solonoid inclusion
// Temporarily lowered for testing indoors (~48 m)//
const float ALTITUDE_ON_THRESHOLD = 35.0;//solonoid inclusion
const float ALTITUDE_OFF_THRESHOLD = 20.0;//solonoid inclusion
bool solenoidActive = false;///solonoid inclusion

// === BMP180 ===
Adafruit_BMP085 bmp;
float seaLevelPressure = 1013.25; // QNH in hPa
bool bmpOK = false;

// === DS18B20 ===
#define ONE_WIRE_BUS 4
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
float externalTemp = NAN;

// === GPS ===
static const int RXPin = 8, TXPin = 7;
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);

// === SD Card ===
const int chipSelect = 10;
File dataFile;

// === RTTY ===
const int RTTY_PIN = 9;
const int LED_PIN = 13;

// === Timing ===
unsigned long lastLogTime = 0;
unsigned long startTime = 0;
const unsigned long LOG_INTERVAL = 1000; // ms

void setup() {
  Serial.begin(115200);
  ss.begin(GPSBaud);
  sensors.begin();
  pinMode(LED_PIN, OUTPUT);
  pinMode(RTTY_PIN, OUTPUT);
  digitalWrite(RTTY_PIN, HIGH);  // idle high for RTTY

  // BMP180
  if (bmp.begin()) {
    bmpOK = true;
    Serial.println(F("✅ BMP180 init OK"));
  } else {
    bmpOK = false;
    Serial.println(F("❌ BMP180 not detected"));
  }
pinMode(solenoidPin, OUTPUT);//solenoid
  digitalWrite(solenoidPin, LOW);//
  pinMode(rttyPin, OUTPUT);//
  digitalWrite(rttyPin, HIGH);//
  
  // SD Card
  if (!SD.begin(chipSelect)) {
    Serial.println(F("❌ SD init failed"));
    return;
  }

  dataFile = SD.open("log.csv", FILE_WRITE);
  if (dataFile) {
    dataFile.println(F("Time(s),Temp(°C),Press(hPa),Alt(m),Lat,Lon,Sats,ExtTemp"));
    dataFile.close();
  }

  startTime = millis();
}

void loop() {
  while (ss.available()) gps.encode(ss.read());
  unsigned long currentMillis = millis();

  if (currentMillis - lastLogTime >= LOG_INTERVAL) {
    lastLogTime = currentMillis;

    float temp = NAN, pressure = NAN, altitude = NAN;
    if (bmpOK) {
      temp = bmp.readTemperature();
      pressure = bmp.readPressure() / 100.0F;
      if (!isnan(temp) && !isnan(pressure)) {
        altitude = 44330.0 * (1.0 - pow(pressure / seaLevelPressure, 0.1903));
      } else {
        temp = pressure = altitude = NAN;
        Serial.println(F("⚠️ Invalid BMP180 read — skipping."));
      }
    }

    sensors.requestTemperatures();
    externalTemp = sensors.getTempCByIndex(0);
    if (externalTemp <= -100.0) externalTemp = NAN;

    float lat = gps.location.isValid() ? gps.location.lat() : NAN;
    float lon = gps.location.isValid() ? gps.location.lng() : NAN;
    int sats = gps.satellites.isValid() ? gps.satellites.value() : 0;
    unsigned long elapsed = (currentMillis - startTime) / 1000;

    // === Serial Output ===
    Serial.print(F("T+")); Serial.print(elapsed); Serial.print(F("s | "));
    Serial.print(F("Temp: ")); isnan(temp) ? Serial.print(F("NA")) : Serial.print(temp, 2);
    Serial.print(F(" °C | Press: ")); isnan(pressure) ? Serial.print(F("NA")) : Serial.print(pressure, 2);
    Serial.print(F(" hPa | Alt: ")); isnan(altitude) ? Serial.print(F("NA")) : Serial.print(altitude, 2);
    Serial.print(F(" m | GPS: "));
    if (!isnan(lat) && !isnan(lon)) {
      Serial.print(lat, 6); Serial.print(F(", ")); Serial.print(lon, 6);
    } else {
      Serial.print(F("NA, NA"));
    }
    Serial.print(F(" | Sats: ")); Serial.print(sats);
    Serial.print(F(" | Ext Temp: ")); isnan(externalTemp) ? Serial.println(F("NA")) : Serial.println(externalTemp, 2);

    // === SD Logging ===
    dataFile = SD.open("log.csv", FILE_WRITE);
    if (dataFile) {
      dataFile.print(elapsed); dataFile.print(",");
      dataFile.print(isnan(temp) ? "NA" : String(temp, 2)); dataFile.print(",");
      dataFile.print(isnan(pressure) ? "NA" : String(pressure, 2)); dataFile.print(",");
      dataFile.print(isnan(altitude) ? "NA" : String(altitude, 2)); dataFile.print(",");
      dataFile.print(isnan(lat) ? "NA" : String(lat, 6)); dataFile.print(",");
      dataFile.print(isnan(lon) ? "NA" : String(lon, 6)); dataFile.print(",");
      dataFile.print(sats); dataFile.print(",");
      dataFile.println(isnan(externalTemp) ? "NA" : String(externalTemp, 2));
      dataFile.close();
    }

    // === RTTY every 5 sec ===
    if (elapsed % 5 == 0) {
      digitalWrite(LED_PIN, HIGH);
      rttyTransmit(elapsed, temp, pressure, altitude, lat, lon, sats, externalTemp);
      digitalWrite(LED_PIN, LOW);
      Serial.println(F("✅ RTTY complete"));
    }
  }
}

// === RTTY Transmit ===
void rttyTransmit(unsigned long t, float temp, float press, float alt, float lat, float lon, int sats, float extTemp) {
  String sentence = "T+" + String(t) + "s,";
  sentence += (isnan(temp) ? "NA" : String(temp, 1)) + ",";
  sentence += (isnan(press) ? "NA" : String(press, 1)) + ",";
  sentence += (isnan(alt) ? "NA" : String(alt, 1)) + ",";
  sentence += (isnan(lat) ? "NA" : String(lat, 5)) + ",";
  sentence += (isnan(lon) ? "NA" : String(lon, 5)) + ",";
  sentence += String(sats) + ",";
  sentence += (isnan(extTemp) ? "NA" : String(extTemp, 1));

  for (unsigned int i = 0; i < sentence.length(); i++) {
    rttyTxByte(sentence[i]);
  }
}

void rttyTxByte(byte b) {
  const unsigned int baudDelay = 1000000 / 50; // 50 baud
  digitalWrite(RTTY_PIN, LOW); delayMicroseconds(baudDelay); // Start bit
  for (byte mask = 1; mask; mask <<= 1) {
    digitalWrite(RTTY_PIN, (b & mask) ? HIGH : LOW);
    delayMicroseconds(baudDelay);
  }
  digitalWrite(RTTY_PIN, HIGH); delayMicroseconds(baudDelay); // Stop bit
}
