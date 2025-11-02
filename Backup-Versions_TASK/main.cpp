// ----------------------------------------------------------------------------
// CanSat Project 2025
// CanSat - BMP280 sensor reading and publishing via MQTT    (no error handling for MQTT)
// CanSat - Data logging to SD card with versioned filenames (no error handling for MQTT)
//        -> (data gets send to MQTT without errors when sd is not available, but slower)
//        -> (data printed to serial when sd is not available, but with error code)
//        -> (uploading code sometimes causes to create 2 (or more) new files, resetting ESP32 causes to create 1 new file (=good))
// ----------------------------------------------------------------------------

// Libraries used
#include <Wire.h>                // I2C
#include <Adafruit_Sensor.h>     // Adafruit unified sensor base
#include <Adafruit_BMP280.h>     // BMP280 
#include <WiFi.h>                // ESP32 WiFi
#include <ArduinoMqttClient.h>   // MQTT client
#include <SD.h>                  // SD card 
#include <SPI.h>                 // SPI bus
#include "Secret.h"              
#include <Arduino.h>             

// Declarations
void printValues();                                  // Create + publish + log CSV line
bool I2C_check(TwoWire *bus, byte address);          // I2C device presence check
void writeToSD(String data);                         // Append one CSV line to current file

// --- New helper functions ---
int  readLastVersion();                              // Read last used version from VERSION_FILE (create with 0 if missing)
void saveLastVersion(int newLast);                   // Store last used version back to VERSION_FILE

#define MSG_LEN 120

// Set the value of the sea level pressure correct at your location
// You can find it at https://www.meteo.be/nl/weer/waarnemingen/belgie
// If you do so, the height will be calculated approx. correctly.
// + or - 8 m height difference is normal, as the sensor has a deviation.
#define SEALEVELPRESSURE_HPA (1015.8)

// BMP280 I2C address (some boards use 0x77)
#define BMP280_ADR 0x76

// Non-default pins for I2C connection
#define SDA_2 32    // Use this pin as secondary I2C SDA connection
#define SCL_2 33    // Use this pin as secondary I2C SCL connection

#define DELAY_TIME 1000 // Delay time between measurements (ms)

// SD Card SPI pins 
#define SD_CS_PIN   25    // CS        = GPIO25
#define SD_MOSI_PIN 23    // MOSI (DI) = GPIO23
#define SD_MISO_PIN 19    // MISO (DO) = GPIO19
#define SD_SCK_PIN  18    // SCK (CLK) = GPIO18

// Filenames / paths on SD
#define DATA_FILE_BASE "/CanSatSend"        // Base name for data files
#define VERSION_FILE   "/CanSatVersion.txt" // Stores last used version number

// Create secondary I2C bus object on ESP32
TwoWire I2Ctwo = TwoWire(1);

// Bind BMP280 to the secondary I2C bus
Adafruit_BMP280 bmp(&I2Ctwo);

bool bmp_connected = false;    // Tracks BMP280 availability
bool sd_available = false;     // Tracks SD availability
int currentVersion;            // version number (saved on SD)
String currentFilename = "";   // Active data filename for this session

// WiFi + MQTT clients
WiFiClient  wifiClient;
MqttClient  mqttClient(wifiClient);

void setup() {

  unsigned bmp_status; // Holds BMP initialization result

  int WIFI_status = WL_IDLE_STATUS;
  char msg[MSG_LEN] = "";

  // Start serial monitor 
  Serial.begin(115200);
  while (!Serial) { // Wait for serial port to connect
    delay(10); 
  }  

  // SD card 
  Serial.println("----------------------------------------------------------------------");
  Serial.println("\nInitialiseren SD kaart...");

  // Initialize the SPI bus with explicit pins
  SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
  delay(100);

  // Initialize SD card
  if (!SD.begin(SD_CS_PIN, SPI)) {
    Serial.println("SD kaart initialisatie mislukt!");
    sd_available = false; // Continue without logging
  } else {
    Serial.println("SD kaart succesvol geïnitialiseerd");

    // Optional: print SD card info (size/type)
    uint64_t cardSize = SD.cardSize() / (1024ULL * 1024ULL);
    Serial.print("SD kaart grootte: ");
    Serial.print(cardSize);
    Serial.println(" MB");

    uint8_t cardType = SD.cardType();
    Serial.print("SD kaart type: ");
    if (cardType == CARD_MMC) {
      Serial.println("MMC");
    } else if (cardType == CARD_SD) {
      Serial.println("SDSC");
    } else if (cardType == CARD_SDHC) {
      Serial.println("SDHC");
    } else {
      Serial.println("UNKNOWN");
    }

    sd_available = true;

    int lastUsed = readLastVersion();

    // Current version = last used + 1
    currentVersion = lastUsed + 1;

    // Build filename for this session
    currentFilename = String(DATA_FILE_BASE) + String(currentVersion) + ".txt";
    Serial.print("Data wordt weggeschreven naar: ");
    Serial.println(currentFilename);

    saveLastVersion(currentVersion);
  }

  // WiFi connection
  Serial.println("\n----------------------------------------------------------------------\n");
  Serial.print("Verbinden met WiFi... ");
  WiFi.begin(SECRET_SSID, SECRET_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000); 
    Serial.print(".");
  }

  // Confirmation message + local IP
  sprintf(msg, "\nWiFi Connected, IP = %s\n", WiFi.localIP().toString().c_str());
  Serial.println(msg);

  // MQTT broker connection
  Serial.println("----------------------------------------------------------------------\n");
  sprintf(msg, "Tracht te verbinden met MQTT broker op hostadres: %s en topic %s\n", BROKER, TOPIC);
  Serial.println(msg);

  // Attempt to connect to broker (loops until successful)
  while (!mqttClient.connect(BROKER, PORT)) {
    Serial.print("MQTT connectie mislukt! Error code = ");
    Serial.println(mqttClient.connectError());
    delay(1000);
  }
  Serial.println("MQTT succesvol verbonden!");
  Serial.println("\n----------------------------------------------------------------------\n");

  // Bring up secondary I2C bus on custom pins
  I2Ctwo.begin(SDA_2, SCL_2);  // SDA=32, SCL=33

  // Initialize BMP280 at given address
  bmp_status = bmp.begin(BMP280_ADR);
  if (!bmp_status) {
    Serial.println("Could not find a valid BMP280 sensor, check wiring, address, sensor ID!");
    return;
  }

  bmp_connected = true;  // Sensor found

  // Optional: configure oversampling/filter to improve stability/noise
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* IIR Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  Serial.println("-- Default Test --");
  Serial.println("CanSat => BMP-280 successful connected (I2C-2)");
  Serial.println("\n----------------------------------------------------------------------\n");
}

void loop() {
  // Re-check BMP280 each loop
  bmp_connected = I2C_check(&I2Ctwo, BMP280_ADR);

  if (bmp_connected) {
    printValues(); // Read, publish, and log one CSV line
  } else {
    Serial.println("ERROR: BMP280 sensor not found!");
  }

  // Keep the MQTT connection alive
  mqttClient.poll();

  delay(DELAY_TIME);
}

// printValues(): read sensors, publish via MQTT, log to SD
void printValues() {
  // Extra safety check, if no sensor, don't continue
  if (!bmp_connected) return;

  // Read BMP280 (temperature in °C, pressure in hPa, altitude in m)
  float temperature = bmp.readTemperature();                  // °C
  float pressure    = bmp.readPressure() / 100.0F;            // hPa
  float altitude    = bmp.readAltitude(SEALEVELPRESSURE_HPA); // m

  // Temporary GPS placeholders (replace later with real GPS module data) (added to code for CSV completeness for testing)
  int   sats = 3;
  int   hdop = 3;
  float lat  = 55.9737;
  float lon  = 4.6376;

  // Build CSV payload (semicolon-separated)
  String payload = String(temperature, 2) + ";" +
                   String(pressure, 2) + ";" +
                   String(altitude, 2) + ";" +
                   String(sats) + ";" +
                   String(hdop)  + ";" +
                   String(lat, 4) + ";" +
                   String(lon, 4);

  // Print to serial for live debugging and for Node.js server data streaming
  Serial.println(payload);

  // Publish payload to MQTT topic
  mqttClient.beginMessage(TOPIC);
  mqttClient.print(payload);
  mqttClient.endMessage();

  // Append same line to SD card if available
  if (sd_available) {
    writeToSD(payload);
  }
}

// writeToSD(): append one CSV line to currentFilename
void writeToSD(String data) {
  File dataFile = SD.open(currentFilename, FILE_APPEND);
  if (dataFile) {
    dataFile.println(data);
    dataFile.close();
  } else {
    Serial.println("Fout: kon niet schrijven naar SD kaart");
  }
}

// I2C_check(): returns true if device at 'address' ACKs on given bus
bool I2C_check(TwoWire *bus, byte address) {
  bus->beginTransmission(address);
  byte error = bus->endTransmission();
  return (error == 0);
}

// readLastVersion(): read last used version (create with "0" if missing)
int readLastVersion() {
  // The file stores the LAST used version number.
  // Doesn't exist? Create it with "0"
  int lastUsed = 0;

  File versionFileRead = SD.open(VERSION_FILE, FILE_READ);
  if (!versionFileRead) {
    File versionFileCreate = SD.open(VERSION_FILE, FILE_WRITE);
    if (versionFileCreate) {
      versionFileCreate.println("0");
      versionFileCreate.close();
    }
    lastUsed = 0;
    Serial.println("Versie bestand aangemaakt met laatste versie 0");
  } else {
    String versionStr = versionFileRead.readStringUntil('\n');
    versionFileRead.close();
    int v = versionStr.toInt();
    if (v >= 0) lastUsed = v;
    else lastUsed = 0;
  }
  return lastUsed;
}

// saveLastVersion(): store last used version number
void saveLastVersion(int newLast) {
  // Sla de HUIDIGE versie terug op als "laatst gebruikt"
  File versionFileWrite = SD.open(VERSION_FILE, FILE_WRITE);
  if (versionFileWrite) {
    versionFileWrite.seek(0);          // Go to start of file
    versionFileWrite.println(newLast); // last used = current
    versionFileWrite.close();
  } else {
    Serial.println("Fout: kon versienummer niet bijwerken");
  }
}