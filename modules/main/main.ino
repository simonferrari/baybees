#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <SparkFun_ADXL345.h>
#include <DHT.h>
#include "HX711.h"

/* --- CONFIGURATION GÉNÉRALE --- */
#define SERIAL_BAUD    9600
#define PRINT_INTERVAL 3000 // Affichage toutes les 3 secondes

/* --- CONFIGURATION PINS --- */
#define PIN_DS18B20    4
#define PIN_DHT_INT    2    // DHT Intérieur
#define PIN_DHT_EXT    15   // DHT Extérieur
#define PIN_HX711_DOUT 32
#define PIN_HX711_SCK  33
#define I2C_SDA        21
#define I2C_SCL        22

/* --- CONFIGURATION CAPTEURS --- */
#define DHT_TYPE      DHT22
#define LUX_I2C_ADDR  0x23
float calibration_factor = -30.0;

/* --- INSTANCIATION --- */
OneWire           oneWire(PIN_DS18B20);
DallasTemperature sensors(&oneWire);
ADXL345           adxl = ADXL345();
DHT               dhtInt(PIN_DHT_INT, DHT_TYPE);
DHT               dhtExt(PIN_DHT_EXT, DHT_TYPE);
HX711             scale;

/* --- VARIABLES GLOBALES --- */
unsigned long lastPrintTime = 0;
int           dsCount       = 0;
uint8_t       luxBuf[2];

void setup() {
  // Initialisation DS18B20
  sensors.begin();
  dsCount = sensors.getDeviceCount();

  Serial.begin(SERIAL_BAUD);
  Wire.begin(I2C_SDA, I2C_SCL);

  // Initialisation ADXL345
  adxl.powerOn();
  adxl.setRangeSetting(16);

  // Initialisation DHT
  dhtInt.begin();
  dhtExt.begin();

  // Initialisation HX711
  scale.begin(PIN_HX711_DOUT, PIN_HX711_SCK);
  scale.set_scale(calibration_factor);
  scale.tare();

  Serial.println("Système Multi-Capteurs Prêt !");
}

void loop() {
  unsigned long currentMillis = millis();

  // Boucle d'affichage cadencée
  if (currentMillis - lastPrintTime >= PRINT_INTERVAL) {
    Serial.println("\n--- NOUVELLE LECTURE ---");
    
    lireDS18B20();
    lireADXL();
    lireLux();
    lireDHT();
    lirePoids();

    lastPrintTime = currentMillis;
  }
}

/* --- FONCTIONS DE LECTURE --- */

void lireDS18B20() {
  if (dsCount == 0) {
    sensors.begin();
    dsCount = sensors.getDeviceCount();
  }
  
  if (dsCount > 0) {
    sensors.requestTemperatures();
    for (int i = 0; i < dsCount; i++) {
      float t = sensors.getTempCByIndex(i);
      Serial.print("DS18B20 ["); 
      Serial.print(i); 
      Serial.print("]: ");
      
      if (t == DEVICE_DISCONNECTED_C) {
        Serial.println("ERREUR");
      } else {
        Serial.print(t, 2); 
        Serial.println(" C");
      }
    }
  } else {
    Serial.println("DS18B20 : Toujours aucune sonde detectee");
  }
}

void lireADXL() {
  int x, y, z;
  adxl.readAccel(&x, &y, &z);
  Serial.print("Accel XYZ: ");
  Serial.print(x); Serial.print(", ");
  Serial.print(y); Serial.print(", ");
  Serial.println(z);
}

void lireLux() {
  // Lecture simplifiée du SEN0562
  Wire.beginTransmission(LUX_I2C_ADDR);
  Wire.write(0x10);
  
  if (Wire.endTransmission() == 0) {
    delay(20); // Petit délai interne requis par le capteur
    Wire.requestFrom(LUX_I2C_ADDR, 2);
    if (Wire.available() == 2) {
      uint16_t data = Wire.read() << 8 | Wire.read();
      float lux = data / 1.2;
      Serial.print("Luminosité: "); 
      Serial.print(lux); 
      Serial.println(" lx");
    }
  }
}

void lireDHT() {
  // Lecture DHT Intérieur
  float hInt = dhtInt.readHumidity();
  float tInt = dhtInt.readTemperature();
  Serial.print("DHT22 INT: ");
  if (!isnan(hInt) && !isnan(tInt)) {
    Serial.print("Hum "); Serial.print(hInt); 
    Serial.print("% | Temp "); Serial.print(tInt); Serial.println(" °C");
  } else {
    Serial.println("Erreur");
  }

  // Lecture DHT Extérieur
  float hExt = dhtExt.readHumidity();
  float tExt = dhtExt.readTemperature();
  Serial.print("DHT22 EXT: ");
  if (!isnan(hExt) && !isnan(tExt)) {
    Serial.print("Hum "); Serial.print(hExt); 
    Serial.print("% | Temp "); Serial.print(tExt); Serial.println(" °C");
  } else {
    Serial.println("Erreur");
  }
}

void lirePoids() {
  float poids_g = scale.get_units(5); // Moyenne sur 5 lectures
  if (poids_g < 0 && poids_g > -2) {
    poids_g = 0;
  }
  Serial.print("Poids: "); 
  Serial.print(poids_g / 1000.0, 2); 
  Serial.println(" kg");
}