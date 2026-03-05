#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <SparkFun_ADXL345.h>
#include <DHT.h>
#include "HX711.h"

/* --- CONFIGURATION GÉNÉRALE --- */
#define SERIAL_BAUD 115200    // Augmenté pour l'ESP32
#define PRINT_INTERVAL 30000 // On envoie toutes les 30 secondes pour LoRaWAN

/* --- CONFIGURATION PINS --- */
#define PIN_DS18B20 4
#define PIN_DHT_INT 2
#define PIN_DHT_EXT 15
#define PIN_HX711_DOUT 32
#define PIN_HX711_SCK 33
#define RXD2 16
#define TXD2 17
#define I2C_SDA 21
#define I2C_SCL 22

/* --- CONFIGURATION CAPTEURS --- */
#define DHT_TYPE DHT22
#define LUX_I2C_ADDR 0x23
float calibration_factor = -30.0;

/* --- INSTANCIATION --- */

/* --- LORAWAN KEYS --- */
String devEui = "70B3D57ED0075EA6";
String appEui = "0000000000000000";
String appKey = "802C6FEA2C744EC280D61DC102EBC9EE";

/* --- VARIABLES GLOBALES --- */
unsigned long lastPrintTime = 0;
int dsCount = 0;

// Helper pour convertir float -> uint16 (x100 pour garder 2 décimales)
uint16_t fToU(float v) {
  if (isnan(v) || v < -100) return 0;
  return (uint16_t)((v + 100) * 100); // Offset de +100 pour gérer les temp négatives
}

void envoyerCommandeAT(String cmd) {
  Serial.print("-> "); Serial.println(cmd);
  Serial2.print(cmd + "\r\n");
  delay(1000); 
  while (Serial2.available()) Serial.write(Serial2.read());
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2); // Communication avec LoRa-E5

  Serial.println("Initialisation LoRaWAN...");
  envoyerCommandeAT("AT+ID=DevEui,\"" + devEui + "\"");
  envoyerCommandeAT("AT+ID=AppEui,\"" + appEui + "\"");
  envoyerCommandeAT("AT+KEY=APPKEY,\"" + appKey + "\"");
  envoyerCommandeAT("AT+MODE=LWOTAA");
  envoyerCommandeAT("AT+JOIN");
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - lastPrintTime >= PRINT_INTERVAL) {
    lastPrintTime = currentMillis;

    // 1. Lecture des valeurs (tes fonctions existantes)
    float t_i = 0;
    float t_o = 0;
    float t_1 = 0;
    float t_2 = 0;
    float h_i = 0;
    float h_o = 0;
    float p_g = 0;
    if (p_g < 0) p_g = 0;

    // Lecture Lux
    uint16_t l = 0;

    // 2. Préparation du Payload (Little Endian comme ton Formatter)
    // On met 0 pour Batterie, Chute, Xiao et Entités
    uint16_t batt = 0; 
    uint16_t w = (uint16_t)p_g;

    char payload[45];
    sprintf(payload, "%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X",
      batt & 0xFF, (batt >> 8) & 0xFF,   // Octets 0-1
      fToU(t_i) & 0xFF, (fToU(t_i) >> 8) & 0xFF, // 2-3
      fToU(t_o) & 0xFF, (fToU(t_o) >> 8) & 0xFF, // 4-5
      fToU(t_1) & 0xFF, (fToU(t_1) >> 8) & 0xFF, // 6-7
      fToU(t_2) & 0xFF, (fToU(t_2) >> 8) & 0xFF, // 8-9
      fToU(h_i) & 0xFF, (h_i > 0 ? (uint16_t)h_i : 0) >> 8, // Simplifié pour Hum
      fToU(h_o) & 0xFF, (h_o > 0 ? (uint16_t)h_o : 0) >> 8, 
      l & 0xFF, (l >> 8) & 0xFF,         // Lux 14-15
      w & 0xFF, (w >> 8) & 0xFF,         // Poids 16-17
      0x01,                             // Chute 18
      0x00,                             // Xiao 19
      0x00,                             // Entité 20
      0x00                              // Fiabilité 21
    );

    // 3. Envoi
    envoyerCommandeAT("AT+CMSGHEX=\"" + String(payload) + "\"");
  }
}