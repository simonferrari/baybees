#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <SparkFun_ADXL345.h>
#include <DHT.h>
#include "HX711.h"

struct __attribute__((packed)) Payload {
  uint16_t batt;      // 2 octets
  int16_t  t_i;       // 2 octets (signé, gère le négatif nativement)
  int16_t  t_0;       // 2 octets
  int16_t  t_1;       // 2 octets
  int16_t  t_2;       // 2 octets
  uint16_t h_i;       // 2 octets
  uint16_t h;       // 2 octets
  uint16_t lux;       // 2 octets
  uint16_t poids;     // 2 octets
  uint8_t  chute;     // 1 octet
};

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

    Payload p;
    p.batt      = 4100;           // mV
    p.t_i       = (int16_t)(22.5 * 10); // Envoie 225 (plus simple que +100)
    p.t_0       = (int16_t)(-5.2 * 10); // Envoie -52 (le signé gère le négatif)
    p.t_1       = (int16_t)(15.2 * 10); // Envoie -52 (le signé gère le négatif)
    p.t_2       = (int16_t)(2.2 * 10); // Envoie -52 (le signé gère le négatif)
    p.h_i       = 65;             // %
    p.h       = 2;             // %
    p.lux       = 800;
    p.poids     = 1250;           // grammes
    p.chute     = 0;

    byte* pBytes = (byte*)&p;
    String hexPayload = "";
    for (int i = 0; i < sizeof(p); i++) {
      if (pBytes[i] < 0x10) hexPayload += "0";
      hexPayload += String(pBytes[i], HEX);
    }
    envoyerCommandeAT("AT+CMSGHEX=\"" + hexPayload + "\"");
  }
}