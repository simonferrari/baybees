#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <SparkFun_ADXL345.h>
#include <DHT.h>
#include "HX711.h"

/* --- CONSTANTES ---*/
#define uS_TO_S_FACTOR 1000000ULL

/* --- CONFIGURATION GÉNÉRALE --- */
#define SERIAL_BAUD 115200
#define SEND_FREQUENCY_HIGH 600  // Affichage toutes les 3 secondes
#define SEND_FREQUENCY_LOW 3600

/* --- CONFIGURATION PINS --- */
#define PIN_DS18B20 2
#define PIN_DHT_INT 19    // DHT Intérieur
#define PIN_DHT_EXT 18    // DHT Extérieur
#define PIN_HX711_DOUT 5
#define PIN_HX711_SCK 4
#define PIN_ADXL345_INT1 25
#define TEMOIN_BUZZER 15
#define ADC_BATTERIE 35
#define SWITCH_ALIM_CAPTEUR 26
#define SWITCH_ALIM_UC 27
#define I2C_SDA 21
#define I2C_SCL 22

/* --- CONFIGURATION CAPTEURS --- */
#define DHT_TYPE DHT22
#define LUX_I2C_ADDR 0x23
float calibration_factor = -29.0;

/* --- INSTANCIATION --- */
OneWire oneWire(PIN_DS18B20);
DallasTemperature sensors(&oneWire);
ADXL345 adxl = ADXL345();
DHT dhtInt(PIN_DHT_INT, DHT_TYPE);
DHT dhtExt(PIN_DHT_EXT, DHT_TYPE);
HX711 scale;

/* --- VARIABLES GLOBALES --- */
int dsCount = 0;
RTC_DATA_ATTR int bootCount = 0;
RTC_DATA_ATTR long hx711_offset = 0;
uint8_t luxBuf[2];
int send_frequency;

/* --- FONCTIONS ---*/

void setup() {
    Serial.begin(SERIAL_BAUD);

    // Gestion du nombre de boot
    bootCount++;

    // Gestion de la batterie
    pinMode(ADC_BATTERIE, INPUT);
    float v_mes = analogRead(ADC_BATTERIE);
    float v_dc = 1.435 * (v_mes / 4095.0) * 3.3;

    if(v_dc < 3.4) {
        // deepsleep imédiat avec interruption toujours active
        Serial.println("batterie critique - sommeil long, deep sleep immédiat");
        send_frequency = SEND_FREQUENCY_LOW;
        esp_sleep_enable_timer_wakeup(send_frequency * uS_TO_S_FACTOR);
        esp_sleep_enable_ext0_wakeup((gpio_num_t)PIN_ADXL345_INT1, 1); 
        esp_deep_sleep_start();
    }
    else if(v_dc < 3.7) {
        Serial.println("batterie faible - sommeil long");
        send_frequency = SEND_FREQUENCY_LOW;
    } 
    else {
        Serial.println("batterie ok - sommeil court");
        send_frequency = SEND_FREQUENCY_HIGH; 
    }

    // Initialisation switch pour alimenter les capteurs + uC
    pinMode(SWITCH_ALIM_CAPTEUR, OUTPUT);
    pinMode(SWITCH_ALIM_UC, OUTPUT);

    digitalWrite(SWITCH_ALIM_CAPTEUR, LOW);
    digitalWrite(SWITCH_ALIM_UC, HIGH);
    delay(500);

    // Initialisation OneWire pour DS18B20
    sensors.begin();
    dsCount = sensors.getDeviceCount();
    float temp_ds18b20[dsCount];

    // Initialisation I2C 
    Wire.begin(I2C_SDA, I2C_SCL);

    // Initialisation ADXL345
    /* a faire qu'une fois car:
        - possède une mémoire
        - ADXL toujours alimenté donc ne perd pas sa configuration
    */
    adxl.powerOn();
    adxl.setRangeSetting(16);
    // Detection du TAP pour le debug
    adxl.setTapDetectionOnXYZ(0, 0, 1);
    adxl.setTapThreshold(50);
    adxl.setTapDuration(15);
    adxl.setFreeFallThreshold(7);
    adxl.setFreeFallDuration(30);
    // Activer les interruptions
    adxl.setImportantInterruptMapping(1, 1, 1, 1, 1);
    adxl.FreeFallINT(1);
    adxl.singleTapINT(1);

    bool alerte_chute = 0;

    // Initialisation DHT
    dhtInt.begin();
    dhtExt.begin();
    float temp_dhtInt;
    float hum_dhtInt;
    float temp_dhtExt;
    float hum_dhtExt;

    // Initialisation HX711
    scale.begin(PIN_HX711_DOUT, PIN_HX711_SCK);
    scale.set_scale(calibration_factor);
    float kg_HX711;

    // Initialisation SEN0562
    float lux_sen0562;
    
    // Cause de réveil
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();

    if(wakeup_reason == ESP_SLEEP_WAKEUP_EXT0) {
        Serial.println("reveil par interruption");
        byte interrupts = adxl.getInterruptSource();
        if(adxl.triggered(interrupts, ADXL345_FREE_FALL)) {
            Serial.println("interruption provenant de l'ADXL");
            alerte_chute = 1;
        }
    }

    // Premier boot (temoin sonore, tare de la balance)
    if(bootCount == 1) {
        Serial.println("Premier boot");
        digitalWrite(TEMOIN_BUZZER, HIGH);
        delay(200);
        digitalWrite(TEMOIN_BUZZER, LOW);
        scale.tare();
        hx711_offset = scale.get_offset();
    }
    // application du tare hors premier boot
    scale.tare(hx711_offset);

    // Mesures
    digitalWrite(SWITCH_ALIM_CAPTEUR, LOW);
    lireDS18B20(temp_ds18b20);
    lireLux(lux_sen0562);
    lireDHT(dhtInt, temp_dhtInt, hum_dhtInt);
    lireDHT(dhtExt, temp_dhtExt, hum_dhtExt);
    lirePoids(kg_HX711);
    digitalWrite(SWITCH_ALIM_CAPTEUR, HIGH);
    // Mesures autres uC
    digitalWrite(SWITCH_ALIM_UC, LOW);
    //...
    digitalWrite(SWITCH_ALIM_UC, HIGH);

    // Envoi LORA
    // Contatenation des alertes + mesures

    Serial.println("dodo...");
    esp_sleep_enable_timer_wakeup(send_frequency * uS_TO_S_FACTOR);
    esp_sleep_enable_ext0_wakeup((gpio_num_t)PIN_ADXL345_INT1, 1); 
    esp_deep_sleep_start();
}

void loop() {}

/* --- FONCTIONS DE LECTURE --- */

void lireDS18B20(float* temp_ds18b20) {
    sensors.requestTemperatures();
    for(int i=0; i<dsCount; i++)
    {
        temp_ds18b20[i] = sensors.getTempCByIndex(i);
        Serial.print("DS18B20 ["); Serial.print(i); Serial.print("]: ");
        Serial.print(temp_ds18b20[i]); Serial.println(" °C");
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

void lireLux(float &lux_SEN0562) {
    // Lecture simplifiée du SEN0562
    Wire.beginTransmission(LUX_I2C_ADDR);
    Wire.write(0x10);
    if (Wire.endTransmission() == 0) {
        delay(20); // Petit délai interne requis par le capteur
        Wire.requestFrom(LUX_I2C_ADDR, 2);
        if(Wire.available() == 2) {
            uint16_t data = Wire.read() << 8 | Wire.read();
            lux_SEN0562 = data / 1.2;
            Serial.print("Luminosité: "); Serial.print(lux_SEN0562); Serial.println(" lx");
        }
    }
}

void lireDHT(DHT &dht, float &temp_dht, float &hum_dht) {
    // Lecture DHT Intérieur
    hum_dht = dht.readHumidity();
    temp_dht = dht.readTemperature();
    Serial.print("DHT22 INT: ");
    if (!isnan(hum_dht) && !isnan(temp_dht)) {
        Serial.print("Hum "); Serial.print(hum_dht); 
        Serial.print("% | Temp "); Serial.print(temp_dht); Serial.println(" °C");
    } else {
        Serial.println("Erreur");
    }
}

void lirePoids(float &kg_HX711) {
    kg_HX711 = scale.get_units(5); // Moyenne sur 5 lectures
    if (kg_HX711 < 0 && kg_HX711 > -2) kg_HX711 = 0;
    Serial.print("Poids: "); Serial.print(kg_HX711 / 1000.0, 2); Serial.println(" kg");
}