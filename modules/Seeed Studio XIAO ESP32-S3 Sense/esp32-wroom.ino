#include <Arduino.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <SparkFun_ADXL345.h>
#include <DHT.h>
#include "HX711.h"
#include <math.h>

/* --- CONSTANTES --- */
#define uS_TO_S_FACTOR 1000000ULL

/* --- STRUCTURES --- */
struct __attribute__((packed)) Payload {
    uint16_t batt;        // 2 octets
    int16_t  t_i;         // 2 octets
    int16_t  t_0;         // 2 octets
    int16_t  t_1;         // 2 octets
    int16_t  t_2;         // 2 octets
    uint16_t h_i;         // 2 octets
    uint16_t h;           // 2 octets
    uint16_t lux;         // 2 octets
    uint16_t poids;       // 2 octets
    uint8_t  chute;       // 1 octet
    int16_t  x;           // 2 octets
    int16_t  y;           // 2 octets
    int16_t  z;           // 2 octets
    uint8_t  frelon;      // venant du XIAO
    uint8_t  nb_frelon;   // venant du XIAO
    uint8_t  frelon_acc;  // venant du XIAO
    uint8_t  audio_classe;
    uint8_t  audio_conf;
};

/* --- CONFIGURATION GÉNÉRALE --- */
#define SERIAL_BAUD         115200
#define SEND_FREQUENCY_HIGH 30
#define SEND_FREQUENCY_LOW  3600

/* --- CONFIGURATION PINS --- */
#define PIN_DS18B20         4
#define PIN_DHT_INT         15
#define PIN_DHT_EXT         2
#define PIN_HX711_DOUT      32
#define PIN_HX711_SCK       33
#define PIN_ADXL345_INT1    13
#define TEMOIN_BUZZER       19
#define ADC_BATTERIE        35
#define SWITCH_ALIM_CAPTEUR 26
#define SWITCH_ALIM_UC      27
#define I2C_SDA             21
#define I2C_SCL             22
#define RXD2                16
#define TXD2                17

/* --- CONFIGURATION CAPTEURS --- */
#define DHT_TYPE      DHT22
#define LUX_I2C_ADDR  0x23

/* --- I2C AUTRES UC --- */
#define NANO_I2C_ADDR 0x08
#define XIAO_I2C_ADDR 0x12

/* --- CODES AUDIO --- */
#define AUDIO_BEEQUEEN  0
#define AUDIO_HORNET    1
#define AUDIO_NOBEE     2
#define AUDIO_NOQUEEN   3
#define AUDIO_PIPING    4
#define AUDIO_INCONNU   255

/* --- MÉMOIRE RTC ENTRE LES BOOTS --- */
RTC_DATA_ATTR int      bootCount         = 0;
RTC_DATA_ATTR long     hx711_offset      = 0;
RTC_DATA_ATTR uint8_t  last_audio_classe = AUDIO_INCONNU;
RTC_DATA_ATTR uint8_t  last_audio_conf   = 0;
RTC_DATA_ATTR uint8_t  last_frelon       = 0;
RTC_DATA_ATTR uint8_t  last_nb_frelon    = 0;
RTC_DATA_ATTR uint8_t  last_frelon_acc   = 0;

/* --- VARIABLES GLOBALES --- */
float calibration_factor = -29.0;
int   dsCount            = 0;
int   send_frequency     = SEND_FREQUENCY_HIGH;

/* --- LORA --- */
String devEui = "70B3D57ED0073206";
String appEui = "0000000000000000";
String appKey = "FE7BB9E5F461932685E0C59807BC8852";

/* --- INSTANCIATION --- */
OneWire oneWire(PIN_DS18B20);
DallasTemperature sensors(&oneWire);
ADXL345 adxl = ADXL345();
DHT dhtInt(PIN_DHT_INT, DHT_TYPE);
DHT dhtExt(PIN_DHT_EXT, DHT_TYPE);
HX711 scale;

/* --- PROTOTYPES --- */
void envoyerCommandeAT(const String &cmd);
void lireDS18B20(float* temp_ds18b20, int maxCount);
void lireADXL(int16_t &x, int16_t &y, int16_t &z);
void lireLux(float &lux_SEN0562);
void lireDHT(DHT &dht, float &temp_dht, float &hum_dht);
void lirePoids(float &kg_HX711);
void lireNano(uint8_t &audio_classe, uint8_t &audio_conf);
void lireXIAO(uint8_t &frelon, uint8_t &nb_frelon, uint8_t &frelon_acc);
String payloadToHex(const Payload &data);
int16_t  floatToTemp10(float v);
uint16_t floatToU16(float v);

/* -------------------------------------------------------------------------- */
/*                                   SETUP                                    */
/* -------------------------------------------------------------------------- */
void setup() {
    Serial.begin(SERIAL_BAUD);
    delay(500);

    Serial.println();
    Serial.println("====================================");
    Serial.println("[uPesy] Boot");
    Serial.println("====================================");

    Payload data = {};   // IMPORTANT : tout initialiser à 0

    // LORA-E5
    Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);

    // Gestion du nombre de boot
    bootCount++;
    Serial.print("[uPesy] Boot numero: ");
    Serial.println(bootCount);

    // Gestion batterie
    pinMode(ADC_BATTERIE, INPUT);
    float v_mes = analogRead(ADC_BATTERIE);
    float v_dc  = 1.435f * (v_mes / 4095.0f) * 3.3f;

    Serial.print("[uPesy] Batterie mesuree: ");
    Serial.print(v_dc, 3);
    Serial.println(" V");

    if (v_dc < 3.4f) {
        Serial.println("[uPesy] Batterie critique -> sommeil long immediat");
        send_frequency = SEND_FREQUENCY_LOW;
        esp_sleep_enable_timer_wakeup((uint64_t)send_frequency * uS_TO_S_FACTOR);
        esp_sleep_enable_ext0_wakeup((gpio_num_t)PIN_ADXL345_INT1, 1);
        esp_deep_sleep_start();
    }
    else if (v_dc < 3.7f) {
        Serial.println("[uPesy] Batterie faible -> sommeil long");
        send_frequency = SEND_FREQUENCY_LOW;
    }
    else {
        Serial.println("[uPesy] Batterie OK -> sommeil court");
        send_frequency = SEND_FREQUENCY_HIGH;
    }

    // Alimentation cartes/capteurs
    Serial.println("[uPesy] Alimentation...");
    gpio_hold_dis((gpio_num_t)SWITCH_ALIM_CAPTEUR);
    gpio_hold_dis((gpio_num_t)SWITCH_ALIM_UC);

    pinMode(SWITCH_ALIM_CAPTEUR, OUTPUT);
    pinMode(SWITCH_ALIM_UC, OUTPUT);

    digitalWrite(SWITCH_ALIM_CAPTEUR, HIGH);
    digitalWrite(SWITCH_ALIM_UC, HIGH);

    Serial.println("[uPesy] Alimentation capteurs + uC ON");
    delay(10000); // temps de boot XIAO + camera + inference

    // I2C
    Wire.begin(I2C_SDA, I2C_SCL, 100000);
    Serial.printf("[uPesy] I2C master pret SDA=%d SCL=%d\n", I2C_SDA, I2C_SCL);

    // Initialisation HX711
    // scale.begin(PIN_HX711_DOUT, PIN_HX711_SCK);
    // scale.set_scale(calibration_factor);

    // Initialisation DS18B20
    sensors.begin();
    dsCount = sensors.getDeviceCount();
    Serial.print("[uPesy] Nombre DS18B20 detectes: ");
    Serial.println(dsCount);

    float temp_ds18b20[2] = {NAN, NAN};

    // Initialisation DHT
    dhtInt.begin();
    dhtExt.begin();
    float temp_dhtInt = NAN;
    float hum_dhtInt  = NAN;
    float temp_dhtExt = NAN;
    float hum_dhtExt  = NAN;

    // Initialisation ADXL345
    adxl.powerOn();
    adxl.setRangeSetting(16);
    adxl.setSpiBit(0);

    adxl.setTapDetectionOnXYZ(0, 0, 1);
    adxl.setTapThreshold(10);
    adxl.setTapDuration(10);
    adxl.setFreeFallThreshold(7);
    adxl.setFreeFallDuration(30);

    bool alerte_chute = 0;

    // Initialisation lux
    float lux_sen0562 = NAN;

    // Premier boot : buzzer + join LoRa
    if (bootCount == 1) {
        pinMode(TEMOIN_BUZZER, OUTPUT);
        Serial.println("[uPesy] Premier boot");

        digitalWrite(TEMOIN_BUZZER, HIGH);
        delay(200);
        digitalWrite(TEMOIN_BUZZER, LOW);

        envoyerCommandeAT("AT");
        envoyerCommandeAT("AT+ID=DevEui,\"" + devEui + "\"");
        envoyerCommandeAT("AT+ID=AppEui,\"" + appEui + "\"");
        envoyerCommandeAT("AT+KEY=APPKEY,\"" + appKey + "\"");
        envoyerCommandeAT("AT+MODE=LWOTAA");
        envoyerCommandeAT("AT+JOIN");
        delay(15000);

        // scale.tare();
        // hx711_offset = scale.get_offset();
    }

    // Hors premier boot
    // scale.set_offset(hx711_offset);

    Serial.println("[uPesy] MESURES");

    // Mesures locales
    Serial.print("[uPesy] Batterie : ");
    Serial.print(v_dc, 3);
    Serial.println(" V");

    lireDS18B20(temp_ds18b20, 2);
    lireLux(lux_sen0562);
    lireDHT(dhtInt, temp_dhtInt, hum_dhtInt);
    lireDHT(dhtExt, temp_dhtExt, hum_dhtExt);

    // float kg_HX711 = 0.0f;
    // lirePoids(kg_HX711);

    int16_t ax = 0, ay = 0, az = 0;
    lireADXL(ax, ay, az);

    // Lecture Nano audio
    uint8_t val_audio_classe = AUDIO_INCONNU;
    uint8_t val_audio_conf   = 0;
    lireNano(val_audio_classe, val_audio_conf);

    // Lecture XIAO vision
    uint8_t val_frelon     = 0;
    uint8_t val_nb_frelon  = 0;
    uint8_t val_frelon_acc = 0;
    lireXIAO(val_frelon, val_nb_frelon, val_frelon_acc);

    // Remplissage payload
    data.batt        = floatToU16(v_dc * 100.0f);
    data.t_i         = floatToTemp10(temp_dhtInt);
    data.t_0         = floatToTemp10(temp_dhtExt);
    data.t_1         = floatToTemp10(temp_ds18b20[0]);
    data.t_2         = floatToTemp10(temp_ds18b20[1]);
    data.h_i         = floatToU16(hum_dhtInt);
    data.h           = floatToU16(hum_dhtExt);
    data.lux         = floatToU16(lux_sen0562);
    data.poids       = 0; // ou floatToU16(kg_HX711) si HX711 activé
    data.chute       = alerte_chute ? 1 : 0;
    data.x           = ax;
    data.y           = ay;
    data.z           = az;
    data.frelon      = val_frelon;
    data.nb_frelon   = val_nb_frelon;
    data.frelon_acc  = val_frelon_acc;
    data.audio_classe= val_audio_classe;
    data.audio_conf  = val_audio_conf;

    // Affichage payload
    Serial.println("[uPesy] Payload rempli :");
    Serial.print("  batt       = "); Serial.println(data.batt);
    Serial.print("  t_i        = "); Serial.println(data.t_i);
    Serial.print("  t_0        = "); Serial.println(data.t_0);
    Serial.print("  t_1        = "); Serial.println(data.t_1);
    Serial.print("  t_2        = "); Serial.println(data.t_2);
    Serial.print("  h_i        = "); Serial.println(data.h_i);
    Serial.print("  h          = "); Serial.println(data.h);
    Serial.print("  lux        = "); Serial.println(data.lux);
    Serial.print("  poids      = "); Serial.println(data.poids);
    Serial.print("  chute      = "); Serial.println(data.chute);
    Serial.print("  x          = "); Serial.println(data.x);
    Serial.print("  y          = "); Serial.println(data.y);
    Serial.print("  z          = "); Serial.println(data.z);
    Serial.print("  frelon     = "); Serial.println(data.frelon);
    Serial.print("  nb_frelon  = "); Serial.println(data.nb_frelon);
    Serial.print("  frelon_acc = "); Serial.println(data.frelon_acc);
    Serial.print("  audio_cls  = "); Serial.println(data.audio_classe);
    Serial.print("  audio_conf = "); Serial.println(data.audio_conf);

    // Conversion HEX
    String hexPayload = payloadToHex(data);
    Serial.print("[uPesy] HEX payload = ");
    Serial.println(hexPayload);

    // Envoi LoRaWAN
    envoyerCommandeAT("AT+CMSGHEX=\"" + hexPayload + "\"");

    // Extinction alimentation
    digitalWrite(SWITCH_ALIM_CAPTEUR, LOW);
    digitalWrite(SWITCH_ALIM_UC, LOW);
    delay(10);

    gpio_hold_en((gpio_num_t)SWITCH_ALIM_CAPTEUR);
    gpio_hold_en((gpio_num_t)SWITCH_ALIM_UC);

    adxl.getInterruptSource();

    Serial.println("[uPesy] dodo...");

    esp_sleep_enable_timer_wakeup((uint64_t)send_frequency * uS_TO_S_FACTOR);
    esp_sleep_enable_ext0_wakeup((gpio_num_t)PIN_ADXL345_INT1, 1);
    esp_deep_sleep_start();
}

void loop() {}

/* -------------------------------------------------------------------------- */
/*                                  FONCTIONS                                 */
/* -------------------------------------------------------------------------- */

void lireDS18B20(float* temp_ds18b20, int maxCount) {
    sensors.requestTemperatures();

    int countToRead = dsCount < maxCount ? dsCount : maxCount;

    for (int i = 0; i < countToRead; i++) {
        temp_ds18b20[i] = sensors.getTempCByIndex(i);
        Serial.print("[DS18B20] [");
        Serial.print(i);
        Serial.print("] : ");
        Serial.print(temp_ds18b20[i]);
        Serial.println(" °C");
    }

    for (int i = countToRead; i < maxCount; i++) {
        temp_ds18b20[i] = NAN;
    }
}

void lireADXL(int16_t &x, int16_t &y, int16_t &z) {
    int ax, ay, az;
    adxl.readAccel(&ax, &ay, &az);

    x = (int16_t)ax;
    y = (int16_t)ay;
    z = (int16_t)az;

    Serial.print("[ADXL] Accel XYZ: ");
    Serial.print(ax); Serial.print(", ");
    Serial.print(ay); Serial.print(", ");
    Serial.println(az);
}

void lireLux(float &lux_SEN0562) {
    Wire.beginTransmission(LUX_I2C_ADDR);
    Wire.write(0x10);

    if (Wire.endTransmission() == 0) {
        delay(180); // plus sûr pour une mesure BH1750
        Wire.requestFrom(LUX_I2C_ADDR, 2);

        if (Wire.available() == 2) {
            uint16_t data = (Wire.read() << 8) | Wire.read();
            lux_SEN0562 = data / 1.2f;
            Serial.print("[LUX] Luminosite: ");
            Serial.print(lux_SEN0562);
            Serial.println(" lx");
            return;
        }
    }

    lux_SEN0562 = NAN;
    Serial.println("[LUX] Erreur lecture");
}

void lireDHT(DHT &dht, float &temp_dht, float &hum_dht) {
    hum_dht  = dht.readHumidity();
    temp_dht = dht.readTemperature();

    Serial.print("[DHT] ");
    if (!isnan(hum_dht) && !isnan(temp_dht)) {
        Serial.print("Hum ");
        Serial.print(hum_dht);
        Serial.print("% | Temp ");
        Serial.print(temp_dht);
        Serial.println(" °C");
    } else {
        Serial.println("Erreur lecture");
    }
}

void lirePoids(float &kg_HX711) {
    kg_HX711 = scale.get_units(5);
    if (kg_HX711 < 0 && kg_HX711 > -2) kg_HX711 = 0;

    Serial.print("[HX711] Poids: ");
    Serial.print(kg_HX711 / 1000.0f, 2);
    Serial.println(" kg");
}

void lireNano(uint8_t &audio_classe, uint8_t &audio_conf) {
    uint8_t nb = Wire.requestFrom((int)NANO_I2C_ADDR, 2);

    if (nb == 2 && Wire.available() >= 2) {
        audio_classe = Wire.read();
        audio_conf   = Wire.read();

        last_audio_classe = audio_classe;
        last_audio_conf   = audio_conf;

        Serial.print("[NANO] classe=");
        Serial.print(audio_classe);
        Serial.print(" conf=");
        Serial.print(audio_conf);
        Serial.println("%");
        return;
    }

    while (Wire.available()) {
        Wire.read();
    }

    Serial.println("[NANO] Pas de reponse -> conservation des dernieres valeurs");
    audio_classe = last_audio_classe;
    audio_conf   = last_audio_conf;
}

void lireXIAO(uint8_t &frelon, uint8_t &nb_frelon, uint8_t &frelon_acc) {
    const uint32_t timeout_ms = 15000;
    const uint32_t retry_ms   = 500;

    uint32_t t0 = millis();

    while (millis() - t0 < timeout_ms) {
        int n = Wire.requestFrom((int)XIAO_I2C_ADDR, 3);

        if (n == 3 && Wire.available() >= 3) {
            frelon     = Wire.read();
            nb_frelon  = Wire.read();
            frelon_acc = Wire.read();

            last_frelon     = frelon;
            last_nb_frelon  = nb_frelon;
            last_frelon_acc = frelon_acc;

            Serial.print("[XIAO] frelon=");
            Serial.print(frelon);
            Serial.print(" nb=");
            Serial.print(nb_frelon);
            Serial.print(" conf=");
            Serial.print(frelon_acc);
            Serial.println("%");
            return;
        }

        while (Wire.available()) {
            Wire.read();
        }

        Serial.println("[XIAO] pas pret, nouvelle tentative...");
        delay(retry_ms);
    }

    Serial.println("[XIAO] timeout -> conservation des dernieres valeurs");
    frelon     = last_frelon;
    nb_frelon  = last_nb_frelon;
    frelon_acc = last_frelon_acc;
}

String payloadToHex(const Payload &data) {
    const uint8_t* pBytes = (const uint8_t*)&data;
    String hexPayload = "";

    for (size_t i = 0; i < sizeof(Payload); i++) {
        if (pBytes[i] < 0x10) hexPayload += "0";
        hexPayload += String(pBytes[i], HEX);
    }

    hexPayload.toUpperCase();
    return hexPayload;
}

void envoyerCommandeAT(const String &cmd) {
    Serial.print("-> ");
    Serial.println(cmd);

    Serial2.print(cmd + "\r\n");

    uint32_t t0 = millis();
    while (millis() - t0 < 2000) {
        while (Serial2.available()) {
            Serial.write(Serial2.read());
        }
    }
}

int16_t floatToTemp10(float v) {
    if (isnan(v)) return 0;
    return (int16_t)(v * 10.0f);
}

uint16_t floatToU16(float v) {
    if (isnan(v) || v < 0.0f) return 0;
    return (uint16_t)(v);
}