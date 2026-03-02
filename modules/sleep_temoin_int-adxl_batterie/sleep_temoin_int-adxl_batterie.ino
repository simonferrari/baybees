#include <SparkFun_ADXL345.h>

#define TEMOIN_LED 12
#define TEMOIN_BUZZER 14
#define ADX345_INT1 13   // GPIO 18 est RTC_GPIO
#define ADC_BATTERIE 35

#define uS_TO_S_FACTOR 1000000ULL

ADXL345 adxl = ADXL345();
RTC_DATA_ATTR int bootCount = 0;

void init_ADXL345(ADXL345 &adxl_ref); 

void setup(){
    Serial.begin(115200);
    delay(500);

    // 1. GESTION BATTERIE
    pinMode(ADC_BATTERIE, INPUT);
    float v_mes = analogRead(ADC_BATTERIE);
    float v_dc = 1.435 * (v_mes / 4095.0) * 3.3;
    
    uint64_t time_to_sleep; // On utilise uint64_t pour les gros chiffres de temps

    if(v_dc < 3.4) {
        Serial.println("BATTERIE CRITIQUE - Sommeil long (1h)");
        time_to_sleep = 5;
        // On s'endort tout de suite, mais on laisse le réveil par mouvement actif !
        esp_sleep_enable_timer_wakeup(time_to_sleep * uS_TO_S_FACTOR);
        esp_sleep_enable_ext0_wakeup((gpio_num_t)ADX345_INT1, 1); 
        esp_deep_sleep_start();
    } 
    else if(v_dc < 3.7) {
        Serial.println("BATTERIE FAIBLE - Sommeil modéré (1h)");
        time_to_sleep = 5; 
    } 
    else {
        Serial.println("BATTERIE OK - Sommeil court (10 min)");
        time_to_sleep = 1; 
    }

    // 2. INITIALISATION MATERIELLE
    pinMode(TEMOIN_LED, OUTPUT);
    pinMode(TEMOIN_BUZZER, OUTPUT);
    adxl.powerOn(); 

    bootCount++;
    Serial.println("Boot : " + String(bootCount) + " | Tension : " + String(v_dc) + "V");

    // 3. ANALYSE CAUSE REVEIL
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();

    if(wakeup_reason == ESP_SLEEP_WAKEUP_EXT0) {
        Serial.println("!!! REVEIL PAR MOUVEMENT !!!");
        byte interrupts = adxl.getInterruptSource();
        if(adxl.triggered(interrupts, ADXL345_FREE_FALL)) {
            Serial.println("ALERTE : CHUTE LIBRE / VOL");
            // Faire biper ou envoyer un message LoRa d'urgence ici
        }
    }

    // 4. PREMIER BOOT (Configuration une seule fois)
    if(bootCount == 1) {
        Serial.println("Config initiale ADXL...");
        init_ADXL345(adxl);
        digitalWrite(TEMOIN_LED, HIGH);
        digitalWrite(TEMOIN_BUZZER, HIGH);
        delay(200);
        digitalWrite(TEMOIN_BUZZER, LOW);
    }

    // 5. PHASE DE MESURE (HX711, DHT22...)
    // if (v_dc >= 3.4) { ... tes fonctions de mesures ... }

    // 6. RETOUR AU SOMMEIL
    esp_sleep_enable_timer_wakeup(time_to_sleep * uS_TO_S_FACTOR);
    esp_sleep_enable_ext0_wakeup((gpio_num_t)ADX345_INT1, 1); 

    Serial.println("Dodo pour " + String(time_to_sleep) + "s");
    Serial.flush(); 
    digitalWrite(TEMOIN_LED, LOW);
    esp_deep_sleep_start();
}

void loop() {}

void init_ADXL345(ADXL345 &adxl_ref) {
    adxl_ref.setFreeFallThreshold(7); 
    adxl_ref.setFreeFallDuration(30);
    adxl_ref.setImportantInterruptMapping(1, 1, 1, 1, 1);
    adxl_ref.FreeFallINT(1);
    adxl_ref.singleTapINT(1);
    adxl_ref.getInterruptSource(); // On vide pour commencer propre
}