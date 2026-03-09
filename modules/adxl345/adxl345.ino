#include <Wire.h>
#include <SparkFun_ADXL345.h>

ADXL345 adxl = ADXL345();             // I2C par défaut
#define INTERRUPT_PIN GPIO_NUM_13     // Pin de réveil

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n--- TEST REVEIL ADXL345 ---");

  // 1. Initialisation I2C
  Wire.begin(21, 22);

  // 2. Vérification de la présence du capteur
  //byte deviceID = adxl.readRegister(0x00);
  //if (deviceID != 0xE5) {
  //  Serial.printf("Erreur: ADXL non trouvé (ID: 0x%02X). Vérifiez le câblage !\n", deviceID);
  //  while (1); 
  //}
  //Serial.println("ADXL345 détecté (ID: 0xE5)");

  // 3. Configuration ADXL
  adxl.powerOn();                     
  adxl.setRangeSetting(16);           

  // --- Configuration de la sensibilité ---
  adxl.setTapThreshold(15);           // Seuil du choc (bas = sensible)
  adxl.setTapDuration(15);            // Durée
  adxl.setTapDetectionOnXYZ(1, 1, 1); // Détecte sur X, Y et Z

  // --- Configuration des Interruptions ---
  adxl.setImportantInterruptMapping(1, 1, 1, 1, 1); // Tout envoyer sur INT1
  adxl.setInterruptLevelBit(0);       // 0 = Signal HIGH au déclenchement
  
  adxl.singleTapINT(1);               // Activer Single Tap
  adxl.doubleTapINT(1);               // Activer Double Tap
  adxl.FreeFallINT(1);                // Activer Free Fall
  adxl.ActivityINT(1);                // Activer mouvement brusque

  // 4. Analyse de la cause du réveil
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();

  if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT0) {
    Serial.println("RÉVEIL : L'ADXL a envoyé une interruption !");
    
    // On lit le registre pour savoir quel type d'interruption et VIDER le flag
    byte source = adxl.getInterruptSource();
    
    if(adxl.triggered(source, ADXL345_SINGLE_TAP)) Serial.println("-> Cause: SIMPLE TAP");
    if(adxl.triggered(source, ADXL345_DOUBLE_TAP)) Serial.println("-> Cause: DOUBLE TAP");
    if(adxl.triggered(source, ADXL345_FREE_FALL))  Serial.println("-> Cause: CHUTE LIBRE");
    if(adxl.triggered(source, ADXL345_ACTIVITY))   Serial.println("-> Cause: ACTIVITÉ");
  } else {
    Serial.println("RÉVEIL : Premier démarrage ou Timer.");
  }

  // 5. Préparation au dodo
  Serial.println("Nettoyage du registre ADXL...");
  adxl.getInterruptSource(); // Lecture à vide pour remettre la pin INT1 à 0

  Serial.println("Mise en Deep Sleep (Pin 13 à l'écoute)...");
  delay(100); // Laisser le temps au Serial de finir

  // Configuration du réveil sur Pin 13, niveau HIGH (1)
  esp_sleep_enable_ext0_wakeup(INTERRUPT_PIN, 1); 
  
  esp_deep_sleep_start();
}

void loop() {
  // Jamais atteint en Deep Sleep
}