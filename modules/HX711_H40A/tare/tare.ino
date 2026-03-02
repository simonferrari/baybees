#include "HX711.h"

// --- CONFIGURATION PINS ESP32 ---
const int LOADCELL_DOUT_PIN = 32;
const int LOADCELL_SCK_PIN = 33;

HX711 scale;

// Facteur de départ (tu l'ajusteras avec le moniteur série)
float calibration_factor = -41.0; 

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n========================================");
  Serial.println("   OUTIL DE CALIBRATION HX711 + H40A    ");
  Serial.println("========================================\n");

  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  
  // On initialise avec un facteur de 1 pour lire les valeurs brutes au début
  scale.set_scale();
  
  Serial.println("INSTRUCTIONS :");
  Serial.println("1. Videz le plateau.");
  Serial.println("2. Envoyez 't' pour faire la TARE (Zéro).");
  Serial.println("3. Posez un poids connu (ex: 500g).");
  Serial.println("4. Ajustez le facteur avec 'a'/'z' ou 'q'/'s' jusqu'a lire le bon poids.");
  Serial.println("----------------------------------------");
}

void loop() {
  // Applique le facteur actuel pour la lecture
  scale.set_scale(calibration_factor);

  // Lecture du poids (moyenne sur 5 lectures pour plus de réactivité)
  float poids = scale.get_units(5);

  Serial.print("Poids : ");
  Serial.print(poids, 2); 
  Serial.print(" g | Facteur : ");
  Serial.println(calibration_factor);

  // Gestion des commandes série
  if(Serial.available()) {
    char touche = Serial.read();
    
    // --- LA TARE ---
    if(touche == 't' || touche == 'T') {
      Serial.println("\n>>> TARE EN COURS... Ne touchez a rien.");
      scale.tare();
      Serial.println(">>> ZERO TERMINE.\n");
    }
    
    // --- AJUSTEMENTS ---
    else if(touche == 'a') calibration_factor += 1.0;    // Très fin
    else if(touche == 'z') calibration_factor -= 1.0;    // Très fin
    else if(touche == 'q') calibration_factor += 10.0;   // Moyen
    else if(touche == 's') calibration_factor -= 10.0;   // Moyen
    else if(touche == 'w') calibration_factor += 100.0;  // Rapide
    else if(touche == 'x') calibration_factor -= 100.0;  // Rapide
  }
  
  delay(200); 
}