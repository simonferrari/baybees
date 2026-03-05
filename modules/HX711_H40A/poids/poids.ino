#include "HX711.h"

const int LOADCELL_DOUT_PIN = 32;
const int LOADCELL_SCK_PIN = 33;

HX711 scale;

// Votre facteur de calibration
float calibration_factor = -29.0; 

void setup() {
  Serial.begin(115200);
  Serial.println("Démarrage de la balance...");

  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale(calibration_factor);
  
  Serial.println("Mise à zéro (Tare)... Videz le plateau !");
  delay(2000); 
  scale.tare(); 
  
  Serial.println("Balance prête ! Posez un objet.");
}

void loop() {
  // Lecture de la valeur en grammes
  float poids_grammes = scale.get_units(10);
  
  // Petite astuce pour garder le 0 parfaitement stable à vide
  if (poids_grammes < 0 && poids_grammes > -2) {
    poids_grammes = 0;
  }

  // Conversion en kilogrammes
  float poids_kg = poids_grammes / 1000.0;

  Serial.print("Poids : ");
  // Le ", 2" force l'affichage de 2 chiffres après la virgule (ex: 0.19 kg)
  Serial.print(poids_kg, 2); 
  Serial.println(" kg");

  delay(500); 
}