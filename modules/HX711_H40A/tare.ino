#include "HX711.h"

// Broches pour l'ESP32
const int LOADCELL_DOUT_PIN = 16;
const int LOADCELL_SCK_PIN = 4;

HX711 scale;

// On commence avec un facteur neutre
float calibration_factor = 1.0; 

void setup() {
  Serial.begin(115200);
  Serial.println("Démarrage de la calibration...");

  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  
  // On remet le facteur à zéro par défaut
  scale.set_scale();
  
  Serial.println("1. Videz le plateau de la balance.");
  delay(3000);
  
  // La fameuse TARE est faite ici
  scale.tare(); 
  Serial.println("Tare effectuée ! Le point zéro est défini.");
  
  Serial.println("2. Posez maintenant un poids connu sur la balance (ex: 500g, 1kg...).");
  Serial.println("3. Dans le moniteur série, tapez :");
  Serial.println("   '+' ou 'a' pour augmenter le facteur");
  Serial.println("   '-' ou 'z' pour diminuer le facteur");
  Serial.println("   Appuyez sur 'Entrée' pour envoyer la commande.");
}

void loop() {
  // On applique le facteur actuel
  scale.set_scale(calibration_factor);

  Serial.print("Poids mesuré : ");
  // Affiche la valeur mesurée. On cherche à faire correspondre cette valeur à votre poids réel
  Serial.print(scale.get_units(10), 2); 
  Serial.print(" | Facteur de calibration actuel : ");
  Serial.println(calibration_factor);

  // Écoute ce que vous tapez dans le moniteur série
  if(Serial.available()) {
    char temp = Serial.read();
    if(temp == '+' || temp == 'a')
      calibration_factor += 10;   // Ajustement fin
    else if(temp == '-' || temp == 'z')
      calibration_factor -= 10;   // Ajustement fin
    else if(temp == '*' || temp == 'q')
      calibration_factor += 100;  // Ajustement rapide
    else if(temp == '/' || temp == 's')
      calibration_factor -= 100;  // Ajustement rapide
  }
  
  delay(500); // Petit délai pour ne pas inonder le moniteur série
}