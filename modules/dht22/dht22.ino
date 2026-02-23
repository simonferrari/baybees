/*
   ______               _                  _///_ _           _                   _
  /   _  \             (_)                |  ___| |         | |                 (_)
  |  [_|  |__  ___  ___ _  ___  _ __      | |__ | | ___  ___| |_ _ __ ___  _ __  _  ___  _   _  ___
  |   ___/ _ \| __|| __| |/ _ \| '_ \_____|  __|| |/ _ \/  _|  _| '__/   \| '_ \| |/   \| | | |/ _ \
  |  |  | ( ) |__ ||__ | | ( ) | | | |____| |__ | |  __/| (_| |_| | | (_) | | | | | (_) | |_| |  __/
  \__|   \__,_|___||___|_|\___/|_| [_|    \____/|_|\___|\____\__\_|  \___/|_| |_|_|\__  |\__,_|\___|
                                                                                      | |
                                                                                      \_|
  Fichier :       DHT22surMoniteurSerie.ino
  
  Description :   Programme permettant d'afficher sur le moniteur série de l'IDE Arduino, des températures et taux d'humidité
                  mesurés par un DHT22, branché sur la pin D6 d'un Arduino Uno

  Auteur :        Jérôme TOMSKI (https://passionelectronique.fr/)
  Créé le :       06.06.2021

  Librairie utilisée : DHT sensor library (https://github.com/adafruit/DHT-sensor-library)

*/

#include <DHT.h>
#define brocheDeBranchementDHT 2    // La ligne de communication du DHT22 sera donc branchée sur la pin D6 de l'Arduino
#define typeDeDHT DHT22             // Ici, le type de DHT utilisé est un DHT22 (que vous pouvez changer en DHT11, DHT21, ou autre, le cas échéant)

// Instanciation de la librairie DHT
DHT dht(brocheDeBranchementDHT, typeDeDHT);


// ========================
// Initialisation programme
// ========================
void setup () {
  
  // Initialisation de la liaison série (pour retourner les infos au moniteur série de l'ordi)
  Serial.begin(9600);
  Serial.println("Programme de test du DHT22");
  Serial.println("==========================");
  Serial.println();

  // Initialisation du DHT22;
  dht.begin();
}
 
// =================
// Boucle principale
// =================
void loop () {
  
  // Lecture des données
  float tauxHumidite = dht.readHumidity();              // Lecture du taux d'humidité (en %)
  float temperatureEnCelsius = dht.readTemperature();   // Lecture de la température, exprimée en degrés Celsius

  // Vérification si données bien reçues
  if (isnan(tauxHumidite) || isnan(temperatureEnCelsius)) {
    Serial.println("Aucune valeur retournée par le DHT22. Est-il bien branché ?");
    delay(2000);
    return;         // Si aucune valeur n'a été reçue par l'Arduino, on attend 2 secondes, puis on redémarre la fonction loop()
  }

  // Calcul de la température ressentie
  float temperatureRessentieEnCelsius = dht.computeHeatIndex(temperatureEnCelsius, tauxHumidite, false); // Le "false" est là pour dire qu'on travaille en °C, et non en °F
  
  // Affichage des valeurs
  Serial.print("Humidité = "); Serial.print(tauxHumidite); Serial.println(" %");
  Serial.print("Température = "); Serial.print(temperatureEnCelsius); Serial.println(" °C");
  Serial.print("Température ressentie = "); Serial.print(temperatureRessentieEnCelsius); Serial.println(" °C");
  Serial.println();

  // Temporisation de 2 secondes (pour rappel : il ne faut pas essayer de faire plus d'1 lecture toutes les 2 secondes, avec le DHT22, selon le fabricant)
  delay(2000);
}