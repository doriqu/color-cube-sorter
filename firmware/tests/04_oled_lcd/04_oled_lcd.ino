#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Initialisation de l'écran LCD (Adresse I2C souvent 0x27 ou 0x3F, 16 colonnes, 2 lignes)
LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n--- Debut du programme LCD ---");

  // Initialisation du bus I2C sur l'ESP32 (SDA = 21, SCL = 22)
  Wire.begin(21, 22);
  Serial.println("Bus I2C initialise (SDA=21, SCL=22)");

  // Initialisation de l'écran LCD
  lcd.init();                      
  lcd.backlight(); // Allumer le rétroéclairage
  Serial.println("LCD initialise et retroeclairage active");

  // Affichage sur la 1ère ligne
  lcd.setCursor(0, 0); // Colonne 0, Ligne 0
  lcd.print("CUBE ROUGE");

  // Affichage sur la 2ème ligne
  lcd.setCursor(0, 1); // Colonne 0, Ligne 1
  lcd.print("TRI\xE9"); 
  Serial.println("Texte envoye sur le LCD avec succes !");
}

void loop() {
  // Rien à faire ici pour un affichage fixe
}