#include <Servo.h>

Servo monServo;          // Crée l'objet pour contrôler le servomoteur
const int pinServo = 34;  // Broche à laquelle est connecté le signal du servo

void setup() {
  monServo.attach(pinServo);  // Associe le servomoteur à la broche 9
}

void loop() {
  // 1. Position à 0 degré
  monServo.write(0);
  delay(2000); // Attend 2 secondes

  // 2. Position à 90 degrés
  monServo.write(90);
  delay(2000); // Attend 2 secondes

  // 3. Position à 180 degrés
  monServo.write(180);
  delay(2000); // Attend 2 secondes
  
  //4. Position à degré
  monServo.write(270); 
  delay(2000);
}