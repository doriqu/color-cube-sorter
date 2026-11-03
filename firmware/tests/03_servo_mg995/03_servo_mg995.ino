#include <Servo.h>

Servo myServo;           // Creates the object to control the servo motor
const int servoPin = 34; // Pin to which the servo signal is connected

void setup() {
  myServo.attach(servoPin); // Attaches the servo motor to the pin
}

void loop() {
  // 1. Position at 0 degrees
  myServo.write(0);
  delay(2000); // Wait 2 seconds

  // 2. Position at 90 degrees
  myServo.write(90);
  delay(2000); // Wait 2 seconds

  // 3. Position at 180 degrees
  myServo.write(180);
  delay(2000); // Wait 2 seconds
  
  // 4. Position at degree
  myServo.write(270); 
  delay(2000);
}
