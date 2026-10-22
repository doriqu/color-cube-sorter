// TB6600 anode commune + transistor NPN sur PUL-/DIR-/ENA-
const uint8_t PIN_PUL = 2;
const uint8_t PIN_DIR = 4;
const uint8_t PIN_ENA = 5;


const float STEPS_PER_REV_MOTOR = 200.0;
const float MICROSTEP           = 4.0;   // identique aux DIP switches
float rpm                       = 2000;
bool  sens                      = true;   // inverse si mauvais sens

const unsigned long PULSE_WIDTH_US = 10;
unsigned long stepInterval_us;
unsigned long lastStep_us = 0;

unsigned long calculerIntervalle(float tr_min) {
  float sps = tr_min * STEPS_PER_REV_MOTOR * MICROSTEP / 60.0;
  return (unsigned long)(1000000.0 / sps);
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_PUL, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_ENA, OUTPUT);

  digitalWrite(PIN_PUL, LOW);              // repos : transistor bloqué
  digitalWrite(PIN_ENA, LOW);              // opto éteint = driver actif
  digitalWrite(PIN_DIR, sens ? HIGH : LOW);
  delayMicroseconds(10);                   // établissement DIR

  stepInterval_us = calculerIntervalle(rpm);
  lastStep_us = micros();
}

void loop() {
  unsigned long maintenant = micros();
  if (maintenant - lastStep_us >= stepInterval_us) {
    lastStep_us += stepInterval_us;

    digitalWrite(PIN_PUL, HIGH);           // transistor passant = impulsion
    delayMicroseconds(PULSE_WIDTH_US);
    Serial.println("OK");
    digitalWrite(PIN_PUL, LOW);
  }
  delay(5);
}