// TB6600 common anode + NPN transistor on PUL-/DIR-/ENA-
const uint8_t PIN_PUL = 2;
const uint8_t PIN_DIR = 4;
const uint8_t PIN_ENA = 5;

const float STEPS_PER_REV_MOTOR = 200.0;
const float MICROSTEP           = 4.0;   // matches DIP switches
float rpm                       = 2000;
bool  directionForward          = true;   // invert if wrong direction

const unsigned long PULSE_WIDTH_US = 10;
unsigned long stepInterval_us;
unsigned long lastStep_us = 0;

unsigned long calculateInterval(float rpm_val) {
  float sps = rpm_val * STEPS_PER_REV_MOTOR * MICROSTEP / 60.0;
  return (unsigned long)(1000000.0 / sps);
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_PUL, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_ENA, OUTPUT);

  digitalWrite(PIN_PUL, LOW);              // idle: transistor cut-off
  digitalWrite(PIN_ENA, LOW);              // opto off = driver active
  digitalWrite(PIN_DIR, directionForward ? HIGH : LOW);
  delayMicroseconds(10);                   // DIR setup time

  stepInterval_us = calculateInterval(rpm);
  lastStep_us = micros();
}

void loop() {
  unsigned long currentMicros = micros();
  if (currentMicros - lastStep_us >= stepInterval_us) {
    lastStep_us += stepInterval_us;

    digitalWrite(PIN_PUL, HIGH);           // conducting transistor = pulse
    delayMicroseconds(PULSE_WIDTH_US);
    Serial.println("OK");
    digitalWrite(PIN_PUL, LOW);
  }
  delay(5);
}
