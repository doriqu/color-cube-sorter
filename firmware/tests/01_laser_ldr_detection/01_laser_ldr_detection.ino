const int INT1 = 32;
const int INT2 = 33;
void setup() {
  Serial.begin(115200);
  // put your setup code here, to run once:
  pinMode(INT1, INPUT);
  pinMode(INT2, INPUT);
}

void loop() {
  float DEBUT;
  float FIN;
  DEBUT = analogRead(INT1);
  FIN = analogRead(INT2);
  Serial.print("DEBUT = ");
  Serial.println(DEBUT);
  Serial.print("FIN = ");
  Serial.println(FIN);
  delay(1000);

}
