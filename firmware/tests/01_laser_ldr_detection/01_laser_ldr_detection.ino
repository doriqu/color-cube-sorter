const int INT1 = 32;
const int INT2 = 33;

void setup() {
  Serial.begin(115200);
  pinMode(INT1, INPUT);
  pinMode(INT2, INPUT);
}

void loop() {
  float START;
  float END;
  START = analogRead(INT1);
  END = analogRead(INT2);
  Serial.print("START = ");
  Serial.println(START);
  Serial.print("END = ");
  Serial.println(END);
  delay(1000);
}
