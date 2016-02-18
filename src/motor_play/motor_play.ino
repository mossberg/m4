const int pin = 7;
void setup() {
  // put your setup code here, to run once:
  pinMode(pin, OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(pin, LOW);
  delay(250);
  digitalWrite(pin, HIGH);
  delay(250);
}
