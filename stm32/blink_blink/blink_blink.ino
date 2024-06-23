
// the shipped blink example says the LED pin is PB1
#define LED_PIN PC13
void setup() {
  pinMode(LED_PIN, OUTPUT);
}
void loop() {
  digitalWrite(LED_PIN, LOW);  delay(20);
  digitalWrite(LED_PIN, HIGH); delay(200);
  digitalWrite(LED_PIN, LOW);  delay(20);
  digitalWrite(LED_PIN, HIGH); delay(200);
  digitalWrite(LED_PIN, LOW);  delay(200);
  digitalWrite(LED_PIN, HIGH); delay(1000);
}
