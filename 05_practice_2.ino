#define PIN_LED 7
unsigned int toggle = 1;

void setup()
{
  pinMode(PIN_LED, OUTPUT);
  Serial.begin(115200);
  while (!Serial) {;}
}

void loop()
{
  digitalWrite(PIN_LED, toggle);
  delay(1000);

  for (int i = 0; i< 11; i++) {
    toggle = toggle_state(toggle);
    digitalWrite(PIN_LED, toggle);
    toggle = !toggle;
    delay(100);
  }

  digitalWrite(PIN_LED, LOW);
  while (1) {}
}

int toggle_state(int toggle) {
    return toggle;
  }