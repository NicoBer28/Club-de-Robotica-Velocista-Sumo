void setup() {
  Serial.begin(115200);
}

void loop() {
  if (Serial.available())
    {
      char dato=Serial.read();
      Serial.println("Dato recibido: ");
      Serial.println(dato);
    }
}
