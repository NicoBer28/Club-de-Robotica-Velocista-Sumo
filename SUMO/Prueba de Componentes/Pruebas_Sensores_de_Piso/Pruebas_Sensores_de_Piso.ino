//Prueba Lectura CNY70


#define SP_PIN A0
#define SP_UMBRAL 500
#define CNY_NEGRO 1
#define CNY_BLANCO 0

int leerCNY(int pin) {
  int medicion = analogRead(pin);
  Serial.print("Valor CNY; ");
  Serial.print(medicion);
  Serial.print("\t Color: ");

  if (medicion > SP_UMBRAL) {
    Serial.println("Negro");
    return CNY_NEGRO;
  } else {
    Serial.println("Blanco");
    return CNY_BLANCO;
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Prueba CNY70");
}

void loop() {

  Serial.println(leerCNY(SP_PIN));
  delay(100);

}
