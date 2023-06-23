#define SP_IZQ A0
#define SP_DER A1
#define SP_UMBRAL 500
#define CNY_NEGRO 1
#define CNY_BLANCO 0
#define MOTI_AD 4
#define MOTI_AT 5
#define MOTD_AD 8
#define MOTD_AT 9
#define MOTD_PWM 10
#define MOTI_PWM 11
#define ECHO_PIN 2  //TO DO PONER EL PIN CORRECTO
#define TRIG_PIN 3  //TO DO PONER EL PIN CORRECTO
#define BOTON 12

int colorIzq;
int colorDer;
float distancia;
int encendido = 0;

void setup() {

  pinMode(MOTI_AD, OUTPUT);
  pinMode(MOTI_AT, OUTPUT);
  pinMode(MOTD_AD, OUTPUT);
  pinMode(MOTD_AT, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BOTON, INPUT_PULLUP);
  Serial.begin(115200);

}

int leerCNY(int pin) {
  int medicion = analogRead(pin);
  // Serial.print("Valor CNY; ");
  //Serial.print(medicion);
  //Serial.print("\t Color: ");

  if (medicion > SP_UMBRAL) {
    // Serial.println("Negro");
    return CNY_NEGRO;
  } else {
    // Serial.println("Blanco");
    return CNY_BLANCO;
  }
}


int noCaerse() {
  // Si el ultrasonico detecta que esta cerca el otro robot que no haga esto
  if (colorIzq == CNY_BLANCO && colorDer == CNY_BLANCO) {
    Serial.print("Linea blanca detectada");
    analogWrite(MOTD_PWM, 255);
    analogWrite(MOTI_PWM, 255);

    digitalWrite(MOTI_AD, LOW);
    digitalWrite(MOTD_AD, LOW);
    digitalWrite(MOTI_AT, HIGH);
    digitalWrite(MOTD_AT, HIGH);
    delay(1500);
   /* analogWrite(MOTD_PWM, 102);
    analogWrite(MOTI_PWM, 102);
    digitalWrite(MOTI_AD, LOW);
    digitalWrite(MOTD_AD, HIGH);
    digitalWrite(MOTI_AT, HIGH);
    digitalWrite(MOTD_AT, LOW);
    delay(1000);*/
    return;
  }
}
int merodeando() {
  if (distancia > 80) { // CAMBIAR PARA LA COMPE


    analogWrite(MOTD_PWM, 255);
    analogWrite(MOTI_PWM, 50);   //125

    digitalWrite(MOTI_AD, HIGH); //high
    digitalWrite(MOTD_AD, HIGH);
    digitalWrite(MOTI_AT, LOW); //low
    digitalWrite(MOTD_AT, LOW);
    return;
  }
}

int buscar() {
  if (distancia < 80) { //CAMBIAR PARA LA COMPE

    analogWrite(MOTD_PWM, 255);
    analogWrite(MOTI_PWM, 255);

    digitalWrite(MOTI_AD, HIGH);
    digitalWrite(MOTD_AD, HIGH);
    digitalWrite(MOTI_AT, LOW);
    digitalWrite(MOTD_AT, LOW);
    return;
  }
}

float medicionUltrasonico() {
  long duracion;
  int distanciaUltrasonico;

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  duracion = pulseIn(ECHO_PIN, HIGH);
  distanciaUltrasonico = duracion * 0.0340 / 2;

  Serial.print("Valor Ultrasonico");
  Serial.print(distanciaUltrasonico);


  return distanciaUltrasonico;
}

void loop() {

  if (digitalRead(BOTON) == LOW && encendido == 0) {
    delay(5000);
    encendido = 1;
  }
  while (encendido == 1) {
    colorIzq = leerCNY(SP_IZQ);
    colorDer = leerCNY(SP_DER);
    distancia = medicionUltrasonico();
    merodeando();
    buscar();
    noCaerse();
  }
}
