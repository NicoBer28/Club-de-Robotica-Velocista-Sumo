#define MOTD_AD 2
#define MOTD_AT 4
#define MOTD_PWM 15
#define MOTI_AD 18
#define MOTI_AT 19
#define MOTI_PWM 21

void setup() {

  pinMode(MOTI_AD, OUTPUT);
  pinMode(MOTI_AT, OUTPUT);
  pinMode(MOTD_AD, OUTPUT);
  pinMode(MOTD_AT, OUTPUT);


}

void loop() {
  analogWrite(MOTD_PWM, 150);
  analogWrite(MOTI_PWM, 150);

  adelante();
  delay(1000);
  derecha();
  delay(1000);
  atras();
  delay(1000);
  izquierda();
  delay(1000);

}

void adelante() {
  digitalWrite(MOTI_AD, HIGH);
  digitalWrite(MOTD_AD, HIGH);
  digitalWrite(MOTI_AT, LOW);
  digitalWrite(MOTD_AT, LOW);
}

void atras() {
  digitalWrite(MOTI_AD, LOW);
  digitalWrite(MOTD_AD, LOW);
  digitalWrite(MOTI_AT, HIGH);
  digitalWrite(MOTD_AT, HIGH);
}

void derecha() {
  digitalWrite(MOTI_AD, HIGH);
  digitalWrite(MOTD_AD, LOW);
  digitalWrite(MOTI_AT, LOW);
  digitalWrite(MOTD_AT, HIGH);
}

void izquierda() {
  digitalWrite(MOTI_AD, LOW);
  digitalWrite(MOTD_AD, HIGH);
  digitalWrite(MOTI_AT, HIGH);
  digitalWrite(MOTD_AT, LOW);
}
