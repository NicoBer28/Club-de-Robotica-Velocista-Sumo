const int Trigger = 4; //Pin digital
const int Echo = 3; //Pin digital


void setup() {
  Serial.begin(115200);
  pinMode(Trigger, OUTPUT);
  pinMode(Echo, INPUT);
  digitalWrite(Trigger, LOW);
}

void loop() {
  long t;
  long d;
  digitalWrite(Trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trigger, LOW);
  t = pulseIn(Echo, HIGH);
  d = t/59;

  Serial.print("Distancia: ");
  Serial.println(d);
  delay(100);
}
