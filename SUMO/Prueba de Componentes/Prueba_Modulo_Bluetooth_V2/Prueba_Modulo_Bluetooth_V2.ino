#include <SoftwareSerial.h>   // Incluimos la librería  SoftwareSerial  
SoftwareSerial BT(10, 11);   // Definimos los pines RX y TX del Arduino conectados al Bluetooth
char dato;
int funcionamiento;

void setup()
{
  BT.begin(9600);       // Inicializamos el puerto serie BT (Para Modo AT 2)
  Serial.begin(9600);   // Inicializamos  el puerto serie
}

void loop() {
  switch (funcionamiento) {
    case 0:

      if (BT.available())   // Si llega un dato por el puerto BT se envía al monitor serial
      {
        //Serial.write(BT.read());
        dato = BT.read();

        Serial.print("Lenght:");
        Serial.println(strlen(BT.read()));
        Serial.print("Dato:");
        Serial.println(BT.read());

        if (BT.read() == 'h') {
          Serial.println("a");
          //delay(2000);
          funcionamiento = 1;
        }
        if (BT.read() == '2') {
          //delay(2000);
          funcionamiento = 2;
        }
        if (BT.read() == "3") {
          //delay(2000);
          funcionamiento = 3;
        }
      }
      break;

    case 1:
      Serial.println("adelante");
      break;
    case 2:
      Serial.println("atras");
      break;
    case 3:
      Serial.println("diagonal");
      break;

  }


}
