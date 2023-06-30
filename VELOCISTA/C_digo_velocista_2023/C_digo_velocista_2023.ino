//AUTOR: NICOLAS BERGERMAN
//Código Velocista Dashito


//SENSORES
#define SP_PIN1 A1//20
#define SP_PIN2 A2//21
#define SP_PIN3 A3//22
#define SP_PIN4 A4//23
#define SP_PIN5 A5//24
#define SP_PIN6 A6//25
#define SP_PIN7 A7//26
//MOTORES
#define MOTD_AD 5
#define MOTD_AT 4
#define MOTD_PWM 3
#define STBY     6
#define MOTI_AD 7
#define MOTI_AT 8
#define MOTI_PWM 9
//SWITCH CASE
#define MODO_0 0
#define MODO_1 1
#define MODO_2 2
#define MODO_3 3

#define TRUE 0
#define FALSE 1
//BOTONES
#define PIN_BOTON1 12
#define PIN_BOTON2 11
//LEDS
#define PIN_LED1 10
#define PIN_LED2 2
//CANTIDAD DE SENSORES
#define cant_sens 7

#define BAUD_RATE 9600


int max_lectura[cant_sens], min_lectura[cant_sens], sens_map[cant_sens];
int medicionSP[cant_sens];
int i;

int proporcional;
int integral;
int derivada;
int p_anterior;
float kProporcional;
float kIntegral = 0;
float kDerivada;
float error;
float a;
float b;
float c;
int estado;

int velocidadMax;

int suma_mapeados;

int lecturaBoton1, lecturaBoton2;
int estadoBoton1;
int estadoBoton2;


float ms;
float seg;

int k;

int empieza;
int seFue;
int ultValor;
int sumador;

int veloci;
int veloci2;
float pRecta;
float dRecta;
float pCurva;
float dCurva;

void setup() {

  TCCR2B = TCCR2B & B11111000 | B00000001; // for PWM frequency of 31372.55 Hz
  TCCR1B = TCCR1B & B11111000 | B00000001; // for PWM frequency of 31372.55 Hz

  pinMode(MOTD_AD, OUTPUT);
  pinMode(MOTD_AT, OUTPUT);
  pinMode(MOTD_PWM, OUTPUT);
  pinMode(MOTI_AD, OUTPUT);
  pinMode(STBY, OUTPUT);
  pinMode(MOTI_AT, OUTPUT);
  pinMode(MOTI_PWM, OUTPUT);

  pinMode(PIN_BOTON1, INPUT_PULLUP);
  pinMode(PIN_BOTON2, INPUT_PULLUP);

  pinMode(PIN_LED1, OUTPUT);
  pinMode(PIN_LED2, OUTPUT);
  digitalWrite(STBY, HIGH);

  Serial.begin(BAUD_RATE);

  // Timer1.initialize(1000);
  // Timer1.attachInterrupt(timerFun);

  //ESTOS VALORES TIENEN QUE EMPEZAR EN UN VALOR ALTO PARA
  //QUE SE PUEDA OBTENER CORRECTAMENTE EL VALOR DE LECTURA MINIMA
  min_lectura[0] = 1000;
  min_lectura[1] = 1000;
  min_lectura[2] = 1000;
  min_lectura[3] = 1000;
  min_lectura[4] = 1000;
  min_lectura[5] = 1000;
  min_lectura[6] = 1000;

}
void loop() {

  switch (estado)
  {
    case MODO_0:
      lecturaBoton1 = digitalRead(PIN_BOTON1);
      digitalWrite(PIN_LED1, HIGH);

      if (lecturaBoton1 == 0) {
        estado = MODO_1;
        digitalWrite(PIN_LED1, LOW);
        digitalWrite(PIN_LED2, HIGH);
      }

    case MODO_1:

      static boolean setup_direccion = false;
      if (setup_direccion == false) {  //tmb se puede escribir como !setup_direccion
        setup_direccion = true;
        digitalWrite(MOTD_AD, HIGH);
        digitalWrite(MOTI_AD, HIGH);
        digitalWrite(MOTD_AT, LOW);
        digitalWrite(MOTI_AT, LOW);
      }
      //LECTURA SENSORES
      medicionSP[0] = leerCNY(SP_PIN1);
      medicionSP[1] = leerCNY(SP_PIN2);
      medicionSP[2] = leerCNY(SP_PIN3);
      medicionSP[3] = leerCNY(SP_PIN4);
      medicionSP[4] = leerCNY(SP_PIN5);
      medicionSP[5] = leerCNY(SP_PIN6);
      medicionSP[6] = leerCNY(SP_PIN7);
      /* PRINT MEDICIONES SENSORES
                if (k > 6) {
                  k = 0;
                }
                Serial.print(k);
                Serial.print("   ");
                Serial.println(medicionSP[k]);
                k += 1;
      */
      //ESTA FUNCION SACA LOS VALORES MINIMO Y MAXIMOS DE CADA SENSOR
      Max_Min();


      lecturaBoton2 = digitalRead(PIN_BOTON2);

      if (lecturaBoton2 == 0 && empieza == 0) {
        empieza = 1;
      }

      if (lecturaBoton2 == 1 && empieza == 1) {

        /*   PRINT MINIMOS Y MAXIMOS
             for (int q = 0; q < cant_sens; q++) {

             Serial.println("minimo: ");
             Serial.println(min_lectura[q]);

             Serial.println("maximo: ");
             Serial.println(max_lectura[q]);
           }
        */
        //      digitalWrite(PIN_LED_CALIBRACION, LOW);

        digitalWrite(PIN_LED1, HIGH);
        digitalWrite(PIN_LED2, LOW);

        estado = MODO_2;
      }


      break;

    case MODO_2:
      //****************************************
      //            SE CAMBIA ACÁ
      //****************************************

      lecturaBoton1 = digitalRead(PIN_BOTON1);
      lecturaBoton2 = digitalRead(PIN_BOTON2);

      if (lecturaBoton1 == 0 && estadoBoton1 == 0) {
        estadoBoton1 = 1;
      }
      if (lecturaBoton1 == 1 && estadoBoton1 == 1) {    //MODO RAPIDO - NEGRO
        veloci = 230; //VELOCIDAD RECTA
        veloci2 = 245;//VELOCIDAD CURVA
        pRecta = 0.3; //
        dRecta = 3.6;//
        pCurva = 0.3;//
        dCurva = 3.6;//
        estado = MODO_3;
      }
      if (lecturaBoton2 == 0 && estadoBoton2 == 0) {
        estadoBoton2 = 1;
      }
      if (lecturaBoton2 == 1 && estadoBoton2 == 1) {    //MODO LENTO - BLANCO
        veloci = 150; //VELOCIDAD RECTA
        veloci2 = 150;  //VELOCIDAD CURVA
        pRecta = 0.3; //
        dRecta = 3.6;//
        pCurva = 0.3;//
        dCurva = 3.6;//
        estado = MODO_3;
      }
      break;

    case MODO_3:
      //LECTURA SENSORES
      medicionSP[0] = leerCNY(SP_PIN1);
      medicionSP[1] = leerCNY(SP_PIN2);
      medicionSP[2] = leerCNY(SP_PIN3);
      medicionSP[3] = leerCNY(SP_PIN4);
      medicionSP[4] = leerCNY(SP_PIN5);
      medicionSP[5] = leerCNY(SP_PIN6);
      medicionSP[6] = leerCNY(SP_PIN7);

      sens_map[0] = mapeado(min_lectura[0], max_lectura[0], medicionSP[0]);
      sens_map[1] = mapeado(min_lectura[1], max_lectura[1], medicionSP[1]);
      sens_map[2] = mapeado(min_lectura[2], max_lectura[2], medicionSP[2]);
      sens_map[3] = mapeado(min_lectura[3], max_lectura[3], medicionSP[3]);
      sens_map[4] = mapeado(min_lectura[4], max_lectura[4], medicionSP[4]);
      sens_map[5] = mapeado(min_lectura[5], max_lectura[5], medicionSP[5]);
      sens_map[6] = mapeado(min_lectura[6], max_lectura[6], medicionSP[6]);

      //LINEA BLANCA FONDO NEGRO: DEJAR LA FUNCION
      //LINEA NEGRA FONDO BLANCO: COMENTAR LA FUNCION
      inversion();

      //SI SE VA DE LA PISTA, LA FUNCION DEVUELVE 1, SINO DEVUELVE 0
      seFue = franBot();

      //POSICION DEL ROBOT RESPECTO A LA LINEA, DE -1000 A 1000
      proporcional = sacaLineas();
      // Serial.println(proporcional);
      //------
      //PID
      integral = integral + proporcional;

      if ((proporcional * integral) < 0) {
        integral = 0;
      } // corrige el overshooting - integral windup

      derivada = proporcional - p_anterior;
      p_anterior = proporcional;
      //------
      
      //-----------
      //SI SE VA DE LA PISTA, PARA DONDE GIRA
      if (seFue == 1) {
        Serial.println("aribot");

        if (ultValor == 0) {
          Serial.println("izq");
          analogWrite(MOTI_PWM, 0);
          analogWrite(MOTD_PWM, 255);
        }
        if (ultValor == 1) {
          Serial.println("der");
          analogWrite(MOTD_PWM, 0);
          analogWrite(MOTI_PWM, 255);
        }
      } else if (proporcional < 0) {
        ultValor = 1;
      } else {
        ultValor = 0;
      }
      //-----------

      if (proporcional > -50 && proporcional < 50) {
        kProporcional = pRecta;
        kDerivada = dRecta;
        velocidadMax = veloci;
        digitalWrite(PIN_LED2, HIGH);

      }
      else {
        digitalWrite(PIN_LED2, LOW);
        velocidadMax = veloci2;

        kProporcional = pCurva;
        kDerivada = dCurva;
      }

      a = kProporcional * proporcional;
      b = kIntegral * integral;
      c = kDerivada * derivada;
      error = a + b + c;
      //error = (kProporcional * proporcional) + (kIntegral * integral) + (kDerivada * derivada);

      //error = ((kProporcional * proporcional) + (kDerivada * derivada));
      if (seFue == 0) {

        if (error  <= 0) {
          seguirLinea(MOTD_PWM, MOTI_PWM, error);
          //Serial.println("If 1");
        }
        if (error > 0) { //DOBLAR A LA IZQUIERDA, IZ QUIER DA
          seguirLinea(MOTI_PWM, MOTD_PWM, error);
          //Serial.println("If 2");
        }
      }
      
      break;
  }
}

void seguirLinea(int pin1, int pin2, float _error) {
  float cambio;

  cambio = velocidadMax - abs(_error);
  /*
    if (cambio >= 0) {
      digitalWrite(mot1A, HIGH);
      digitalWrite(mot2A, HIGH);
      digitalWrite(mot1B, LOW);
      digitalWrite(mot2B, LOW);
    }
    if (cambio < 0) {

      digitalWrite(mot1A, LOW);
      digitalWrite(mot2A, HIGH);
      digitalWrite(mot1B, HIGH);
      digitalWrite(mot2B, LOW);
    }
  */
  if (cambio < 0) {
    cambio = 0;
  }
  //Serial.println(cambio);
  analogWrite(pin1, abs(cambio));
  analogWrite(pin2, velocidadMax);
}


int leerCNY(int pin) {
  int medicion = analogRead(pin);

  return medicion;
}



void Max_Min() {

  for (i = 0; i < cant_sens; i++) {

    if (medicionSP[i] > max_lectura[i]) {
      max_lectura[i] = medicionSP[i];
    }

    if (medicionSP[i] < min_lectura[i]) {
      min_lectura[i] = medicionSP[i];
    }
  }
}

int mapeado(int minimo, int maximo, int medicion) {

  int mapeado_final;

  if (medicion < minimo) {
    medicion = minimo;
  }
  if (medicion > maximo) {
    medicion = maximo;
  }

  mapeado_final = map(medicion, minimo, maximo, 0, 1000);

  return mapeado_final;
}

void inversion() {
  sumador = 0;
  for (i = 0; i < cant_sens; i++) {
    sens_map[i] = 1000 - sens_map[i];
    //Serial.println(sens_map[i]);
  }
}

int sacaLineas() {
  long dividendo = 0;
  long suma = 0;
  int linea;
  for (i = 0; i < cant_sens; i++) {
    dividendo += sens_map[i] * (i + 1);
    suma += sens_map[i];
  }
  if (suma > 0) {
    linea = (dividendo * 1000) / suma;
  }

  if (linea < 1000) {
    linea = 1000;
  }

  if (linea > 7000) {
    linea = 7000;
  }

  linea = map(linea, 1000, 7000, -1000, 1000);
  //Serial.println(linea);

  return linea;
}


int franBot() {
  //  int sumador;
  int afuera;

  for (int l = 0; l < cant_sens; l++) {
    if (sens_map[l] < 100) {
      sumador += 1;
    }
  }
  //Serial.println(sumador);

  if (sumador == 7) {
    afuera = 1;
    sumador = 0;
  } else {
    afuera = 0;
  }
  return afuera;

}
