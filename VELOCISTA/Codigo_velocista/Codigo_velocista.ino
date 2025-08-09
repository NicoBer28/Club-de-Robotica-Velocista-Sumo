//CREADOR: NICOLAS BERGERMAN
//NTEGRANTES DEL GRUPO: NICOLAS BERGERMAN, FRANCISCO MOMBELLI, SANTIAGO RAPETTI, FLORENCIA GRILLO, SANTIAGO EULMESEKIAN
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

//BOTONES
#define PIN_BOTON1 12
#define PIN_BOTON2 11
//LEDS
#define PIN_LED1 10
#define PIN_LED2 2
//CANTIDAD DE SENSORES
#define CANT_SENS 7

#define BAUD_RATE 9600

#define LINEA_POS_MIN (1 * 1000)
#define LINEA_POS_MAX (CANT_SENS * 1000)

//--------------------------------------------------------------------------------------------------------------------

                        //****************************************
                        //************ SE CAMBIA ACÁ *************
                        //****************************************

//---------------MODO LENTO-------------//
#define VEL_RECTA_MAX_LENTO 150          //velocidad maxima en recta para el modo lento
#define VEL_CURVA_MAX_LENTO 100         //velocidad maxima en curva para el modo lento
#define VEL_RECTA_MIN_LENTO 0           //velocidad minima en recta para el modo lento
#define VEL_CURVA_MIN_LENTO 0           //velocidad minima en curva para el modo lento
#define P_RECTA_LENTO 0.02               //constante de proporcional en recta para el modo lento
#define D_RECTA_LENTO 0.02               //constante de derivada en recta para el modo lento
#define I_RECTA_LENTO 0                 //constante de integral en recta para el modo lento
#define P_CURVA_LENTO 0.1               //constante de proporcional en curva para el modo lento
#define D_CURVA_LENTO 0.1        //constante de derivada en curva para el modo lento
#define I_CURVA_LENTO 0                 //constante de integral en curva para el modo lento

//---------------MODO RAPIDO------------//
#define VEL_RECTA_MAX_RAPIDO 200        //velocidad maxima en recta para el modo rapido
#define VEL_CURVA_MAX_RAPIDO 135        //velocidad maxima en curva para el modo rapido
#define VEL_RECTA_MIN_RAPIDO 0          //velocidad minima en recta para el modo rapido
#define VEL_CURVA_MIN_RAPIDO 0          //velocidad minima en curva para el modo rapido
#define P_RECTA_RAPIDO 0.02             //constante de proporcional en recta para el modo rapido
#define D_RECTA_RAPIDO 0.02              //constante de derivada en recta para el modo rapido
#define I_RECTA_RAPIDO 0                //constante de integral en recta para el modo rapido
#define P_CURVA_RAPIDO 0.1              //constante de proporcional en curva para el modo rapido
#define D_CURVA_RAPIDO 0.1              //constante de derivada en curva para el modo rapido
#define I_CURVA_RAPIDO 0                //constante de integral en curva para el modo rapido

//---MODO RECTA/CURVA---//
#define LIM_RECTA_CURVA 250             //limite en valor absoluto del proporcional (-1000 a 1000) para considerar recta/curva
#define HISTERESIS_RECTA_CURVA 10       //desfasaje del proporicional para que no haya oscilamiento

//---FUNCION SEFUE---//
#define UMBRAL_SIN_LINEA 200            //umbral para determinar si se fue de la linea (va de 0 a 1000, menos de UMBRAL_SIN_LINEA se considera que se fue)
#define CANT_SIN_LINEA (CANT_SENS - 0) //cantidad de sensores necesarios que detecten < UMBRAL_SIN_LINEA para que se considere que el robot se fue de la pista
#define VEL_RUEDA_EXTERIOR 140          //velocidad de la rueda mas lejos de la linea, una vez que se fue
#define VEL_RUEDA_INTERIOR 40          //velocidad de la rueda mas cerca de la linea, una vez que se fue
#define RUEDA_INTERIOR_AD LOW           //HIGH para que la rueda interior gire hacia adelante, LOW para que gire hacia atras

#define RUEDA_INTERIOR_AT !RUEDA_INTERIOR_AD //no hace falta cambiarlo, con cambiar RUEDA_INTERIOR_AD es suficiente

//--------------------------------------------------------------------------------------------------------------------

int max_lectura[CANT_SENS], min_lectura[CANT_SENS], sens_map[CANT_SENS];
int medicionSP[CANT_SENS];

int estado = MODO_0;

struct ConfigModo {
    int velRectaMax;
    int velCurvaMax;
    int velRectaMin;
    int velCurvaMin;
    float pRecta;
    float dRecta;
    float iRecta;
    float pCurva;
    float dCurva;
    float iCurva;
  };

static const ConfigModo modoLento = {VEL_RECTA_MAX_LENTO, VEL_CURVA_MAX_LENTO, VEL_RECTA_MIN_LENTO, VEL_CURVA_MIN_LENTO, P_RECTA_LENTO, D_RECTA_LENTO, I_RECTA_LENTO, P_CURVA_LENTO, D_CURVA_LENTO, I_CURVA_LENTO};
static const ConfigModo modoRapido = {VEL_RECTA_MAX_RAPIDO, VEL_CURVA_MAX_RAPIDO, VEL_RECTA_MIN_RAPIDO, VEL_CURVA_MIN_RAPIDO, P_RECTA_RAPIDO, D_RECTA_RAPIDO, I_RECTA_RAPIDO, P_CURVA_RAPIDO, D_CURVA_RAPIDO, I_CURVA_RAPIDO};


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
  int seFue;
  int proporcional, derivada;
  static int integral = 0;
  static int p_anterior = 0;
  float kProporcional, kIntegral, kDerivada;
  float error;
  float errorProporcional, errorIntegral, errorDerivada;

  int velocidadMax, velocidadMin;
  static const ConfigModo* modoActual = nullptr;


  static int lecturaBoton1, lecturaBoton2;
  static int estadoBoton1 = 0, estadoBoton2 = 0;

  int k;

  static int empieza = 0;
  static int ultValor;

  bool enRecta = true;


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
      break;

    case MODO_1:

      static boolean setup_direccion = false;
      if (setup_direccion == false) {  //tmb se puede escribir como !setup_direccion
        setup_direccion = true;
        setMotores(HIGH, HIGH, LOW, LOW); //MOTD_AD - MOTI_AD - MOTD_AT - MOTI_AT
      }
      //LECTURA SENSORES
      medicionSP[0] = analogRead(SP_PIN1);
      medicionSP[1] = analogRead(SP_PIN2);
      medicionSP[2] = analogRead(SP_PIN3);
      medicionSP[3] = analogRead(SP_PIN4);
      medicionSP[4] = analogRead(SP_PIN5);
      medicionSP[5] = analogRead(SP_PIN6);
      medicionSP[6] = analogRead(SP_PIN7);
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
             for (int q = 0; q < CANT_SENS; q++) {

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

      lecturaBoton1 = digitalRead(PIN_BOTON1);
      lecturaBoton2 = digitalRead(PIN_BOTON2);

      if (lecturaBoton1 == 0 && estadoBoton1 == 0) {
        estadoBoton1 = 1;
      }
      if (lecturaBoton1 == 1 && estadoBoton1 == 1) {    //MODO LENTO - CERCA DEL LED
        modoActual = &modoLento;
        estado = MODO_3;
      }
      if (lecturaBoton2 == 0 && estadoBoton2 == 0) {
        estadoBoton2 = 1;
      }
      if (lecturaBoton2 == 1 && estadoBoton2 == 1) {    //MODO RAPIDO - LEJOS DEL LED
        modoActual = &modoRapido;
        estado = MODO_3;
      }
      break;

    case MODO_3:
      //LECTURA SENSORES
      medicionSP[0] = analogRead(SP_PIN1);
      medicionSP[1] = analogRead(SP_PIN2);
      medicionSP[2] = analogRead(SP_PIN3);
      medicionSP[3] = analogRead(SP_PIN4);
      medicionSP[4] = analogRead(SP_PIN5);
      medicionSP[5] = analogRead(SP_PIN6);
      medicionSP[6] = analogRead(SP_PIN7);

      sens_map[0] = mapeado(min_lectura[0], max_lectura[0], medicionSP[0]);
      sens_map[1] = mapeado(min_lectura[1], max_lectura[1], medicionSP[1]);
      sens_map[2] = mapeado(min_lectura[2], max_lectura[2], medicionSP[2]);
      sens_map[3] = mapeado(min_lectura[3], max_lectura[3], medicionSP[3]);
      sens_map[4] = mapeado(min_lectura[4], max_lectura[4], medicionSP[4]);
      sens_map[5] = mapeado(min_lectura[5], max_lectura[5], medicionSP[5]);
      sens_map[6] = mapeado(min_lectura[6], max_lectura[6], medicionSP[6]);

      //LINEA BLANCA FONDO NEGRO: DEJAR LA FUNCION
      //LINEA NEGRA FONDO BLANCO: COMENTAR LA FUNCION
      //inversion();

      //SI SE VA DE LA PISTA, LA FUNCION DEVUELVE 1, SINO DEVUELVE 0
      seFue = seFueFun();

      //POSICION DEL ROBOT RESPECTO A LA LINEA, DE -1000 A 1000
      proporcional = posicionRobot();
      //Serial.println(proporcional);

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
        //Serial.println("aribot");

        if (ultValor == 0) {
          //Serial.println("izq");
          setMotores(HIGH, RUEDA_INTERIOR_AD, LOW, RUEDA_INTERIOR_AT); //MOTD_AD - MOTI_AD - MOTD_AT - MOTI_AT

          analogWrite(MOTI_PWM, VEL_RUEDA_INTERIOR);
          analogWrite(MOTD_PWM, VEL_RUEDA_EXTERIOR);
        }
        if (ultValor == 1) {
          //Serial.println("der");
          setMotores(RUEDA_INTERIOR_AD, HIGH, RUEDA_INTERIOR_AT, LOW); //MOTD_AD - MOTI_AD - MOTD_AT - MOTI_AT
          analogWrite(MOTD_PWM, VEL_RUEDA_INTERIOR);
          analogWrite(MOTI_PWM, VEL_RUEDA_EXTERIOR);
        }
      } else {
        if (proporcional < 0) {
          ultValor = 1;
        }else{
          ultValor = 0;
        }
        setMotores(HIGH, HIGH, LOW, LOW); //MOTD_AD - MOTI_AD - MOTD_AT - MOTI_AT
      }
      //-----------

      if(enRecta && abs(proporcional) > LIM_RECTA_CURVA + HISTERESIS_RECTA_CURVA){
        enRecta = false;
      }else if(!enRecta && abs(proporcional) < LIM_RECTA_CURVA - HISTERESIS_RECTA_CURVA){
        enRecta = true;
      }
      if(enRecta) {
        //digitalWrite(PIN_LED2, HIGH);
        velocidadMax = modoActual->velRectaMax;
        velocidadMin = modoActual->velRectaMin;
        kProporcional = modoActual->pRecta;
        kDerivada = modoActual->dRecta;
        kIntegral = modoActual->iRecta;
      }
      else {
        //digitalWrite(PIN_LED2, LOW);
        velocidadMax = modoActual->velCurvaMax;
        velocidadMin = modoActual->velCurvaMin;
        kProporcional = modoActual->pCurva;
        kDerivada = modoActual->dCurva;
        kIntegral = modoActual->iCurva;
      }

      errorProporcional = kProporcional * proporcional;
      errorIntegral = kIntegral * integral;
      errorDerivada = kDerivada * derivada;
      error = errorProporcional + errorIntegral + errorDerivada;
      //error = (kProporcional * proporcional) + (kIntegral * integral) + (kDerivada * derivada);

      //error = ((kProporcional * proporcional) + (kDerivada * derivada));
      if (seFue == 0) {

        if (error  <= 0) {
          seguirLinea(MOTD_PWM, MOTI_PWM, error, velocidadMax, velocidadMin);
          //Serial.println("If 1");
        } else { //DOBLAR A LA IZQUIERDA, IZ QUIER DA
          seguirLinea(MOTI_PWM, MOTD_PWM, error, velocidadMax, velocidadMin);
          //Serial.println("If 2");
        }
      }

      break;
  }
}

void seguirLinea(int pin_lento, int pin_rapido, float _error, int velocidadMax, int velocidadMin) {
  int cambio;

  cambio = velocidadMax - (int)abs(_error);
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
  if (cambio < velocidadMin) {
    cambio = velocidadMin;
  }
  //Serial.println(cambio);
  analogWrite(pin_lento, abs(cambio));
  analogWrite(pin_rapido, velocidadMax);
}

void Max_Min() {

  for (int i = 0; i < CANT_SENS; i++) {

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
  for (int i = 0; i < CANT_SENS; i++) {
    sens_map[i] = 1000 - sens_map[i];
    //Serial.println(sens_map[i]);
  }
}

int posicionRobot() {
  long dividendo = 0;
  long suma = 0;
  int linea = 0;
  for (int i = 0; i < CANT_SENS; i++) {
    dividendo += sens_map[i] * (i + 1);
    suma += sens_map[i];
  }
  if (suma > 0) {
    linea = (dividendo * LINEA_POS_MIN) / suma;
  }

  if (linea < LINEA_POS_MIN) {
    linea = LINEA_POS_MIN;
  }

  if (linea > LINEA_POS_MAX) {
    linea = LINEA_POS_MAX;
  }

  linea = map(linea, LINEA_POS_MIN, LINEA_POS_MAX, -1000, 1000);
  //Serial.println(linea);

  return linea;
}


int seFueFun() {
  int cantSensoresFuera = 0;

  for (int i = 0; i < CANT_SENS; i++) {
    /*    Serial.print(l);
        Serial.print(": ");
        Serial.println(sens_map[l]);
    */
    if (sens_map[i] < UMBRAL_SIN_LINEA) {
      cantSensoresFuera++;
    }
  }
  //Serial.println(cantSensoresFuera);

  return (cantSensoresFuera >= CANT_SIN_LINEA);
}

void setMotores(bool motd_ad, bool moti_ad, bool motd_at, bool moti_at) {
  digitalWrite(MOTD_AD, motd_ad);
  digitalWrite(MOTI_AD, moti_ad);
  digitalWrite(MOTD_AT, motd_at);
  digitalWrite(MOTI_AT, moti_at);
}
