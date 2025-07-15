/*  CARRO SEGUIDOR DE LINEA 5 SENSOSRES PID
    FECHA: 11/07/2025
    #MGU2508ARDUINO
    NOTA: ESTE CODIGO SOLO FUNCIONA CON SENSORES INFRAROJOS DIGITALES.
*/

/* PINES SENSORES INFRAROJOS */
#define PIN_IR1   A0
#define PIN_IR2   A1
#define PIN_IR3   A2
#define PIN_IR4   A3
#define PIN_IR5   A4

/* PINES MODULO PUENTE H (L298n) */
#define PIN_IN1   4
#define PIN_IN2   5
#define PIN_IN3   8
#define PIN_IN4   9
#define PIN_ENA   3
#define PIN_ENB   10

/* CONFIGURACION DE VELOCIDADES

   EL NIVEL DE VELOCIDA DEPENDE DE LA ENERGIA DE LAS BATERIAS
   ASI COMO DEL NIVEL DE VOLTAJE DE LAS MISMAS.

   LA VELOIDAD VA EN UN RANGO DE 0 A 255. DONDE 255 EL NIVEL MAXIMO DE VELOCIDAD.

   ¡IMPORTANTE!
   COLOCAR UN NIVEL DEL VELOCIDAD MAXIMA PARA PREVENIR UNA BRUSCA CORRECCION
   DEL ERROR REALIZADO POR EL PID HACIENDO QUE SE SALGA DE FORMA AGRESIVA DE LA
   LINEA.
*/
#define BASE_VEL1 130
#define BASE_VEL2 130
#define MAX_VEL1  (BASE_VEL1+60)
#define MAX_VEL2  (BASE_VEL2+60)

/* PARAMETROS DEL PID
  KP: PARAMETRO PROPORCIONAL
  KD: PARAMETRO DERIVATIVO
*/

float Kp = 0.065f;
float Kd = 1.3f;

int P = 0;
int D = 0;
int error = 0;
int last_error = 0;

/* FUNCIONES */
int16_t CalculatePosition(int *values, unsigned int sensor_count)
{
  uint32_t Wsum(0U);
  uint32_t sum(0U);

  for(int i(0); i < sensor_count; i++){
    Wsum += values[i] * (i + 1)*1000;
    sum += values[i];
  }

  if(sum > 0 && sum != 5) {
    return static_cast<int16_t> (Wsum / sum);
  }
  
  return -1;
}

void MoveForward() {
  digitalWrite(PIN_IN1, HIGH);
  digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, HIGH);
  digitalWrite(PIN_IN4, LOW);
}

void MoveStop() {
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN4, LOW);

  analogWrite(PIN_ENA, 0);
  analogWrite(PIN_ENB, 0);
}

void PIDcontrol()
{
  int irs[5];
  int irposition = -1;
  int pid = 0;
  int ena = 0;
  int enb = 0;

  irs[0] = digitalRead(PIN_IR1);
  irs[1] = digitalRead(PIN_IR2);
  irs[2] = digitalRead(PIN_IR3);
  irs[3] = digitalRead(PIN_IR4);
  irs[4] = digitalRead(PIN_IR5);

  irposition = CalculatePosition(irs, 5);

  if (irposition != -1)
    error = 3000 - irposition;

  P = error;
  D = error - last_error;
  last_error = error;

  pid = P*Kp + D*Kd;

  ena = BASE_VEL1 + pid;
  enb = BASE_VEL2 - pid;

  ena = (ena > MAX_VEL1) ? MAX_VEL1 : ena;
  enb = (enb > MAX_VEL2) ? MAX_VEL2 : enb;

  ena = (ena < 0) ? 0 : ena;
  enb = (enb < 0) ? 0 : enb;

  analogWrite(PIN_ENA, ena);
  analogWrite(PIN_ENB, enb);
  
}

void setup()
{
  delay(100);
  // Configuramos los pines del puente H
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);
  pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_ENB, OUTPUT);

  // Configurar pines del los sensores IR
  pinMode(PIN_IR1, INPUT);
  pinMode(PIN_IR2, INPUT);
  pinMode(PIN_IR3, INPUT);
  pinMode(PIN_IR4, INPUT);
  pinMode(PIN_IR5, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  digitalWrite(LED_BUILTIN, HIGH);
  MoveForward();
  
  analogWrite(PIN_ENA, 0);
  analogWrite(PIN_ENB, 0);
}

void loop()
{
  PIDcontrol();
  // Espera 10ms
  delay(10);
}
