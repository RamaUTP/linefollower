#define LedQTR     13
#define LedMotor   12
#define PINBUZZER  10
#define PINBOTON   2

#define AIN1 8    // pin 1 de dirección del Motor Izquierdo
#define AIN2 7    // pin 2 de dirección del Motor Izquierdo
#define PWMA 11    // pin PWM del Motor Izquierdo

#define BIN1 4    // pin 1 de dirección del Motor Derecho
#define BIN2 5    // pin 2 de dirección del Motor Derecho
#define PWMB 3    // pin PWM del Motor Derecho
#define Umbral 720

int base = 255;
float Kprop = 0.5;
float Kderiv = 50.0;
float Kinte = 0.0;
int pos;

int setpoint = 0;
int last_error = 0;
int pot_limite = 300;
int GiroRetorno = 351;
int Freno = 350;
int integrative = 0;

int v_s_min[8] = {1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023};
int v_s_max[8] = {0, 0, 0, 0, 0, 0, 0, 0};
volatile int s_p[8];
boolean online;

int l_pos;

void setup() {
  Serial.begin(115200);
  pinMode(LedQTR, OUTPUT);
  pinMode(LedMotor, OUTPUT);
 
  digitalWrite(LedQTR, LOW);
  digitalWrite(LedMotor, LOW);
  Serial.println("hola");
  Motores(0, 0);
  //WaitBoton();
  digitalWrite(LedQTR, HIGH);
  Peripherals_init();

  beep();
  //delay(1000);
  calibracion();

  digitalWrite(LedQTR, LOW);
  tone(PINBUZZER, 1500, 50);
  delay(70);
  tone(PINBUZZER, 1500, 50);
  delay(70);
  //while(!digitalRead(PINBOTON));

  while(!digitalRead(PINBOTON));
  MotorIz(100);
  MotorDe(100);
  delay(50);
  MotorIz(150);
  MotorDe(150);
  delay(50);
  MotorIz(200);
  MotorDe(200);
  delay(50);
  digitalWrite(LedQTR, HIGH);
  //digitalWrite(LedMotor, HIGH);
}

void loop() {
  int line_position = GetPos();
  int Correction_power = PIDLambo(line_position, Kprop, Kderiv, Kinte);
  Motores(base + Correction_power, base - Correction_power);
  if(Correction_power==0){digitalWrite(LedMotor, HIGH);}
  else{digitalWrite(LedMotor, LOW);}
  if(line_position >= Freno) {Motores(255,-250);delay(1);}
  if(line_position <= -Freno) {Motores(-250,255);delay(1);}
  //Serial.print(line_position);
  //Serial.print("\t");
  //Serial.println(Correction_power);
}

void Peripherals_init() {
  pinMode(PINBOTON, INPUT);
  pinMode(PINBUZZER, OUTPUT);
}

void WaitBoton() {
  while (!digitalRead(PINBOTON));
  tone(PINBUZZER, 2000, 100);
}

void beep() {
  tone(PINBUZZER, 2000, 100);
  delay(200);
}

int PIDLambo(int POS, float Kp, float Kd, float Ki) {
  int error = POS - setpoint;
  int derivative = error - last_error;
  integrative += error;
  last_error = error;
  int pot_giro = (error * Kp + derivative * Kd + integrative * Ki);
  if (pot_giro > pot_limite) {
    pot_giro = pot_limite;
  } else if (pot_giro < -pot_limite) {
    pot_giro = -pot_limite;
  }
  return pot_giro;
}

void calibracion() {

  int v_s[8];

  for (int j = 0; j < 100; j++) {
    delay(10);
    v_s[7] = analogRead(A7);
    v_s[6] = analogRead(A6);
    v_s[5] = analogRead(A5);
    v_s[4] = analogRead(A4);
    v_s[3] = analogRead(A3);
    v_s[2] = analogRead(A2);
    v_s[1] = analogRead(A1);
    v_s[0] = analogRead(A0);

    for (int i = 0; i < 8; i++) {

      Serial.print(v_s[i]);
      Serial.print("\t");

    }
    Serial.println();

    for (int i = 0; i < 8; i++) {
      if (v_s[i] < v_s_min[i]) {
        v_s_min[i] = v_s[i];
      }
    }


    for (int i = 0; i < 8; i++) {
      if (v_s[i] > v_s_max[i]) {
        v_s_max[i] = v_s[i];
      }
    }
  }

  beep();
  beep();

  Serial.println();
  Serial.print("Mínimos ");
  Serial.print("\t");

  for (int i = 0; i < 8; i++) {

    Serial.print(v_s_min[i]);
    Serial.print("\t");

  }
  Serial.println();
  Serial.print("Máximos ");
  Serial.print("\t");

  for (int i = 0; i < 8; i++) {

    Serial.print(v_s_max[i]);
    Serial.print("\t");

  }
  Serial.println();
  Serial.println();
  Serial.println();
}

void readSensors() {
  int s[8];
  
  s[0] = analogRead(A0);
  s[1] = analogRead(A1);
  s[2] = analogRead(A2);
  s[3] = analogRead(A3);
  s[4] = analogRead(A4);
  s[5] = analogRead(A5);
  s[6] = analogRead(A6);
  s[7] = analogRead(A7);

  for (int i = 0; i < 8; i++) {
    /*if (s[i] < v_s_min[i]) {
      s[i] = v_s_min[i];
    }

    if (s[i] > v_s_max[i]) {
      s[i] = v_s_max[i];
    }*/
    if (s[i] < Umbral) {
      s_p[i] = 0;
    }

    if (s[i] >= Umbral) {
      s_p[i] = 100;
    }
    //s_p[i] = map(s[i], v_s_min[i], v_s_max[i], 0, 100); // Ajustado para línea negra sobre fondo blanco
   }

  volatile int sum = s_p[0] + s_p[1] + s_p[2] + s_p[3] + s_p[4] + s_p[5] + s_p[6] + s_p[7];
  if (sum >= 100) {
    online = 1;

  } else {
    online = 0;
    //sum = 100;
  }

  //if (online) {
    //for (int i = 0; i < 8; i++) {Serial.print(s_p[i]);Serial.print("\t");}
    //Serial.println();
  //}
}

int GetPos() {
  readSensors();
  int prom = -3.5 * s_p[0] - 2.5 * s_p[1] - 1.5 * s_p[2] - 0.5 * s_p[3] + 0.5 * s_p[4] + 1.5 * s_p[5] + 2.5 * s_p[6] + 3.5 * s_p[7];
  int sum = s_p[0] + s_p[1] + s_p[2] + s_p[3] + s_p[4] + s_p[5] + s_p[6] + s_p[7];

  if (online) {
    pos = int(100.0 * prom / sum);
  } 
  else {
    if (l_pos <= -350) {
      pos = -GiroRetorno;
    }
    if (l_pos >= 350) {
      pos = GiroRetorno;
    }
  }
  l_pos = pos;
  return pos;
}

void TB6612FNG_init() {
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
}

void MotorIz(int value) {
  if (value >= 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    value *= -1;
  }
  analogWrite(PWMA, value);
}

void MotorDe(int value) {
  if (value >= 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    value *= -1;
  }
  analogWrite(PWMB, value);
}

void Motores(int left, int right) {
  if(left < -255){left = -255;}
  if(left > 255){left = 255;}
  if(right < -255){right = -255;}
  if(right > 255){right = 255;}
  MotorIz(left);
  MotorDe(right);
}