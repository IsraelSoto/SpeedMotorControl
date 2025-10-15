// ================================================================
//        CONTROL DE MOTOR DC CON DRIVER DUAL AT8236 (sin enable)
// ================================================================

// ---------- CONFIGURACIÓN DE PINES ----------
const byte encoderPinA = 2;  // Encoder canal A
const byte encoderPinB = 3;  // Encoder canal B

// Canal A del AT8236
const byte pinIN1 = 5;  // Entrada 1
const byte pinIN2 = 6;  // Entrada 2
//#define Vin A5 //Analog pin reading power supply voltage

// Canal B disponible si se desea agregar otro motor
// const byte IN1_B = 9;
// const byte IN2_B = 10;

// ---------- PARÁMETROS DEL SISTEMA ----------
const int pulsesPerRev = 11;       // Pulsos por vuelta del encoder
const float gearRatio = 30.0;      // Relación de engranaje 56-(205 RPM),30-(333 RPM), 19-(550 RPM)
const float supplyVoltage = 12.0;  // Voltaje de alimentación
const float Ts = 0.005;             // Tiempo de muestreo (s)
//const float TiempoTotal = 75.0;    // Duración total del experimento (s)
const float TiempoTotal = 3.0;    // Duración total del experimento (s)

// ---------- FILTRO DE VELOCIDAD -------------
const float alpha = 0.35;           // Filtro exponencial

// ---------- SEÑAL DE EXCITACIÓN ----------
const String excitationType = "cuadrada";  // "rampa", "cuadrada", "trapezoidal"

// Parámetros de la señal cuadrada
const int pwmMin = 0;
const int pwmMax = 6*255/12;            // Amplitud (0–255)
const unsigned int squarePeriod_ms = 3000;

// Parámetros de señal trapezoidal
const unsigned int rampTime_ms = 1000;
const unsigned int holdTime_ms = 1000;
const unsigned int cycleTime_ms = 2 * rampTime_ms + 2 * holdTime_ms;

// ---------- VARIABLES GLOBALES ----------
volatile long encoderCount = 0;
volatile int lastEncoded = 0;
long lastCount = 0;
float filteredVelocity = 0.0;
int pwmValue = 0;

volatile bool flag_muestreo = false;
unsigned long startMillis, startMicros;

// ================================================================
//                  CONFIGURACIÓN DEL TIMER1
// ================================================================
void setupTimer1(float Ts) {
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;

  // Frecuencia base de 16 MHz, prescaler 64
  unsigned long compareMatch = (unsigned long)(Ts * 16000000.0 / 64.0) - 1;
  OCR1A = compareMatch;
  TCCR1B |= (1 << WGM12);              // Modo CTC
  TCCR1B |= (1 << CS11) | (1 << CS10); // Prescaler 64
  TIMSK1 |= (1 << OCIE1A);             // Habilita interrupción
  interrupts();
}

// ================================================================
//                     INTERRUPCIONES
// ================================================================

// Timer1 → muestreo
ISR(TIMER1_COMPA_vect) {
  flag_muestreo = true;
}

// Encoder cuadratura
void updateEncoder() {
  int MSB = digitalRead(encoderPinA);
  int LSB = digitalRead(encoderPinB);
  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
    encoderCount++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
    encoderCount--;

  lastEncoded = encoded;
}

// ================================================================
//            GENERACIÓN DE LA SEÑAL DE EXCITACIÓN
// ================================================================
int generateExcitation(unsigned long t_ms) {
  if (excitationType == "rampa") {
    int pwm = map(t_ms % 5000, 0, 4999, pwmMin, pwmMax);
    return constrain(pwm, pwmMin, pwmMax);
  }

  if (excitationType == "cuadrada") {
    return ((t_ms / squarePeriod_ms) % 2 == 0) ? pwmMax : pwmMin;
  }

  if (excitationType == "trapezoidal") {
    unsigned int t_cycle = t_ms % cycleTime_ms;

    if (t_cycle < rampTime_ms) {
      return map(t_cycle, 0, rampTime_ms, pwmMin, pwmMax);
    } else if (t_cycle < rampTime_ms + holdTime_ms) {
      return pwmMax;
    } else if (t_cycle < 2 * rampTime_ms + holdTime_ms) {
      return map(t_cycle - rampTime_ms - holdTime_ms, 0, rampTime_ms, pwmMax, pwmMin);
    } else {
      return pwmMin;
    }
  }
  // --- NUEVA SEÑAL: ESCALONADA CRECIENTE ---
  if (excitationType == "escalonada") {
    // Incrementa 0.1 V por segundo hasta 12 V
    float tiempo_s = t_ms / 1000.0;
    float voltaje = floor(tiempo_s/3.0) * 0.5;
    if (voltaje > supplyVoltage) voltaje = supplyVoltage;

    // Convertir a PWM (0–255)
    int pwm = (int)((voltaje / supplyVoltage) * 255.0);
    return constrain(pwm, 0, 255);
  }


  return 128; // valor por defecto
}

// ================================================================
//            CONTROL DE MOTOR (PWM + DIRECCIÓN)
// ================================================================
/*
void driveMotor(int pwm) {
  pwm = constrain(pwm, -255, 255);

  if (pwm > 0) {
    analogWrite(IN1_A, pwm);
    digitalWrite(IN2_A, LOW);
  } else if (pwm < 0) {
    analogWrite(IN2_A, -pwm);
    digitalWrite(IN1_A, LOW);
  } else {
    digitalWrite(IN1_A, LOW);
    digitalWrite(IN2_A, LOW);
  }
}
*/
// ================================================================
//                         SETUP
// ================================================================
void setup() {
  // Encoder
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPinA), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), updateEncoder, CHANGE);

  // Puente H
//  pinMode(IN1_A, OUTPUT);
//  pinMode(IN2_A, OUTPUT);
pinMode(pinIN1, OUTPUT);
  pinMode(pinIN2, OUTPUT);
  //Analog input pin
  //pinMode(Vin,INPUT); //Initialize as input

  // Timer y comunicación
  setupTimer1(Ts);
  Serial.begin(115200);
  startMillis = millis();
  startMicros = micros();
}

// ================================================================
//                          LOOP
// ================================================================
void loop() {
  unsigned long elapsedMillis = millis() - startMillis;

  if (elapsedMillis < (unsigned long)(TiempoTotal * 1000.0)) {
    if (flag_muestreo) {
      flag_muestreo = false;

      unsigned long t_us = micros() - startMicros;
      long currentCount = encoderCount;

      // Posición (rev)
      float positionRev = (float)currentCount / (pulsesPerRev * 4.0 * gearRatio);

      // Velocidad instantánea (RPM)
      float deltaRev = (float)(currentCount - lastCount) / (pulsesPerRev * 4.0 * gearRatio);
      float velocityRPM = deltaRev * (1.0 / Ts) * 60.0;

      // Filtro
      filteredVelocity = alpha * velocityRPM + (1.0 - alpha) * filteredVelocity;

      // Señal de excitación
      pwmValue = generateExcitation(elapsedMillis);
      //driveMotor(pwmValue);
      
      // --- Aplicar PWM directo a IN1 (IN2 en bajo) ---
      analogWrite(pinIN1, pwmValue);
      analogWrite(pinIN2, 0);


      // Voltaje aplicado estimado
      float voltage = (abs(pwmValue) / 255.0) * supplyVoltage;
      // Voltaje aplicado leido
      //float volt=analogRead(Vin)*0.05371;

      // Salida serial
      Serial.print(t_us / 1e6, 3); Serial.print(",");
      Serial.print(positionRev, 2); Serial.print(",");
      Serial.print(velocityRPM, 2); Serial.print(",");
      Serial.print(filteredVelocity, 2); Serial.print(",");
      Serial.println(voltage, 2);

      lastCount = currentCount;
    }
  } else {
    // Parar al cumplir tiempo total
    analogWrite(pinIN1, 0);
    analogWrite(pinIN2, 0);
    Serial.end();
    while (1);
  }
}
