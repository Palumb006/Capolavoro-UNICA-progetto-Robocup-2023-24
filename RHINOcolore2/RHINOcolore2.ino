#include <QTRSensors.h>

#define speedLimit 220
#define rightSpeed 120
#define leftSpeed 120
#define NUM_SENSORS 8

#define S0_PIN_1 22
#define S1_PIN_1 24
#define S2_PIN_1 26
#define S3_PIN_1 28
#define OUT_PIN_1 30

#define S0_PIN_2 23
#define S1_PIN_2 25
#define S2_PIN_2 27
#define S3_PIN_2 29
#define OUT_PIN_2 31

int red_1, green_1, blue_1, clear_1;
int red_2, green_2, blue_2, clear_2;

QTRSensorsRC qtrrc((unsigned char[]){
                     A0, A1, A2, A3, A4, A5, A6, A7  // SENSOR PIN NUMBERS
                   },
                   NUM_SENSORS, 2500, 2);
unsigned int sensorValues[NUM_SENSORS];
int myPins[6] = {5, 6, 7, 8, 9, 10};  // MOTOR PIN NUMBERS

//// PID TUNNING PARAMETERS
float Kp = 0.6;
float Kd = 0.1;
float Ki = 0;
///////////////////////////////

int preError = 0;
int integral = 0;
int rightSpeedPwm = 0;
int leftSpeedPwm = 0;

void setup() {
  for (int i = 0; i < 6; i++) {
    pinMode(myPins[i], OUTPUT);
  }

  pinMode(S0_PIN_1, OUTPUT);
  pinMode(S1_PIN_1, OUTPUT);
  pinMode(S2_PIN_1, OUTPUT);
  pinMode(S3_PIN_1, OUTPUT);
  pinMode(OUT_PIN_1, INPUT);

  pinMode(S0_PIN_2, OUTPUT);
  pinMode(S1_PIN_2, OUTPUT);
  pinMode(S2_PIN_2, OUTPUT);
  pinMode(S3_PIN_2, OUTPUT);
  pinMode(OUT_PIN_2, INPUT);

  digitalWrite(S0_PIN_1, LOW);
  digitalWrite(S1_PIN_1, LOW);
  digitalWrite(S0_PIN_2, LOW);
  digitalWrite(S1_PIN_2, LOW);

  Serial.begin(9600);

  for (int i = 0; i < 200; i++) {
    if (0 <= i && i < 5) rightFunc();
    if (5 <= i && i < 15) leftFunc();
    if (15 <= i && i < 25) rightFunc();
    if (25 <= i && i < 35) leftFunc();
    if (35 <= i && i < 45) rightFunc();
    if (45 <= i && i < 55) leftFunc();
    if (55 <= i && i < 65) rightFunc();
    if (65 <= i && i < 75) leftFunc();
    if (75 <= i && i < 85) rightFunc();
    if (85 <= i && i < 90) leftFunc();

    if (i >= 90) {
      stopFunc();
      delay(4);
    }

    qtrrc.calibrate();
    delay(4);
  }
}

void loop() {
  unsigned int sensorValues[8];
  unsigned int position = qtrrc.readLine(sensorValues, 1, 0);
  int error = position - 3500;

  // Trova il colore verde sui sensori
  bool greenDetectedLeft = trovaVerde1();
  bool greenDetectedRight = trovaVerde2();

  if (sensorValues[3] < 100 && sensorValues[4] < 100) {
    delay(1500);
  }
  if (greenDetectedLeft && greenDetectedRight) {
    digitalWrite(myPins[1], HIGH);
    digitalWrite(myPins[2], LOW);
    digitalWrite(myPins[3], LOW);
    digitalWrite(myPins[4], HIGH);
    delay(2120);
  }

  else if (greenDetectedLeft) {
    digitalWrite(myPins[1], HIGH);
    digitalWrite(myPins[2], LOW);
    digitalWrite(myPins[3], LOW);
    digitalWrite(myPins[4], HIGH);
    delay(1060);
  }

  else if (greenDetectedRight) {
    digitalWrite(myPins[1], LOW);
    digitalWrite(myPins[2], HIGH);
    digitalWrite(myPins[3], HIGH);
    digitalWrite(myPins[4], LOW);
    delay(1060);
  }
  // Altrimenti, continua dritto
  else {
    // Esegui il PID per mantenere la direzione corretta
    integral = integral + error;
    int myTurn = Kp * error + Kd * (error - preError) + Ki * integral;
    preError = error;

    rightSpeedPwm = rightSpeed + myTurn;
    leftSpeedPwm = leftSpeed - myTurn;
    rightSpeedPwm = constrain(rightSpeedPwm, -speedLimit, speedLimit);
    leftSpeedPwm = constrain(leftSpeedPwm, -speedLimit, speedLimit);
    moveRobot(rightSpeedPwm, leftSpeedPwm);
  }

  bool rosso = trovaRosso();

  if (rosso) {
    delay(20000000);
  }
}

void moveRobot(int myLeftSpeed, int myRightSpeed) {
  if (myLeftSpeed <= 0) {
    digitalWrite(myPins[1], LOW);
    digitalWrite(myPins[2], HIGH);
  } else {
    digitalWrite(myPins[1], HIGH);
    digitalWrite(myPins[2], LOW);
  }

  if (myRightSpeed <= 0) {
    digitalWrite(myPins[3], LOW);
    digitalWrite(myPins[4], HIGH);
  } else {
    digitalWrite(myPins[3], HIGH);
    digitalWrite(myPins[4], LOW);
  }

  analogWrite(myPins[0], abs(myLeftSpeed));
  analogWrite(myPins[5], abs(myRightSpeed));
}


void rightFunc() {
  moveRobot(-90, 90);  // Cambiare -90 in -255
}

void leftFunc() {
  moveRobot(90, -90);  // Cambiare -90 in -255
}

bool trovaVerde1() {
  updateRGB(1);
  return green_1 < red_1 && green_1 < blue_1;  // Modifica il valore di soglia in base alla lettura effettiva
}

bool trovaVerde2() {
  updateRGB(2);
  return green_2 < red_2 && green_2 < blue_2;  // Modifica il valore di soglia in base alla lettura effettiva
}


bool trovaRosso() {
  updateRGB(1);
  updateRGB(2);
  return (red_2 < green_2 && red_2 < blue_2) && (red_1 < green_1 && red_1 < blue_1);
}
void updateRGB(int sensorNumber) {
  if (sensorNumber == 1) {
    // Set the frequency for sensor 1
    digitalWrite(S0_PIN_1, HIGH);
    digitalWrite(S1_PIN_1, LOW);

    red_1 = 0;
    green_1 = 0;
    blue_1 = 0;
    clear_1 = 0;

    for (int i = 0; i < 5; i++) {
      digitalWrite(S2_PIN_1, LOW);
      digitalWrite(S3_PIN_1, LOW);
      red_1 += pulseIn(OUT_PIN_1, LOW);
      delay(1);

      digitalWrite(S2_PIN_1, HIGH);
      clear_1 += pulseIn(OUT_PIN_1, LOW);
      delay(1);

      digitalWrite(S3_PIN_1, HIGH);
      green_1 += pulseIn(OUT_PIN_1, LOW);
      delay(1);

      digitalWrite(S2_PIN_1, LOW);
      blue_1 += pulseIn(OUT_PIN_1, LOW);
      delay(1);
    }

    red_1 /= 5;
    green_1 /= 5;
    blue_1 /= 5;
    clear_1 /= 5;

    // Power down for sensor 1
    digitalWrite(S0_PIN_1, LOW);
    digitalWrite(S1_PIN_1, LOW);
  } else if (sensorNumber == 2) {
    // Set the frequency for sensor 2
    digitalWrite(S0_PIN_2, HIGH);
    digitalWrite(S1_PIN_2, LOW);
    delay(10);

    red_2 = 0;
    green_2 = 0;
    blue_2 = 0;
    clear_2 = 0;

    for (int i = 0; i < 5; i++) {
      digitalWrite(S2_PIN_2, LOW);
      digitalWrite(S3_PIN_2, LOW);
      red_2 += pulseIn(OUT_PIN_2, LOW);
      delay(1);

      digitalWrite(S2_PIN_2, HIGH);
      clear_2 += pulseIn(OUT_PIN_2, LOW);
      delay(1);

      digitalWrite(S3_PIN_2, HIGH);
      green_2 += pulseIn(OUT_PIN_2, LOW);
      delay(1);

      digitalWrite(S2_PIN_2, LOW);
      blue_2 += pulseIn(OUT_PIN_2, LOW);
      delay(1);
    }

    red_2 /= 5;
    green_2 /= 5;
    blue_2 /= 5;
    clear_2 /= 5;

    // Power down for sensor 2
    digitalWrite(S0_PIN_2, LOW);
    digitalWrite(S1_PIN_2, LOW);
  }
}
void stopFunc() {
  moveRobot(0, 0);
}