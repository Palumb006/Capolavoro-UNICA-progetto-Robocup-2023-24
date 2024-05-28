#include <QTRSensors.h>

#define speedLimit 220
#define rightSpeed 160
#define leftSpeed 160
#define NUM_SENSORS 8

#define S0_PIN_1 22 //sx
#define S1_PIN_1 24
#define S2_PIN_1 26
#define S3_PIN_1 28
#define OUT_PIN_1 30

#define S0_PIN_2 23 //dx
#define S1_PIN_2 25
#define S2_PIN_2 27
#define S3_PIN_2 29
#define OUT_PIN_2 31

int motor1pin1 = 3;
int motor1pin2 = 4; //dx
int motor2pin1 = 5;
int motor2pin2 = 6;
#define enB 2
#define enA 7 //dx

int red_1, green_1, blue_1, clear_1;
int red_2, green_2, blue_2, clear_2;

QTRSensorsRC qtrrc((unsigned char[]){
                     A0, A1, A2, A3, A4, A5, A6, A7  // SENSOR PIN NUMBERS
                   },
                   NUM_SENSORS, 2500, 2);
unsigned int sensorValues[NUM_SENSORS];

int controlloQTR[8] = {53, 51, 49, 47, 46, 48, 50, 52};
int ledRGB[3] = {40, 42, 44};
int myInput = 45; // pulsante
#define debug true
bool stat = true;
//// PID TUNNING PARAMETERS ////
float Kp = 0.09;
float Kd = 0.8;
float Ki = 0;
////////////////////////////////

int preError = 0;
int integral = 0;
int rightSpeedPwm = 0;
int leftSpeedPwm = 0;
int myFloor = 0;

void setup() {
  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(motor2pin1, OUTPUT);
  pinMode(motor2pin2, OUTPUT);

  for (int i = 0; i < 8; i++) pinMode(controlloQTR[i], OUTPUT);
  for (int i = 0; i < 3; i++) pinMode(ledRGB[i], OUTPUT);
  start();
  while (stat) {
    stat = digitalRead(myInput) == 1 ? false : true;
  }
  digitalWrite(ledRGB[1], HIGH);

  // pinMode(11, OUTPUT);
  // digitalWrite(11, HIGH);
  delay(1000);
  int i;

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

  // fine calibrazione led blu
  digitalWrite(ledRGB[0], LOW);
  digitalWrite(ledRGB[1], LOW);
  digitalWrite(ledRGB[2], HIGH);

  stat = true;
  while (stat) {
    stat = digitalRead(myInput) == 1 ? false : true;
  }

  digitalWrite(ledRGB[2], LOW);
  digitalWrite(ledRGB[1], HIGH);

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
}

void loop() {
  unsigned int sensorValues[8];
  unsigned int position = qtrrc.readLine(sensorValues, 1, 0);
  int error = position - 3500;
/**
  if (sensorValues[0] < 100 && sensorValues[7] < 100) {
    myFloor = 0;  //white ground - black line
  }

  if (sensorValues[0] > 500 && sensorValues[7] > 500) {
    myFloor = 1;  //black ground - white line
  }
**/
  int verde1 = trovaVerde1();
  int verde2 = trovaVerde2();
  // trovaVerde1() sensore sx, trovaVerde2() sensore dx
  if (verde1 == 1 && verde2 == 1) {  // 1 = trovato, 0 = non trovato
    digitalWrite(motor1pin1, HIGH);
    digitalWrite(motor1pin2, LOW);
    digitalWrite(motor2pin1, LOW);
    digitalWrite(motor2pin2, HIGH);
    delay(2120);
  }

  if (verde1 == 1 && verde2 == 0) {  // 1 = trovato, 0 = non trovato
    digitalWrite(motor1pin1, HIGH);
    digitalWrite(motor1pin2, LOW);
    digitalWrite(motor2pin1, LOW);
    digitalWrite(motor2pin2, HIGH);
    delay(1060);
  }

  if (verde1 == 0 && verde2 == 1) {  // 1 = trovato, 0 = non trovato
    digitalWrite(motor1pin1, LOW);
    digitalWrite(motor1pin2, HIGH);
    digitalWrite(motor2pin1, HIGH);
    digitalWrite(motor2pin2, LOW);
    delay(1060);
  }

  integral = integral + error;
  int myTurn = Kp * error + Kd * (error - preError) + Ki * integral;
  preError = error;

  rightSpeedPwm = rightSpeed + myTurn;
  leftSpeedPwm = leftSpeed - myTurn;
  rightSpeedPwm = constrain(rightSpeedPwm, -speedLimit, speedLimit);
  leftSpeedPwm = constrain(leftSpeedPwm, -speedLimit, speedLimit);
  moveRobot(rightSpeedPwm, leftSpeedPwm);
}

void moveRobot(int myLeftSpeed, int myRightSpeed) {
  if (myLeftSpeed <= 0) {
    digitalWrite(motor1pin1, LOW);
    digitalWrite(motor1pin2, HIGH);
  } else {
    digitalWrite(motor1pin1, HIGH);
    digitalWrite(motor1pin2, LOW);
  }
  if (myRightSpeed <= 0) {
    digitalWrite(motor2pin1, LOW);
    digitalWrite(motor2pin2, HIGH);
  } else {
    digitalWrite(motor2pin1, HIGH);
    digitalWrite(motor2pin2, LOW);
  }

  analogWrite(enA, abs(myLeftSpeed));
  analogWrite(enB, abs(myRightSpeed));
}

// test di tutti i led e imposta del bottone su rosso in attesa del click del bottone
void start() {
  for (int i = 0; i < 8; i++) {
    digitalWrite(controlloQTR[i], HIGH);
    delay(500);
    digitalWrite(controlloQTR[i], LOW);
  }
  delay(100);
  for (int i = 0; i < 3; i++) {
    digitalWrite(ledRGB[i], HIGH);
    delay(500);
    digitalWrite(ledRGB[i], LOW);
  }
  digitalWrite(ledRGB[0], HIGH);
}

void stopFunc() {
  moveRobot(0, 0);
}

void rightFunc() {
  moveRobot(-90, 90);
}

void leftFunc() {
  moveRobot(90, -90);
}

int trovaVerde1() {
  updateRGB(1);
  if (red_1 < green_1 && red_1 < blue_1) {
    digitalWrite(31, LOW);
  } else if (green_1 < red_1 && green_1 < blue_1) {
    digitalWrite(31, HIGH);
    return 1;
  } else if (blue_1 < red_1 && blue_1 < green_1) {
    digitalWrite(31, LOW);
  } else {
    digitalWrite(31, LOW);
  }

  return 0;
}

int trovaVerde2() {
  updateRGB(2);
  if (red_2 < green_2 && red_2 < blue_2) {
    digitalWrite(31, LOW);
  } else if (green_2 < red_2 && green_2 < blue_2) {
    digitalWrite(31, HIGH);
    return 1;
  } else if (blue_2 < red_2 && blue_2 < green_2) {
    digitalWrite(31, LOW);
  } else {
    digitalWrite(31, LOW);
  }

  return 0;
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