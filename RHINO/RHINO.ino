#include <QTRSensors.h>

/////////////////////////////////////////////////

#define speedLimit 150
#define rightSpeed 130
#define leftSpeed 130
#define NUM_SENSORS 8
QTRSensorsRC qtrrc((unsigned char[]) {
  A0, A1, A2, A3, A4, A5 ,A6 ,A7           // SENSOR PIN NUMBERS
},
NUM_SENSORS, 2500, 2);
unsigned int sensorValues[NUM_SENSORS];
int myPins[6] = {5, 6, 7, 8, 9, 10};      // MOTOR PIN NUMBERS

//// PID TUNNING PARAMETERS
float Kp = 0.6;
float Kd = 0.1;
float Ki = 0;

/////////////////////////////////////////////////

int preError = 0;
int integral = 0;
int rightSpeedPwm = 0;
int leftSpeedPwm = 0;
//int myFloor = 0;


void setup()
{
  for (int i = 0; i < 6; i++) {
    pinMode(myPins[i], OUTPUT);
  }
  int i;

  for (int i = 0; i < 200; i++)
  {
    if ( 0 <= i && i < 5   )  rightFunc();
    if ( 5 <= i && i  < 15   )  leftFunc();
    if ( 15 <= i && i < 25   )  rightFunc();
    if ( 25 <= i && i < 35   )  leftFunc();
    if ( 35 <= i && i < 45   )  rightFunc();
    if ( 45 <= i && i < 55  )  leftFunc();
    if ( 55 <= i && i < 65   )  rightFunc();
    if ( 65 <= i && i < 75  )  leftFunc();
    if ( 75 <= i && i < 85   )  rightFunc();
    if ( 85 <= i && i < 90  )  leftFunc();

    if ( i >= 90  )  {
      stopFunc();
      delay(4);
    }

    qtrrc.calibrate();
    delay(4);
  }
  Serial.begin(9600);

}

void loop(){
  // Serial.println("GIRO");
  unsigned int sensorValues[8];
  unsigned int position = qtrrc.readLine(sensorValues, 1, 0);
  int error = position - 3500;

  /*if (position == 0) {
      digitalWrite(myPins[1], HIGH);
      digitalWrite(myPins[2], LOW);
      digitalWrite(myPins[3], HIGH);
      digitalWrite(myPins[4], LOW);
      unsigned int position = qtrrc.readLine(sensorValues, 1, 0);

    //analogWrite(myPins[0], 80);
    //analogWrite(myPins[5], 80);
  }*/

  integral = integral + error;
  int myTurn = Kp * error + Kd * (error - preError) + Ki * integral;
  preError = error;

  rightSpeedPwm = rightSpeed + myTurn  ;
  leftSpeedPwm = leftSpeed - myTurn  ;
  rightSpeedPwm = constrain(rightSpeedPwm, -speedLimit, speedLimit);
  leftSpeedPwm = constrain(leftSpeedPwm, -speedLimit, speedLimit);
  moveRobot(rightSpeedPwm, leftSpeedPwm);

}

void moveRobot(int myLeftSpeed, int myRightSpeed)
{
  if (myLeftSpeed <= 0) {
    digitalWrite(myPins[1], 0);
    digitalWrite(myPins[2], 1);
  }
  else {
    digitalWrite(myPins[1], 1);
    digitalWrite(myPins[2], 0);
  }

  if (myRightSpeed <= 0) {
    digitalWrite(myPins[3], 0);
    digitalWrite(myPins[4], 1);
  }
  else {
    digitalWrite(myPins[3], 1);
    digitalWrite(myPins[4], 0);
  }

  analogWrite(myPins[0], abs(myLeftSpeed));
  analogWrite(myPins[5], abs(myRightSpeed));
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