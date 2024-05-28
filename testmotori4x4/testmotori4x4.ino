int motor1pin1 = 22;
int motor1pin2 = 24;
int motor2pin1 = 26;
int motor2pin2 = 28;
int enableA = 5; //ex32
int enableB = 4; //ex22

int motor3pin1 = 23; // ex3
int motor3pin2 = 25; // ex4
int motor4pin1 = 27; // ex5
int motor4pin2 = 29; // ex6
int enableC = 2;
int enableD = 3; //ex7

int velocitaRobot = 100;

void setup(){
pinMode(motor1pin1, OUTPUT);
pinMode(motor1pin2, OUTPUT);
pinMode(motor2pin1, OUTPUT);
pinMode(motor2pin1, OUTPUT);
pinMode(enableA, OUTPUT);
pinMode(enableB, OUTPUT);

pinMode(motor3pin1, OUTPUT);
pinMode(motor3pin2, OUTPUT);
pinMode(motor4pin1, OUTPUT);
pinMode(motor4pin2, OUTPUT);
pinMode(enableC, OUTPUT);
pinMode(enableD, OUTPUT);
}

void loop() {
  dx();
}

void avanti() {
  analogWrite(enableA, velocitaRobot);
  analogWrite(enableB, velocitaRobot);
  analogWrite(enableC, velocitaRobot);
  analogWrite(enableD, velocitaRobot);

  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW);
  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);

  digitalWrite(motor3pin1, HIGH);
  digitalWrite(motor3pin2, LOW);
  digitalWrite(motor4pin1, HIGH);
  digitalWrite(motor4pin2, LOW);
}

void dxlegg() {
  analogWrite(enableA, velocitaRobot);
  analogWrite(enableB, velocitaRobot * 0.5);
  analogWrite(enableC, velocitaRobot);
  analogWrite(enableD, velocitaRobot * 0.5 );

  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW);
  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);

  digitalWrite(motor3pin1, HIGH);
  digitalWrite(motor3pin2, LOW);
  digitalWrite(motor4pin1, HIGH);
  digitalWrite(motor4pin2, LOW);
}

void sxlegg() {
  analogWrite(enableA, velocitaRobot * 0.5);
  analogWrite(enableB, velocitaRobot);
  analogWrite(enableC, velocitaRobot * 0.5 );
  analogWrite(enableD, velocitaRobot);

  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW);
  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);

  digitalWrite(motor3pin1, HIGH);
  digitalWrite(motor3pin2, LOW);
  digitalWrite(motor4pin1, HIGH);
  digitalWrite(motor4pin2, LOW);
}

void sx() {
  analogWrite(enableA, velocitaRobot * 0.5);
  analogWrite(enableB, velocitaRobot);
  analogWrite(enableC, velocitaRobot * 0.5);
  analogWrite(enableD, velocitaRobot);

  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, HIGH);
  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);

  digitalWrite(motor3pin1, LOW);
  digitalWrite(motor3pin2, HIGH);
  digitalWrite(motor4pin1, HIGH);
  digitalWrite(motor4pin2, LOW);
}

void dx() {
  analogWrite(enableA, velocitaRobot);
  analogWrite(enableB, velocitaRobot * 0.7);
  analogWrite(enableC, velocitaRobot);
  analogWrite(enableD, velocitaRobot * 0.7);

  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW);
  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, HIGH);
 
  digitalWrite(motor3pin1, HIGH);
  digitalWrite(motor3pin2, LOW);
  digitalWrite(motor4pin1, LOW);
  digitalWrite(motor4pin2, HIGH);
}

void indietro(){
  analogWrite(enableA, velocitaRobot);
  analogWrite(enableB, velocitaRobot);
  analogWrite(enableC, velocitaRobot);
  analogWrite(enableD, velocitaRobot);

  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, HIGH);
  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, HIGH);

  digitalWrite(motor3pin1, LOW);
  digitalWrite(motor3pin2, HIGH);
  digitalWrite(motor4pin1, LOW);
  digitalWrite(motor4pin2, HIGH);
}