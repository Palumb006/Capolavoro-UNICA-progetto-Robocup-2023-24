int motor1pin1 = 24;
int motor1pin2 = 26;
int motor2pin1 = 28;
int motor2pin2 = 30;
int enableA = 22;
int enableB = 22;

int motor3pin1 = 3;
int motor3pin2 = 4;
int motor4pin1 = 5;
int motor4pin2 = 6;
int enableC = 2;
int enableD = 7;

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
analogWrite(enableA, 255);
analogWrite(enableB, 255);
digitalWrite(motor1pin1, HIGH);
digitalWrite(motor1pin2, LOW);
digitalWrite(motor2pin1, HIGH);
digitalWrite(motor2pin2, LOW);

analogWrite(enableC, 255);
analogWrite(enableD, 255);
digitalWrite(motor3pin1, HIGH);
digitalWrite(motor3pin2, LOW);
digitalWrite(motor4pin1, HIGH);
digitalWrite(motor4pin2, LOW);
}