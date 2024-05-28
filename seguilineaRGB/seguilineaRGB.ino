# include <QTRSensors.h>
QTRSensors qtr;

const uint8_t SensorCount = 8;
int sensorValues[SensorCount];
int sensorValuesB[SensorCount];
int motor1pin1 = 6;
int motor1pin2 = 5;
int motor2pin1 = 4;
int motor2pin2 = 3;
int enableA = 2;
int enableB = 7;
int ledr = 8, ledg = 9, ledb = 10;
int velocitaRobot = 50;
void setup(){
  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(motor2pin1, OUTPUT);
  pinMode(motor2pin1, OUTPUT);
  pinMode(enableA, OUTPUT);
  pinMode(enableB, OUTPUT);
  pinMode(ledr, OUTPUT);
  pinMode(ledg, OUTPUT);
  pinMode(ledb, OUTPUT);
  digitalWrite(ledr, HIGH);

  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, SensorCount);

  delay(500);
  for (uint16_t i = 0; i < 400; i++){
    qtr.calibrate();
  }

  Serial.begin(9600);
  for (uint8_t i = 0; i < SensorCount; i++){
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++){
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }

  Serial.println();
  Serial.println();
  delay(1000);
  digitalWrite(ledr, LOW);
  digitalWrite(ledg, HIGH);
}

void loop(){
  int position = qtr.readLineBlack(sensorValues);
  for (int i = 0; i<SensorCount; i++){
    if (sensorValues[i] > 850){
      sensorValuesB[i] = 1;
    }
    else{
      sensorValuesB[i] = 0;
    }
    Serial.print(sensorValuesB[i]);
    Serial.print('\t');
  }

  if (sensorValuesB[0] == 0 && sensorValuesB[1] == 0 && sensorValuesB[2] == 0 && sensorValuesB[3] == 1 && sensorValuesB[4] == 1 && sensorValuesB[5] == 0 && sensorValuesB[6] == 0 && sensorValuesB[7] == 0){
    avanti();
  }

  if (sensorValuesB[0] == 1 && sensorValuesB[1] == 1 && sensorValuesB[2] == 1 && sensorValuesB[3] == 1 && sensorValuesB[4] == 1 && sensorValuesB[5] == 1 && sensorValuesB[6] == 1 && sensorValuesB[7] == 1){
    avanti();
  }

  if (sensorValuesB[0] == 0 && sensorValuesB[1] == 0 && sensorValuesB[2] == 0 && sensorValuesB[3] == 1 && sensorValuesB[4] == 1 && sensorValuesB[5] == 1 && sensorValuesB[6] == 0 && sensorValuesB[7] == 0){
    dxlegg();
  }
  
  if (sensorValuesB[0] == 0 && sensorValuesB[1] == 1 && sensorValuesB[2] == 1 && sensorValuesB[3] == 0 && sensorValuesB[4] == 0 && sensorValuesB[5] == 0 && sensorValuesB[6] == 0 && sensorValuesB[7] == 0){
    dxlegg(); //stabilizzazione
  }

  if (sensorValuesB[0] == 0 && sensorValuesB[1] == 0 && sensorValuesB[2] == 0 && sensorValuesB[3] == 0 && sensorValuesB[4] == 1 && sensorValuesB[5] == 0 && sensorValuesB[6] == 0 && sensorValuesB[7] == 0){
    dxlegg(); //stabilizzazione
  }

  if (sensorValuesB[0] == 1 && sensorValuesB[1] == 1 && sensorValuesB[2] == 1 && sensorValuesB[3] == 1 && sensorValuesB[4] == 1 && sensorValuesB[5] == 0 && sensorValuesB[6] == 0 && sensorValuesB[7] == 0){
    dx();
  }

  if (sensorValuesB[0] == 0 && sensorValuesB[1] == 0 && sensorValuesB[2] == 1 && sensorValuesB[3] == 1 && sensorValuesB[4] == 1 && sensorValuesB[5] == 1 && sensorValuesB[6] == 0 && sensorValuesB[7] == 0){
    sxlegg();
  }

  if (sensorValuesB[0] == 0 && sensorValuesB[1] == 0 && sensorValuesB[2] == 0 && sensorValuesB[3] == 1 && sensorValuesB[4] == 1 && sensorValuesB[5] == 0 && sensorValuesB[6] == 0 && sensorValuesB[7] == 0){
    sxlegg(); //stabilizzazione
  }

  if (sensorValuesB[0] == 0 && sensorValuesB[1] == 0 && sensorValuesB[2] == 0 && sensorValuesB[3] == 1 && sensorValuesB[4] == 0 && sensorValuesB[5] == 0 && sensorValuesB[6] == 0 && sensorValuesB[7] == 0){
    sxlegg(); //stabilizzazione
  }

  if (sensorValuesB[0] == 0 && sensorValuesB[1] == 0 && sensorValuesB[2] == 0 && sensorValuesB[3] == 0 && sensorValuesB[4] == 1 && sensorValuesB[5] == 1 && sensorValuesB[6] == 1 && sensorValuesB[7] == 1){
    sx();
  }

  if (sensorValuesB[0] == 0 && sensorValuesB[1] == 0 && sensorValuesB[2] == 0 && sensorValuesB[3] == 1 && sensorValuesB[4] == 1 && sensorValuesB[5] == 1 && sensorValuesB[6] == 1 && sensorValuesB[7] == 1){
    sx();
  }

  if (sensorValuesB[0] == 0 && sensorValuesB[1] == 0 && sensorValuesB[2] == 0 && sensorValuesB[3] == 0 && sensorValuesB[4] == 0 && sensorValuesB[5] == 0 && sensorValuesB[6] == 0 && sensorValuesB[7] == 0){
    stop();
  }

  Serial.println();
  delay(10);
}

void stop(){
  digitalWrite(ledb, LOW);
  analogWrite(enableA, 0);
  analogWrite(enableB, 0);
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, LOW);
  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, LOW);
}

void avanti() {
  digitalWrite(ledb, HIGH);
  analogWrite(enableA, velocitaRobot);
  analogWrite(enableB, velocitaRobot);
  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW);
  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);
}

void dxlegg() {
  analogWrite(enableA, velocitaRobot);
  analogWrite(enableB, velocitaRobot * 0.5);
  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW);
  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);
}

void sxlegg() {
  analogWrite(enableA, velocitaRobot * 0.5);
  analogWrite(enableB, velocitaRobot);
  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, HIGH);
  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);
}

void sx() {
  analogWrite(enableA, velocitaRobot);
  analogWrite(enableB, velocitaRobot);
  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, HIGH);
  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);
}

void dx() {
  analogWrite(enableA, velocitaRobot);
  analogWrite(enableB, velocitaRobot);
  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW);
  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, HIGH);
}