# include <QTRSensors.h>

QTRSensors qtr;

const uint8_t SensorCount = 8;
int sensorValues[SensorCount];
int sensorValuesB[SensorCount];

int motor1pin1 = 23; // sinistra
int motor1pin2 = 25;
int motor2pin1 = 27; // destra
int motor2pin2 = 29;

int enable1 = 2; // sinistra
int enable2 = 3; // destra

int velocitaRobot = 100;

int ledr = 8, ledg = 9, ledb = 10;

void setup() {
  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(motor2pin1, OUTPUT);
  pinMode(motor2pin2, OUTPUT);

  pinMode(enable1, OUTPUT);
  pinMode(enable2, OUTPUT);

  pinMode(ledr, OUTPUT);
  pinMode(ledg, OUTPUT);
  pinMode(ledb, OUTPUT);

  digitalWrite(ledr, HIGH);

  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, SensorCount);

  delay(500);
  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
  }

  Serial.begin(9600);
  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }

  Serial.println();
  Serial.println();
  delay(1000);
  digitalWrite(ledr, LOW);
  digitalWrite(ledg, HIGH);
}

void loop() {
  qtr.read(sensorValues);

  int position = qtr.readLineBlack(sensorValues);

  // Print the sensor values to the Serial Monitor
  Serial.print("Sensor Values: ");
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

  if (sensorValuesB[0] == 0 && sensorValuesB[1] == 0 && sensorValuesB[2] == 0 && sensorValuesB[3] == 1 && sensorValuesB[4] == 1 && sensorValuesB[5] == 0 && sensorValuesB[6] == 0 && sensorValuesB[7] == 0) {
    avanti();
  }

  if (sensorValuesB[0] == 1 && sensorValuesB[1] == 1 && sensorValuesB[2] == 1 && sensorValuesB[3] == 1 && sensorValuesB[4] == 1 && sensorValuesB[5] == 1 && sensorValuesB[6] == 1 && sensorValuesB[7] == 1) {
    avanti();
  }


  if (sensorValuesB[0] == 0 && sensorValuesB[1] == 1 && sensorValuesB[2] == 1 && sensorValuesB[3] == 1 && sensorValuesB[4] == 0 && sensorValuesB[5] == 0 && sensorValuesB[6] == 0 && sensorValuesB[7] == 0) {
    dxlegg();
  }

  if (sensorValuesB[0] == 0 && sensorValuesB[1] == 0 && sensorValuesB[2] == 0 && sensorValuesB[3] == 0 && sensorValuesB[4] == 0 && sensorValuesB[5] == 0 && sensorValuesB[6] == 1 && sensorValuesB[7] == 0) {
    dxlegg(); // Stabilizzazione
  }

  if (sensorValuesB[0] == 0 && sensorValuesB[1] == 0 && sensorValuesB[2] == 0 && sensorValuesB[3] == 0 && sensorValuesB[4] == 0 && sensorValuesB[5] == 1 && sensorValuesB[6] == 1 && sensorValuesB[7] == 0) {
    dxlegg(); // Stabilizzazione
  }

  if (sensorValuesB[0] == 0 && sensorValuesB[1] == 0 && sensorValuesB[2] == 0 && sensorValuesB[3] == 0 && sensorValuesB[4] == 1 && sensorValuesB[5] == 1 && sensorValuesB[6] == 1 && sensorValuesB[7] == 0) {
    dxlegg(); // Stabilizzazione
  }


  if (sensorValuesB[0] == 0 && sensorValuesB[1] == 0 && sensorValuesB[2] == 1 && sensorValuesB[3] == 1 && sensorValuesB[4] == 1 && sensorValuesB[5] == 0 && sensorValuesB[6] == 0 && sensorValuesB[7] == 0) {
    sxlegg(); // Stabilizzazione
  }

  if (sensorValuesB[0] == 0 && sensorValuesB[1] == 0 && sensorValuesB[2] == 0 && sensorValuesB[3] == 1 && sensorValuesB[4] == 0 && sensorValuesB[5] == 0 && sensorValuesB[6] == 0 && sensorValuesB[7] == 0) {
    sxlegg(); // Stabilizzazione
  }

  if (sensorValuesB[0] == 0 && sensorValuesB[1] == 0 && sensorValuesB[2] == 1 && sensorValuesB[3] == 1 && sensorValuesB[4] == 0 && sensorValuesB[5] == 0 && sensorValuesB[6] == 0 && sensorValuesB[7] == 0) {
    sxlegg(); // Stabilizzazione
  }

  if (sensorValuesB[0] == 0 && sensorValuesB[1] == 1 && sensorValuesB[2] == 1 && sensorValuesB[3] == 0 && sensorValuesB[4] == 0 && sensorValuesB[5] == 0 && sensorValuesB[6] == 0 && sensorValuesB[7] == 0) {
    sxlegg(); // Stabilizzazione
  }


  if (sensorValuesB[0] == 1 && sensorValuesB[1] == 1 && sensorValuesB[2] == 1 && sensorValuesB[3] == 1 && sensorValuesB[4] == 1 && sensorValuesB[5] == 0 && sensorValuesB[6] == 0 && sensorValuesB[7] == 0) {
    dx();
  }

  if (sensorValuesB[0] == 1 && sensorValuesB[1] == 1 && sensorValuesB[2] == 1 && sensorValuesB[3] == 1 && sensorValuesB[4] == 0 && sensorValuesB[5] == 0 && sensorValuesB[6] == 0 && sensorValuesB[7] == 0) {
    dx();
  }

  if (sensorValuesB[0] == 0 && sensorValuesB[1] == 0 && sensorValuesB[2] == 0 && sensorValuesB[3] == 0 && sensorValuesB[4] == 1 && sensorValuesB[5] == 1 && sensorValuesB[6] == 1 && sensorValuesB[7] == 1) {
    sx();
  }

  if (sensorValuesB[0] == 0 && sensorValuesB[1] == 0 && sensorValuesB[2] == 0 && sensorValuesB[3] == 1 && sensorValuesB[4] == 1 && sensorValuesB[5] == 1 && sensorValuesB[6] == 1 && sensorValuesB[7] == 1) {
    sx();
  }

  if (sensorValuesB[0] == 0 && sensorValuesB[1] == 0 && sensorValuesB[2] == 0 && sensorValuesB[3] == 0 && sensorValuesB[4] == 0 && sensorValuesB[5] == 0 && sensorValuesB[6] == 0 && sensorValuesB[7] == 0) {
    stop();
  }

  delay(10);
}

void stop() {
  digitalWrite(ledb, LOW);

  analogWrite(enable1, 0);
  analogWrite(enable2, 0);

  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, LOW);
  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, LOW);
}

void avanti() {
  digitalWrite(ledb, HIGH);
  analogWrite(enable1, velocitaRobot);
  analogWrite(enable2, velocitaRobot);

  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW);
  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);
}

void dxlegg() {
  analogWrite(enable1, velocitaRobot);
  analogWrite(enable2, velocitaRobot * 0.7);

  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW);
  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);
}

void sxlegg() {
  analogWrite(enable1, velocitaRobot * 0.7);
  analogWrite(enable2, velocitaRobot);

  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW);
  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);
}

void sx() {
  analogWrite(enable1, velocitaRobot * 0.5);
  analogWrite(enable2, velocitaRobot);

  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW);
  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);
}

void dx() {
  analogWrite(enable1, velocitaRobot);
  analogWrite(enable2, velocitaRobot * 0.5);

  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW);
  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);
}

void indietro(){
  analogWrite(enable1, velocitaRobot);
  analogWrite(enable2, velocitaRobot);

  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, HIGH);
  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, HIGH);
}