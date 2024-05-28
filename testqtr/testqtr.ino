# include <QTRSensors.h>
QTRSensors qtr;

const uint8_t SensorCount = 8;
int sensorValues[SensorCount];
int sensorValuesB[SensorCount];
void setup(){
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
}

void loop(){
  int position = qtr.readLineBlack(sensorValues);
  for (int i = 0; i<SensorCount; i++){
    if (sensorValues[i] > 800){
      sensorValuesB[i] = 1;
    }
    else{
      sensorValuesB[i] = 0;
    }
    Serial.print(sensorValuesB[i]);
    Serial.print('\t');
  }
  Serial.println(position);
  delay(400);
}