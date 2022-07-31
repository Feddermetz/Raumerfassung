#include "MeBaseBoard.h"
#include <Wire.h>
#include <SoftwareSerial.h>

// Servo zum Drehen der Ultraschallsensoren
#define SERV_SIGN_PIN 10
Servo servoMotor;

// Motoren
MeEncoderMotor motorRechts(0x09, SLOT1);   //  Motor rechts
MeEncoderMotor motorLinks(0x09, SLOT2);   //  Motor links

// Ultraschallsenoren
MeUltrasonicSensor uss_v(6); // Ultraschallsensor vorne
MeUltrasonicSensor uss_h(7); // Ultraschallsensor links

// Bluetoothmodul
MeBluetooth bt(PORT_4);


const float radAbstand = 181; //Abstand Radmitte zu Radmitte in mm
int measurementsCounter; //Zähler Anzahl Messreihen
int startzeit, endzeit, gefahreneZeit;
float startDistanzHintenUss, gefahreneDistanzUss;
float startPosMotorRechts, startPosMotorLinks, gefahreneDistanzMotorRechtsCm, gefahreneDistanzMotorLinksCm;
int measurements = 180 / 5;
const int arrayLength = 77; // 2*180/5 Messwerte + MotorRechts + MotorLinks + GefahreneZeit
int distanceData[arrayLength];
int drivingTime;
const int RpmGlobal = 60;
const float vGlobal = 115.4284589;
int drivingStep = 10000; //Fahrdauer pro Befehl, entspricht Wert* vGlobal


void setup() {
  Initialisieren();
  motorRechts.begin();
  motorLinks.begin();
  Serial.begin(115200);
  bt.begin(115200);
  servoMotor.attach(SERV_SIGN_PIN, 900, 1500);
  measurementsCounter = 0;
}

void loop() {
  byte inDat;
  char inDatC;
  if (bt.available()) {
    Stop();
    inDat = bt.read();
    inDatC = (char)inDat;
    Initialisieren();
    delay(10);
    switch (inDatC) {
      case '8':
        Drive('f', 0);
        break;
      case '2':
        Drive('b', 0);
        break;
      case '4':
        TurnRobotAngle(90);
        break;
      case '6':
        TurnRobotAngle(-90);
        break;
      case '7':
        TurnRobotAngle(135);
        break;
      case '9':
        TurnRobotAngle(-135);
        break;
      case '1':
        TurnRobotAngle(45);
        break;
      case '3':
        TurnRobotAngle(-45);
        break;;
      default:
        break;
    }
    ScanAndSend();
  }
}
//#####################################################################################################################
int TurnRobotAngle(int angle) {
  float runtime;
  float neededDistance;
  int startMillis = 0;
  drivingTime = 0;
  startMillis = millis();

  neededDistance = (PI * radAbstand) / (360 / angle);
  runtime = (neededDistance / vGlobal) * 1000;
  if (runtime < 0) {
    runtime = runtime * (-1);
    motorRechts.runSpeed(-RpmGlobal);
    motorLinks.runSpeed(-RpmGlobal);
  }
  else {
    motorRechts.runSpeed(RpmGlobal);
    motorLinks.runSpeed(RpmGlobal);
  }
  delay(runtime);
  drivingTime = millis() - startMillis;
  Stop();
  distanceData[arrayLength - 4] = angle;
  //delay(100);
}
//#####################################################################################################################
void Drive(char dir, int duration) {
  int startMillis = 0;
  drivingTime = 0;
  if (duration == 0) {
    duration = drivingStep;
  }
  startMillis = millis();
  switch (dir) {
    case 'f':
      motorLinks.runSpeed(-60);
      motorRechts.runSpeed(60);
      delay(duration);
      motorLinks.runSpeed(0);
      motorRechts.runSpeed(0);
      drivingTime = millis() - startMillis;
      distanceData[arrayLength - 4] = 0;
      break;
    case 'b':
      motorLinks.runSpeed(60);
      motorRechts.runSpeed(-60);
      delay(duration);
      motorLinks.runSpeed(0);
      motorRechts.runSpeed(0);
      drivingTime = millis() - startMillis;
      distanceData[arrayLength - 4] = 999;
      break;
    default:
      break;
  }
}
//#####################################################################################################################
void ScanAndSend() {
  GetSensorData();
  GetMotorData();
  SendMeasurements();
}
//#####################################################################################################################
void Initialisieren() {
  servoMotor.write(90);
  startzeit = 0;
  endzeit = 0;
  gefahreneZeit = 0;
  startDistanzHintenUss = 0;
  gefahreneDistanzUss = 0;
  startPosMotorRechts = 0;
  startPosMotorLinks = 0;
  motorRechts.begin();
  motorLinks.begin();
  gefahreneDistanzMotorRechtsCm = 0;
  gefahreneDistanzMotorLinksCm = 0;
}
//#####################################################################################################################
void Stop() {
  motorRechts.runSpeed(0);
  motorLinks.runSpeed(0);
}
//#####################################################################################################################
void GetMotorData() {
  distanceData[arrayLength - 3] = (int)(motorRechts.getCurrentPosition() + 0.5);
  distanceData[arrayLength - 2] = (int)(motorLinks.getCurrentPosition() + 0.5);
  distanceData[arrayLength - 1] = (int)(drivingTime + 0.5);
}
//#####################################################################################################################
int GetSensorData() { //SensorDatenaufnehmen mit 5°Schritten (tatsächlich weniger, Umrechnung in echten Winkel in Python
  servoMotor.write(0);
  measurementsCounter++;
  distanceData[0] = measurementsCounter;
  for (int i = 1; i < 37; i++) {
    servoMotor.write(i * 5);
    delay(100);
    distanceData[i] = (int)(uss_v.distanceCm() + 0.5);
    delay(100);
    distanceData[i + 180 / 5] = (int)(uss_h.distanceCm() + 0.5);
    delay(100);
  }
}
//#####################################################################################################################
void SendMeasurements() {
  delay(50);
  for (int i = 0; i < arrayLength; i++) {
    Serial.println(distanceData[i]);
    bt.println(distanceData[i]);
    delay(10);
  }
  bt.print('\n');
  delay(1000);
}
