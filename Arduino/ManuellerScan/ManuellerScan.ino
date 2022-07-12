#include "MeBaseBoard.h"
#include <Wire.h>
#include <SoftwareSerial.h>

// Programm nur zur Aufnahme von Testdaten, da echter Testlauf auf Grund von Problemen mit python nicht möglich ist (Probleme mit kivy nach update der Entwicklungsumgebung 27.06.2022)


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


const int anzahlMesswerte = 12;// die Sensoren haben einen Messwinkel von ca.30°, somit ergeben sich 360/30 = 12 Messwerte pro Messung
const float messwinkel = 30;
const float minAbstandVHLR = 50;

const float radAbstand = 181; //Abstand Radmitte zu Radmitte in mm
const float radDurchmesser = 68.5; // in mm

float abstandsDaten[anzahlMesswerte];
float vTest = 50;
float vStop = 0;
int measurementsCounter; //Zähler Anzahl Messreihen
int gi_richtung = 99;

int startzeit, endzeit, gefahreneZeit;
float startDistanzHintenUss, gefahreneDistanzUss;
float startPosMotorRechts, startPosMotorLinks, gefahreneDistanzMotorRechtsCm, gefahreneDistanzMotorLinksCm;

// für bluetoothsteuerung
const byte vorwaerts = 0xfa;
const byte rueckwaerts = 0xfb;
const byte links = 0xfc;
const byte rechts = 0xfd;
const byte automatik = 0xfe;
const byte messen = 0xff;

bool pruefeHinterenSensor;
char drivingMode = 'a';

//NEU
int measurements = 180/5;
const int arrayLength = 77; // Messungsnummer + 2*180/5 Messwerte + MotorRechts + MotorLinks + GefahreneZeit+ gedrehter winkel
float distanceData[arrayLength];
//int distanceData[arrayLength];
int drivingTime;
const int RpmGlobal = 60;
const float vGlobal = 115.4284589;
int angle;
int drivingStep = 1000; //Fahrdauer pro Befehl, entspricht Wert* vGlobal

void setup() {
  Initialisieren();
  motorRechts.begin();
  motorLinks.begin();
  Serial.begin(115200);
  bt.begin(115200);
  servoMotor.attach(SERV_SIGN_PIN, 900, 1500);
  measurementsCounter = 1;
  pruefeHinterenSensor = false;
}

void loop() {
  byte inDat;
  char inDatC;
  angle = 0;

  Initialisieren();

  if (measurementsCounter == 0){
    GetSensorData();
    GetMotorData();
    PrintData();
    Reset();
  }
  delay(2000);

  for (int i = 0; i<10; i++){
    Drive('f', 0);
    GetSensorData();
    GetMotorData();
    PrintData();
    Reset();
  }
        
}
//#####################################################################################################################
void PrintData(){
  for (int j = 0; j<arrayLength; j++){
       Serial.print(distanceData[j]);
       Serial.print(";");
    }
    Serial.println("");
}
//#####################################################################################################################
void Turn(char dir){
  int startMillis = 0;
  drivingTime = 0;
  startMillis = millis();
    switch (dir) {
      case 'l':
        motorLinks.runSpeed(-RpmGlobal);
        motorRechts.runSpeed(RpmGlobal);
        delay(1000);
        motorLinks.runSpeed(0);
        motorRechts.runSpeed(0);
        drivingTime = millis() - startMillis;
        break;
      case 'r':
        motorLinks.runSpeed(RpmGlobal);
        motorRechts.runSpeed(-RpmGlobal);
        delay(1000);
        motorLinks.runSpeed(0);
        motorRechts.runSpeed(0);
        drivingTime = millis() - startMillis;
        break;
      default:
        break;
    }
    delay(2000);  
}
//#####################################################################################################################
int TurnRobotAngle(int richtung){
  float runtime; // delaytime
  float neededDistance;
  int startMillis = 0;
  drivingTime = 0;
  startMillis = millis();
  
  neededDistance = (PI * radAbstand)/(360/angle);
  runtime = (neededDistance/vGlobal)*1000;
  Serial.print("Laufzeit für Drehung = ");
  Serial.println(runtime);
  if (runtime < 0){
    runtime = runtime * (-1); 
    motorRechts.runSpeed(-RpmGlobal);
    motorLinks.runSpeed(-RpmGlobal);
    delay(runtime);
    drivingTime = millis() - startMillis;
  }
  else {
    motorRechts.runSpeed(RpmGlobal);
    motorLinks.runSpeed(RpmGlobal);
    delay(runtime);
    drivingTime = millis() - startMillis;
  }
  Stop();
  delay(100);
}
//#####################################################################################################################
void Drive(char dir, int duration){
  int startMillis = 0;
  drivingTime = 0;

  if (duration == 0){
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
        break;
      case 'b':
        motorLinks.runSpeed(60);
        motorRechts.runSpeed(-60);
        delay(duration);
        motorLinks.runSpeed(0);
        motorRechts.runSpeed(0);
        drivingTime = millis() - startMillis;
        break;
      default:
        break;
    }
    delay(2000);
}
//#####################################################################################################################
void ScanAndSend(){
  GetSensorData();
  GetMotorData();
  SendMeasurements();
}
//#####################################################################################################################
void Reset(){
  motorRechts.reset();
  motorLinks.reset();
  // Delete Measurements
  for(int i = 0; i < arrayLength; ++i)
  {
    distanceData[i] = 0;
  }
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
void FahreDistanzCm(float distanz, float rpm, char richtung) {
  float runtime; // delaytime
  float rps;
  float radUmdrehungen = 0;
  distanz = distanz * 10; // in mm umrechnen
  radUmdrehungen = 2*distanz / (radDurchmesser * PI);
  rps = rpm/60;
  
  runtime = (radUmdrehungen/rps)*1000;

  Serial.print("Benötigte Umdrehungen für ");
  Serial.print(distanz/10);
  Serial.print(" cm = ");
  Serial.println(radUmdrehungen);
  
  Serial.print("Laufzeit bei ");
  Serial.print(distanz/10);
  Serial.print(" cm Distanz = ");
  Serial.print(runtime);
  Serial.println(" ms.");
  
  if (richtung == 'f') {
    motorRechts.runSpeed(rpm);
    motorLinks.runSpeed(-rpm);
    delay(runtime);
  }
  if (richtung == 'b') {
    motorRechts.runSpeed(-rpm);
    motorLinks.runSpeed(rpm);
    delay(runtime);
  }
  delay(100);
  Stop();
}
//#####################################################################################################################
void FahreVorwaerts(float v) {
  motorRechts.runSpeed(v);
  motorLinks.runSpeed(-v);
}
//#####################################################################################################################
void Stop() {
  motorRechts.runSpeed(0);
  motorLinks.runSpeed(0);
}
//#####################################################################################################################
void GetMotorData(){
  distanceData[arrayLength-3] = motorRechts.getCurrentPosition();
  distanceData[arrayLength-2] = motorLinks.getCurrentPosition();
  distanceData[arrayLength-1] = drivingTime;
}
//#####################################################################################################################
int GetSensorData(){ //SensorDatenaufnehmen mit 5°Schritten (tatsächlich weniger, Umrechnung in echten Winkel in Python
  Stop();
  servoMotor.write(0);
  measurementsCounter++;
  distanceData[0] = measurementsCounter;
  for(int i = 1; i < 37; i++){
    servoMotor.write(i*5);
    delay(100);
    distanceData[i] = uss_v.distanceCm()*10;//wir wollen alles in mm
    delay(100);
    distanceData[i+180/5] = uss_h.distanceCm()*10;//wir wollen alles in mm
    delay(100);
  }
  servoMotor.write(90); //Beim Fahren nach vorne 
  delay(500);
}
//#####################################################################################################################
void SendMeasurements(){
  Serial.println("Sende Daten..."); 
  for (int i = 0; i < arrayLength; i++) {
      int dataToSend = distanceData[i]*10;
      bt.println(dataToSend);
    //bt.print(distanceData[i]);
    //bt.print('\n');
    delay(20);
  } 
}
//#####################################################################################################################
void AutomodeSteps() {
  Initialisieren();

  //Wir bringen den Roboter in Position
  int nextDirection = 0;
  float distance = distanceData[0];
  for (int i = 1; i< (arrayLength -3);i++){ //ermittle Datensatz mit größter Distanz
    if (distanceData[i] > distance) {
     nextDirection = i;  
    }
  }
 
  //Berechne notwendigen Drehwinkel für neue Richtung
  int nextAngle = 999;
  if (nextDirection < 36){
    nextAngle = -70+(nextDirection * 5);
  }
  else {
    nextAngle = 110+((nextDirection - 36) * 5);
  }

  Serial.print("neue Richtung = ");
  Serial.println(nextAngle);
  Serial.print("Distanz = ");
  Serial.println(distanceData[nextDirection]);
  
  TurnRobotAngle(nextAngle);
  ScanAndSend();
  Reset();
  Initialisieren();
  
  //berechne theoretische Fahrtzeit und los geht's
  int duration = (((distanceData[nextDirection] - minAbstandVHLR) * 10 )/ vGlobal)*1000;
  if (duration < 0){
    duration *= -1;
  }
  Serial.print("neue Fahrtzeit = ");
  Serial.println(duration);
  
  Drive( 'f' , duration);
  ScanAndSend();
  Reset();
  
  delay(500);
}
