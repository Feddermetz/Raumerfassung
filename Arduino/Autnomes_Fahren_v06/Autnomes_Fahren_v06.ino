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
const int arrayLength = 77; // 2*180/5 Messwerte + MotorRechts + MotorLinks + GefahreneZeit
float distanceData[arrayLength];
int drivingTime;
const int RpmGlobal = 60;
const float vGlobal = 115.4284589;
int angle;
int drivingStep = 10000; //Fahrdauer pro Befehl, entspricht Wert* vGlobal

void setup() {
  Initialisieren();
  motorRechts.begin();
  motorLinks.begin();
  Serial.begin(115200);
  bt.begin(115200);
  servoMotor.attach(SERV_SIGN_PIN, 900, 1500);

  measurementsCounter = 0;
  pruefeHinterenSensor = false;
}

void loop() {
  byte inDat;
  char inDatC;
  angle = 0;
  if (bt.available()) {
    drivingMode = ' ';
    Stop();
    inDat = bt.read();
    inDatC = (char)inDat;
    Serial.println(inDat, HEX);
    Serial.println(inDat, BIN); //,HEX);
    Serial.println(inDatC); //,HEX);

    Initialisieren();

    if (measurementsCounter == 0){
      measurementsCounter++;
      ScanAndSend();
    }
    delay(5000);
    switch (inDatC) {
      case '8':
        Drive('f', 0);
        ScanAndSend();
        PrintData();
        Reset();
        break;
      case '2':
        Drive('b', 0);
        ScanAndSend();
        PrintData();
        Reset();
        break;
      case '4':
      angle = +90;
        TurnRobotAngle(angle);
        ScanAndSend();
        PrintData();
        Reset();
        break;
      case '6':
        angle = -90;
        TurnRobotAngle(angle);
        ScanAndSend();
        PrintData();
        Reset();
        break;  
      case '7': //45° left
        angle = 45;
        TurnRobotAngle(angle);
        ScanAndSend();
        PrintData();
        Reset();
        break;
      case '9': //45° right
        angle = -45;
        TurnRobotAngle(angle);
        ScanAndSend();
        PrintData();
        Reset();
        break;
      case '1': //45° left
        angle = 135;
        TurnRobotAngle(angle);
        ScanAndSend();
        PrintData();
        Reset();
        break;
      case '3': //45° right
        angle = -135;
        TurnRobotAngle(angle);
        ScanAndSend();
        PrintData();
        Reset();
        break;
      case 'a':
        drivingMode = 'a';
        AutomodeSteps();
        break;
      case 's':
        drivingMode = 's';
        break;
      case 'm':
        break;
      default:  
        Serial.println("Fehler, kein Fahrmodus gewählt");
        break;
    }
    
  }
  else {
     //GetSensorData();
     //AutomodeSteps();
     //delay(5000);
  }
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
  distanceData[arrayLength-4] = angle;
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
void PrintData(){
  for (int j = 0; j<arrayLength; j++){
       Serial.print(distanceData[j]);
       Serial.print(";");
    }
    Serial.println("");

    for (int i = 0; i < arrayLength; i++) {
      int dataToSend = distanceData[i];
      Serial.print(dataToSend);
      Serial.print(";");
    //bt.print(distanceData[i]);
    //bt.print('\n');
    delay(20);
  } 
  Serial.println("");
  
}
//#####################################################################################################################
void SendMeasurements(){
  Serial.println("Sende Daten..."); 
  for (int i = 0; i < arrayLength; i++) {
      int dataToSend = distanceData[i];
      bt.println(dataToSend);
    //bt.print(distanceData[i]);
    //bt.print('\n');
    delay(50);
  } 
  bt.print('\n');
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
/*

int PruefeUmgebung() {
  //Prüefe ob Vorwärtsfahrt möglich
  int neueRichtung = DatenAufnehmen(anzahlMesswerte);
  return neueRichtung;
}
void FahreRueckwaerts(float v) { //anpassen: hole akt v und fahre rückwärts
  motorRechts.runSpeed(-v);
  motorLinks.runSpeed(v);
}
int DatenAufnehmen(int anzahlMesswerte) { //nimmt Daten auf und gibt zurück wo es als nächstes hingeht
  Stop(); //Anhalten, wir messen erstmal nicht während der Fahrt
  int anzahlMessungen = anzahlMesswerte / 2;
  int winkelSchritt = 360 / (anzahlMesswerte - 2);
  int winkel = 0;
  InitSensoren();

  // Daten aufnehmen und in Array schreiben
  for (int i = 0; i < anzahlMessungen; i++) {
    abstandsDaten[i] = uss_v.distanceCm();
    abstandsDaten[i + anzahlMessungen] = uss_h.distanceCm();
    winkel = i * winkelSchritt;
    DreheSensoren(winkel);
    delay(500);
  }

  //Ausgabe, merken der nächsten Fahrtrichtung
  int neueRichtung = 99; //wir merken uns gleich den Index des größten Abstandswert, dieser gibt die nächste Richtung vor
  float maxDistanz = 0;
  for (int i = 0; i < anzahlMesswerte; i++) {
    Serial.println(abstandsDaten[i]);

    //Idee: wir fahren als nächstes in die Rictung mit dem größten abstand zu einem Objekt, ausgenommen der Bereiche 0° bis 60° und 300° bis 360°. Gilt nicht bei erster Messreihe
    if (anzahlMessungenBisher != 0) {
      if (i >= 3 && i <= 9) { //bei diesen Messwerten interessieren die Abstände für die neue Richtung nicht
        if (abstandsDaten[i] > maxDistanz) {
          maxDistanz = abstandsDaten[i];
          neueRichtung = i;//i mal Winkelschrittweite ergibt neue Richtung
        }
      }
    }
    else { // Bei der ersten Messung kann der Roboter auch in die oben ausgeschlossenen Bereichen losfahren
      if (abstandsDaten[i] >  maxDistanz) {
        maxDistanz = abstandsDaten[i];
        neueRichtung = i;//i mal Winkelschrittweite ergibt neue Richtung
      }
    }
  }

  if (maxDistanz >= 380) { //Merker Setzen
    pruefeHinterenSensor = true;
  }
  InitSensoren();
  anzahlMessungenBisher++; // TODO: Den Wert optional auch in das Array einbauen
  SendeDaten();
  return neueRichtung; // Alles super, wir geben die neue Richtung zurück
}
void SendeDaten() {
  Serial.println("Sende Daten...");
  for (int i = 0; i < anzahlMesswerte; i++) {
    bt.print(abstandsDaten[i]);
    bt.print('\n');
    delay(20);
  }
  float mPosRight = motorRechts.getCurrentPosition();
  float mPosLeft = motorLinks.getCurrentPosition();
  bt.print(mPosLeft);
  bt.print('\n');
  delay(20);
  bt.print(mPosRight);
  bt.print('\n');
  delay(20);
}


void automode_continuous() {
  Initialisieren();

  //Wir bringen den Roboter in Position
  Serial.print("neue Richtung = ");
  Serial.println(gi_richtung * (360 / anzahlMesswerte));
  if (gi_richtung != 99) {
    if (gi_richtung <= 6) { //wir drehen nach links
      for (int j = 0; j < gi_richtung; j++) {
        DreheRoboterRunSpeed(messwinkel, vTest, 'l');
      }
    }
    if (gi_richtung >= 7) { //wir drehen nach rechts
      for (int j = 0; j < (anzahlMesswerte - gi_richtung); j++) {
        DreheRoboterRunSpeed(messwinkel, vTest, 'r');
      }
    }
  }
  // TODO: Drehungsdaten noch über Motordaten berechnen

  Initialisieren();
  startzeit = millis();
  startDistanzHintenUss = uss_h.distanceCm();
  FahreVorwaerts(vTest); //Fahre Vorwärts bis es nicht mehr geht
  if (pruefeHinterenSensor == true) { // Neue Richtung hatte eine Distanz die auf kein Objekt im Weg hinweist.
    while (uss_h.distanceCm() < 380) { // Wir benötogen also einen neuen Messpunkt nach ca. 380cm fahrt
      Serial.print("Prüfe hinteren sensor: ");
      Serial.println(uss_h.distanceCm());
    }
    Serial.print("Letzter Messwert Sensor hinten: ");
    Serial.println(uss_h.distanceCm());
    pruefeHinterenSensor = false;
  }
  else { //wir fahren bis wir auf ein hindernis stoßen
    while (uss_v.distanceCm() > minAbstandVHLR) {
      //Serial.println(uss_v.distanceCm());
    }
  }

  Stop();
  gefahreneDistanzUss = uss_h.distanceCm() - startDistanzHintenUss;
  gefahreneDistanzMotorRechtsCm = (motorRechts.getCurrentPosition() / (2 * 360)) * radDurchmesser * PI * 10;
  gefahreneDistanzMotorLinksCm = (motorLinks.getCurrentPosition() / (2 * 360)) * radDurchmesser * PI * 10;
  Serial.print("gefahrene Distanz MotorLinks = ");
  Serial.println(gefahreneDistanzMotorLinksCm);
  Serial.print("gefahrene Distanz MotorRechts = ");
  Serial.println(gefahreneDistanzMotorRechtsCm);

  gefahreneZeit = millis() - startzeit ;
  gi_richtung = PruefeUmgebung();
  Serial.print("gefahrene Zeit in Microsekunden = ");
  Serial.println(gefahreneZeit);
  delay(500);
}
int DatenAufnehmenAlt(int anzahlMesswerte) { //nimmt Daten auf und gibt zurück wo es als nächstes hingeht
  Stop(); //Anhalten, wir messen erstmal nicht während der Fahrt
  int anzahlMessungen = anzahlMesswerte / 2;
  int winkelSchritt = 360 / (anzahlMesswerte - 2);
  int winkel = 0;
  InitSensoren();

  // Daten aufnehmen und in Array schreiben
  for (int i = 0; i < anzahlMessungen; i++) {
    abstandsDaten[i] = uss_v.distanceCm();
    abstandsDaten[i + anzahlMessungen] = uss_h.distanceCm();
    winkel = i * winkelSchritt;
    DreheSensoren(winkel);
    delay(500);
  }

  //Ausgabe, merken der nächsten Fahrtrichtung
  int neueRichtung = 99; //wir merken uns gleich den Index des größten Abstandswert, dieser gibt die nächste Richtung vor
  float maxDistanz = 0;
  for (int i = 0; i < anzahlMesswerte; i++) {
    Serial.println(abstandsDaten[i]);

    //Idee: wir fahren als nächstes in die Rictung mit dem größten abstand zu einem Objekt, ausgenommen der Bereiche 0° bis 60° und 300° bis 360°. Gilt nicht bei erster Messreihe
    if (anzahlMessungenBisher != 0) {
      if (i >= 3 && i <= 9) { //bei diesen Messwerten interessieren die Abstände für die neue Richtung nicht
        if (abstandsDaten[i] > maxDistanz) {
          maxDistanz = abstandsDaten[i];
          neueRichtung = i;//i mal Winkelschrittweite ergibt neue Richtung
        }
      }
    }
    else { // Bei der ersten Messung kann der Roboter auch in die oben ausgeschlossenen Bereichen losfahren
      if (abstandsDaten[i] >  maxDistanz) {
        maxDistanz = abstandsDaten[i];
        neueRichtung = i;//i mal Winkelschrittweite ergibt neue Richtung
      }
    }
  }

  if (maxDistanz >= 380) { //Merker Setzen
    pruefeHinterenSensor = true;
  }
  InitSensoren();
  anzahlMessungenBisher++; // TODO: Den Wert optional auch in das Array einbauen
  SendeDaten();
  return neueRichtung; // Alles super, wir geben die neue Richtung zurück
}
//TODOS:
/*  1. Wenn neue Abstandswert zur neuen Richtung größer 380: roboter muss dann auch den hinteren Sensor bei vorwärtsfahrt
       prüfen und bei distanz Sensor hinten >= 380 cm stehen bleiben und neue Messung starten
       ALTERNATIV: nie in richtung >380/400 fahren. ist aber problematisch in großem Raum ##################erledigt
    2. abstand in allen modi mittels drehzahlen der räder berechnen. initialisieren() bereits gemacht
*/

// LOG
/* 

*/
// Nutzloses Zeug
/*
  void DreheNachLinks(){
  motorRechts.runSpeed(50);
  motorLinks.runSpeed(50);
  //delay(250);
  //Stop();
  }
  void DreheNachRechts(){
  motorRechts.runSpeed(-50);
  motorLinks.runSpeed(-50);
  //delay(250);
  //Stop();
  }
*/
/*
  bool PruefeHindernisse(){
   //Nehme die Daten der Sensoren auf für vorne und links
  // Für Später wenn der Servo da ist
  if (uss_v.distanceCm()< minAbstandVhCm){
    return 'V';
  }
  if (uss_h.distanceCm()< minAbstandLrCm){
    return 'L';
  }
  DreheSensoren(0);
  if (uss_v.distanceCm()< minAbstandVhCm){
    return 'H';
  }
  if (uss_h.distanceCm()< minAbstandLrCm){
    return 'R';
  }
  return 'Z';
  }*/
/*
  void DreheRoboter30(){
  motorRechts.runSpeed(vTest);
  motorLinks.runSpeed(vTest);
  delay(500);
  Stop;
  }  
  */

/*int DreheRoboterRunTurns(float winkel, char richtung) {
  float streckeRad = 0;
  float radUmdrehungen = 0;

  streckeRad = (winkel / 360) * PI * radAbstand;
  radUmdrehungen = streckeRad / (radDurchmesser * PI);

  if (richtung == 'l') {
    Serial.println("Drehe Roboter links");
    motorRechts.runTurns(2 * radUmdrehungen, 50);
    motorLinks.runTurns(2 * radUmdrehungen, 50);
  }
  if (richtung == 'r') {
    Serial.println("Drehe Roboter rechts");
    motorRechts.runTurns(2 * radUmdrehungen, -50);
    motorLinks.runTurns(2 * radUmdrehungen, -50);
  }
  delay(5000);
  return 0;
}
//#####################################################################################################################
void SendAck(){
  Serial.println("Sende Bestätigung...");
  bt.print(1);  //Statuscode 1: Drehung
  bt.print('\n');
  
  float mPosRight = motorRechts.getCurrentPosition();
  float mPosLeft = motorLinks.getCurrentPosition();
  bt.print(mPosLeft);
  bt.print('\n');
  delay(20);
  bt.print(mPosRight);
  bt.print('\n');
  delay(20);
}
//#####################################################################################################################
void DreheSensoren(int winkel) {
  servoMotor.write(winkel);
  delay(200);
}
//#####################################################################################################################
void InitSensoren() {
  servoMotor.write(0);
  delay(100);
}
//#####################################################################################################################
*/
