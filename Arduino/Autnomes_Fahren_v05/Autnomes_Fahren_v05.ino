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
int anzahlMessungenBisher = 99; //Zähler Anzahl Messreihen
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

void setup() {
  Initialisieren();
  motorRechts.begin();
  motorLinks.begin();
  Serial.begin(115200);
  bt.begin(115200);
  servoMotor.attach(SERV_SIGN_PIN, 900, 1500);

  anzahlMessungenBisher = 0;
  gi_richtung = PruefeUmgebung(); // Anfangswerte Aufnehmen
  pruefeHinterenSensor = false;
}

void loop() {
  //char hindernisErkannt = 'N';
  byte inDat;
  char inDatC;
  if (bt.available()) {
    drivingMode = ' ';
    Stop();
    inDat = bt.read();
    inDatC = (char)inDat;
    Serial.println(inDat, HEX);
    Serial.println(inDat, BIN); //,HEX);
    Serial.println(inDatC); //,HEX);

    Initialisieren();

    switch (inDatC) {
      case 'f':
        FahreDistanzCm(10, vTest, 'f');
        PruefeUmgebung();
        motorRechts.reset();
        motorLinks.reset();
        break;
      case 'b':
        FahreDistanzCm(10, vTest,  'b');
        PruefeUmgebung();
        break;
      case 'l':
        DreheRoboterRunSpeed(messwinkel, vTest, 'l');
        PruefeUmgebung();
        break;
      case 'r':
        DreheRoboterRunSpeed(messwinkel, vTest, 'r');
        PruefeUmgebung();
        break;
      case 'a':
        drivingMode = 'a';
        break;
      case 's':
        drivingMode = 's';
        break;
      case 'm':
        gi_richtung = DatenAufnehmen(anzahlMesswerte);
        break;
      default:
        Serial.println("Fehler, kein Fahrmodus gewählt");
        break;
    }
  }
  switch (inDatC) {
    case 'a':
      Serial.println("Fahrmodus: Kontinuierlich");
      automode_continuous();
      break;
    case 's':
      Serial.println("Fahrmodus: Schritt");
      automode_steps();
      break;
    default:
      // automode_steps();
      break;
  }
}
void automode_steps() {
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

  float distFront = uss_v.distanceCm();
  while (distFront > minAbstandVHLR) {
    startzeit = millis();
    startDistanzHintenUss = uss_h.distanceCm();
    FahreDistanzCm(20, vTest ,'f');
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
    distFront = abstandsDaten[0];
  }

  delay(500);
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
void Initialisieren() {
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
void FahreVorwaerts(float v) {
  motorRechts.runSpeed(v);
  motorLinks.runSpeed(-v);
}


int DreheRoboterRunTurns(float winkel, char richtung) {
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
int DreheRoboterRunSpeed(float winkel, float vrpm, char richtung){
  float runtime; // delaytime
  float rps;
  float streckeRad = 0;
  float radUmdrehungen = 0;

  streckeRad = (winkel / 360) * PI * radAbstand;
  radUmdrehungen = 2*streckeRad / (radDurchmesser * PI);
  rps = vrpm/60;
  
  runtime = (radUmdrehungen/rps)*1000;

  Serial.print("Benötigte Umdrehungen für ");
  Serial.print(winkel);
  Serial.print(" ° = ");
  Serial.println(radUmdrehungen);
  
  Serial.print("Laufzeit bei ");
  Serial.print(streckeRad/10);
  Serial.print(" cm Distanz = ");
  Serial.print(runtime);
  Serial.println(" ms.");
  
  if (richtung == 'l') {
    motorRechts.runSpeed(vrpm);
    motorLinks.runSpeed(vrpm);
    delay(runtime);
  }
  if (richtung == 'r') {
    motorRechts.runSpeed(-vrpm);
    motorLinks.runSpeed(-vrpm);
    delay(runtime);
  }
  delay(100);
  Stop();
  
}
void FahreRueckwaerts(float v) { //anpassen: hole akt v und fahre rückwärts
  motorRechts.runSpeed(-v);
  motorLinks.runSpeed(v);
}

void Stop() {
  motorRechts.runSpeed(0);
  motorLinks.runSpeed(0);
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
  Serial.println("SENDE...");
  for (int i = 0; i < anzahlMesswerte; i++) {
    bt.print(abstandsDaten[i]);
    bt.print('\n');
    delay(20);
  }
  
  bt.print(30);
  bt.print('\n');
  delay(20);
  bt.print(40);
  bt.print('\n');
  delay(20);
}

void DreheSensoren(int winkel) {
  servoMotor.write(winkel);
  delay(200);
}

int PruefeUmgebung() {
  //Prüefe ob Vorwärtsfahrt möglich
  int neueRichtung = DatenAufnehmen(anzahlMesswerte);
  return neueRichtung;
}

void InitSensoren() {
  servoMotor.write(0);
  delay(100);
}

//TODOS:
/*  1. Wenn neue Abstandswert zur neuen Richtung größer 380: roboter muss dann auch den hinteren Sensor bei vorwärtsfahrt
       prüfen und bei distanz Sensor hinten == 380 cm stehen bleiben und neue Messung starten
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
  }*/
