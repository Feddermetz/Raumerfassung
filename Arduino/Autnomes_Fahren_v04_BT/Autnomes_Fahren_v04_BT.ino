
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
MeUltrasonicSensor uss_v(6); // Ultraschallsensor vorne und hinten
MeUltrasonicSensor uss_h(7); // Ultraschallsensor links und rechts

// Bluetoothmodul
MeBluetooth bt(PORT_4);


const int anzahlMesswerte = 12;// die Sensoren haben einen Messwinkel von ca.30°, somit ergeben sich 360/30 = 12 Messwerte pro Messung
const float messwinkel = 30;                             
const float minAbstandVhCm = 30;
const float minAbstandLrCm = 30;

const float radAbstand = 181; //Abstand Radmitte zu Radmitte
const float radDurchmesser = 68.5;

float abstandsDaten[anzahlMesswerte];
float vTest = 50;
float vStop = 0;
int anzahlMessungenBisher = 99; //Zähler Anzahl Messreihen
int gi_richtung = 99;

int startzeit, endzeit, gefahreneZeit;
float startDistanzHintenUss, gefahreneDistanzUss;
float startPosMotorRechts, startPosMotorLinks,gefahreneDistanzMotorRechtsCm, gefahreneDistanzMotorLinksCm;

// für bluetoothsteuerung
const byte vorwaerts = 0xfa;
const byte rueckwaerts = 0xfb;
const byte links = 0xfc;
const byte rechts = 0xfd;
const byte automatik = 0xfe; 
const byte messen = 0xff;

bool pruefeHinterenSensor;
bool btMode = false;

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
  if(bt.available()){
    Stop();
    btMode = true;
    inDat = bt.read();
    inDatC = (char)inDat;
    Serial.println(inDat,HEX);
    Serial.println(inDat, BIN); //,HEX);
    Serial.println(inDatC); //,HEX);
    
    Initialisieren();
    
    switch (inDat){
      case 'f':
        FahreDistanzCm(10, 'f');
        break;
      case 'b':
        FahreDistanzCm(10, 'b');
        break;
      case 'l':
        dreheRoboter(messwinkel, 'l');
        break;
      case 'r':
        dreheRoboter(messwinkel, 'r');
        break;
      case automatik:
        automode();
        break;
      case messen:
        // Messdaten aufnehmen.
        gi_richtung = DatenAufnehmen(anzahlMesswerte);
        break;
      default:
        // warten auf anweisung
        break;
    }

    if (inDat != messen){
      gi_richtung = DatenAufnehmen(anzahlMesswerte);
    }
  }
  else{
    btMode = false;
    automode();
  }
}
void automode(){
  Initialisieren();
  
  //Fahre Vorwärts bis es nicht mehr geht
  startzeit = millis();
  startDistanzHintenUss = uss_h.distanceCm();
  //FahreVorwaerts(vTest);

  Stop();
  gefahreneDistanzUss = uss_h.distanceCm() - startDistanzHintenUss;
  gefahreneDistanzMotorRechtsCm = (motorRechts.getCurrentPosition()/(2*360))*radDurchmesser* PI * 10; 
  gefahreneDistanzMotorLinksCm = (motorLinks.getCurrentPosition()/(2*360))*radDurchmesser* PI * 10;
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
void Initialisieren(){
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
void FahreDistanzCm(float distanz, float richtung){
  
  float radUmdrehungen = 0;
  distanz = distanz * 10; // in mm umrechnen
  radUmdrehungen = distanz/(radDurchmesser*PI);

  if (richtung == 'f'){
    motorRechts.runTurns(2*radUmdrehungen,-50);
    motorLinks.runTurns(2*radUmdrehungen,50);
  }
  if (richtung == 'b') {
    motorRechts.runTurns(2*radUmdrehungen,50);
    motorLinks.runTurns(2*radUmdrehungen,-50);
  }
  delay(5000);
}
void FahreVorwaerts(float v){
  if (btMode == true){
    // TODO Hier überlegen was sinnvoll ist (vorwärtsfahrt für bestimmte distanz oder dauer oder oder oder
  }
  else{
    motorRechts.runSpeed(v);
    motorLinks.runSpeed(-v);
  } 
}

int dreheRoboter(float winkel, char richtung){ 
  
  float streckeRad = 0;
  float radUmdrehungen = 0;
    
  streckeRad= (winkel/360) * PI * radAbstand;
  radUmdrehungen = streckeRad/(radDurchmesser*PI);

  if (richtung == 'l'){
    Serial.println("Drehe Roboter links");
    motorRechts.runTurns(2*radUmdrehungen,-50);
    motorLinks.runTurns(2*radUmdrehungen,-50);
  }
  if (richtung == 'r'){
    Serial.println("Drehe Roboter rechts");
    motorRechts.runTurns(2*radUmdrehungen,50);
    motorLinks.runTurns(2*radUmdrehungen,50);
  }
  delay(5000);
  return 0;   
}
void FahreRueckwaerts(float v){ //anpassen: hole akt v und fahre rückwärts
  if (btMode == true){
    // Hier überlegen was sinnvoll ist (vorwärtsfahrt für bestimmte distanz oder dauer oder oder oder
  }
  else{  
    motorRechts.runSpeed(-v);
    motorLinks.runSpeed(v);
  }
}

void Stop(){
  motorRechts.runSpeed(0);
  motorLinks.runSpeed(0);
}

int DatenAufnehmen(int anzahlMesswerte){ //nimmt Dten auf und gibt zurück wo es als nächstes hingeht
  Stop(); //Anhalten, wir messen erstmal nicht während der Fahrt
  int anzahlMessungen = anzahlMesswerte/2;
  int winkelSchritt = 360/(anzahlMesswerte-2);
  int winkel = 0;  
  InitSensoren();

  // Daten aufnehmen und in Array schreiben
  
  for (int i=0; i<anzahlMessungen;i++){
    abstandsDaten[i]= uss_v.distanceCm();
    abstandsDaten[i+anzahlMessungen] = uss_h.distanceCm();
    winkel = i*winkelSchritt;
    DreheSensoren(winkel);
    delay(500); 
  }

  //Ausgabe, merken der nächsten Fahrtrichtung
  int neueRichtung = 99; //wir merken uns gleich den Index des größten Abstandswert, dieser gibt die nächste Richtung vor
  float maxDistanz = 0;
  for (int i=0; i<anzahlMesswerte;i++){
    Serial.println(abstandsDaten[i]);   
   
    //Idee: wir fahren als nächstes in die Rictung mit dem größten abstand zu einem Objekt, ausgenommen der Bereiche 0° bis 60° und 300° bis 360°. Gilt nicht bei erster Messreihe
    if (anzahlMessungenBisher != 0){
      if (i>=3 && i<=9){//bei diesen Messwerten interessieren die Abstände für die neue Richtung nicht
        if (abstandsDaten[i] > maxDistanz){
          maxDistanz = abstandsDaten[i];
          neueRichtung = i;//i mal Winkelschrittweite ergibt neue Richtung
        }   
      } 
    }
    else { // Bei der ersten Messung kann der Roboter auch in die oben ausgeschlossenen Bereichen losfahren
      if (abstandsDaten[i] >  maxDistanz){
          maxDistanz = abstandsDaten[i];
          neueRichtung = i;//i mal Winkelschrittweite ergibt neue Richtung
        }   
    }
   
  }
  
  if (maxDistanz >= 380){ //Merker Setzen
    pruefeHinterenSensor = true;     
  }
  
  InitSensoren();
  anzahlMessungenBisher++; // TODO: Den Wert optional auch in das Array einbauen
  int ok_senden = SendeDaten();
  if (ok_senden && (neueRichtung != 99)){
    return neueRichtung; // Alles super, wir geben die neue Richtung zurück
  }
  return -1;  
}

int SendeDaten(){
  bool error = false;
  
  Serial.println("SENDE...");  
  for (int i=0; i<anzahlMesswerte;i++){
    bt.print(abstandsDaten[i]);
    bt.print('\n');
  }
  
  bt.print(30);
  bt.print('\n');
  bt.print(40);
  bt.print('\n');
    
  if (error) {
    return -1;
  }
  return 1; //alles ok
}

void DreheSensoren(int winkel){
  servoMotor.write(winkel);
  delay(500);
}

int PruefeUmgebung(){
  //Prüefe ob Vorwärtsfahrt möglich
  int neueRichtung = DatenAufnehmen(anzahlMesswerte);
  return neueRichtung;
}

void InitSensoren(){
  servoMotor.write(0);
  delay(100);
}

//TODOS:
/*  1. Wenn neue Abstandswert zur neuen Richtung größer 380: roboter muss dann auch den hinteren Sensor bei vorwärtsfahrt 
 *     prüfen und bei distanz Sensor hinten == 380 cm stehen bleiben und neue Messung starten
 *     ALTERNATIV: nie in richtung >380/400 fahren. ist aber problematisch in großem Raum ##################erledigt
 *  2. abstand in allen modi mittels drehzahlen der räder berechnen. initialisieren() bereits gemacht
 */

// LOG
/* 120422Minimalabstand geändert auf 30 cm 
 *  
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
