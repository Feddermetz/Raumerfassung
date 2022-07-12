
#include "MeBaseBoard.h"
#include <Wire.h>
#include <SoftwareSerial.h>


// Servo zum Drehen der Ultraschallsensoren
#define SERV_SIGN_PIN 10
Servo servoMotor;

// Motoren 
MeEncoderMotor motor1(0x09, SLOT1);   //  Motor rechts
MeEncoderMotor motor2(0x09, SLOT2);   //  Motor links

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

bool pruefeHinterenSensor;

void setup() {
  initialisieren();
  motor1.begin();
  motor2.begin();
  Serial.begin(9600);
  bt.begin(9600);
  servoMotor.attach(SERV_SIGN_PIN, 900, 1500);

  anzahlMessungenBisher = 0;
  gi_richtung = PruefeUmgebung(); // Anfangswerte Aufnehmen
  pruefeHinterenSensor = false;
}

void loop() {
  //char hindernisErkannt = 'N'; 
  int startzeit, endzeit, gefahreneZeit;

  //Wir bringen den Roboter in Position
  
  Serial.print("neue Richtung = ");
  Serial.println(gi_richtung*(360/anzahlMesswerte));
  if (gi_richtung != 99 && gi_richtung != 99 ){
    for(int j = 0; j<gi_richtung;j++){
      dreheRoboter(messwinkel);
    }  
  }
  //Fahre Vorwärts bis es nicht mehr geht
  startzeit = millis();
  Vorwaerts(vTest);
  if (pruefeHinterenSensor == true){ // Neue Richtung hatte eine Distanz die auf kein Objekt im Weg hinweist. 
     while (uss_h.distanceCm()<380){ // Wir benötogen also einen neuen Messpunkt nach ca. 380cm fahrt
      Serial.print("Prüfe hinteren sensor: ");
      Serial.println(uss_h.distanceCm());
     }
     Serial.print("Prüfe hinteren sensor: ");
     Serial.println(uss_h.distanceCm());
     pruefeHinterenSensor = false;
  }
  else { //wir fahren bis wir auf ein hindernis stoßen
    while (uss_v.distanceCm()> minAbstandVhCm){ 
     //Serial.println(uss_v.distanceCm()); 
    }
  }
  
  Stop();
  gefahreneZeit = millis() - startzeit ;    
  gi_richtung = PruefeUmgebung();
  Serial.print("gefahrene Zeit in Microsekunden = ");
  Serial.println(gefahreneZeit);
  delay(500);
  
}
void initialisieren(){
 
}
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
}
void Vorwaerts(float v){
  motor1.runSpeed(v);
  motor2.runSpeed(-v);
}
void DreheRoboter30(){
  motor1.runSpeed(vTest);
  motor2.runSpeed(vTest);
  delay(500);
  Stop;
}

int dreheRoboter(float winkel){ 
  //Roboter dreht nur in eine Richtung, da negative werte bei runturns seltsames verhalten auslösen 
  //TODO: Prüfen warum
  float streckeRad = 0;
  float radUmdrehungen = 0;
  
  streckeRad= (winkel/360) * PI * radAbstand;
  radUmdrehungen = streckeRad/(radDurchmesser*PI);
  motor1.runTurns(2*radUmdrehungen,50);
  motor2.runTurns(2*radUmdrehungen,50);

  delay(5000);
  return 0;   
}
void Rueckwaerts(float v){ //anpassen: hole akt v und fahre rückwärts
  motor1.runSpeed(-v);
  motor2.runSpeed(v);
}
void DreheNachLinks(){
  motor1.runSpeed(50);
  motor2.runSpeed(50);
  //delay(250);
  //Stop();
}
void DreheNachRechts(){
  motor1.runSpeed(-50);
  motor2.runSpeed(-50);
  //delay(250);
  //Stop();
}
void Stop(){
  motor1.runSpeed(0);
  motor2.runSpeed(0);
}

int DatenAufnehmen(int anzahlMesswerte){ //nimmt Dten auf und gibt zurück ob und wo ein Hindernis ist
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

    if (maxDistanz >= 380){ //Merker Setzen
      pruefeHinterenSensor = true;     
    }
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
  
  //Hier Daten senden
  
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
 *     ALTERNATIV: nie in richtung >380/400 fahren. ist aber problematisch in großem Raum
 * 
 */

// LOG
/* 120422Minimalabstand geändert auf 30 cm 
 *  
 */ 
