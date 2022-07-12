
#include "MeBaseBoard.h"
#include <Wire.h>
#include <SoftwareSerial.h>

#define SERV_SIGN_PIN 10

Servo servoMotor;

// Motoren 
MeEncoderMotor motor1(0x09, SLOT1);   //  Motor rechts
MeEncoderMotor motor2(0x09, SLOT2);   //  Motor links

// Ultraschallsenoren 
MeUltrasonicSensor uss_vh(6); //Ultraschallsensor vorne und hinten
MeUltrasonicSensor uss_lr(7); //Ultraschallsensor links und rechts

const float minAbstandVhCm = 20;
const float minAbstandLrCm = 20;

float vTest = 50;
float vStop = 0;
  
void setup() {

  motor1.begin();
  motor2.begin();
  Serial.begin(9600);
}
void loop() {
  char hindernisErkannt = 'N';
  int startzeit, endzeit, gefahreneZeit;
  
  startzeit = micros();
  
  Vorwaerts(vTest);
  hindernisErkannt = PruefeAbstaende();

  if (hindernisErkannt != 'N'){
    Stop();
    gefahreneZeit = startzeit - micros();
    if (hindernisErkannt == 'V'){
      DreheNachRechts();
    }
    if (hindernisErkannt == 'L'){
      DreheNachRechts();
    }
    if (hindernisErkannt == 'H'){
      DreheNachRechts();
    }
    if (hindernisErkannt == 'R'){
      DreheNachLinks();
    }
  }

  Serial.print("gefahrene Zeit in Microsekunden = ");
  Serial.println(gefahreneZeit);
  
  SendeDaten();
}

char PruefeAbstaende(){
   //Nehme die Daten der Sensoren auf für vorne und links
  if (uss_vh.distanceCm()< minAbstandVhCm){
    return 'V';
  }
  if (uss_lr.distanceCm()< minAbstandLrCm){
    return 'L';
  }
  
  /* Für Später wenn der Servo da ist
  DreheSensoren();

  //Nehme die ersten Daten der Sensoren auf für hinten und rechts
  if (uss_vh.distanceCm()< minAbstandVhCm){
    return 'H';
  }
  if (uss_lr.distanceCm()< minAbstandLrCm){
    return 'R';
  }
  */
  return 'Z';

}
void Vorwaerts(float v){
  motor1.runSpeed(v);
  motor2.runSpeed(-v);
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

int DatenAufnehmen(){ //nimmt Dten auf und gibt zurück ob und wo ein Hindernis ist
  bool hindernisLinks = false, hindernisRechts = false, hindernisVorne = false, hindernisHinten = false;
  
  Stop(); //Anhalten, wir messen erstmal nicht während der Fahrt
  
  //Nehme die ersten Daten der Sensoren auf für vorne und links
  if ((uss_vh.distanceCm()< minAbstandVhCm)){
    hindernisVorne = true;
  }
  if ((uss_lr.distanceCm()< minAbstandLrCm)){
    hindernisLinks = true;
  }

  DreheSensoren();
  
  //Nehme die zweiten Daten der Sensoren auf für vorne und links
  if ((uss_vh.distanceCm()< minAbstandVhCm)){
    hindernisHinten = true;
  }
  if ((uss_lr.distanceCm()< minAbstandLrCm)){
    hindernisRechts = true;
  }

  //entsprechenden Rückgabewert zurückgeben
  if (!hindernisVorne && !hindernisLinks && !hindernisHinten && !hindernisRechts)
    {return 0;}
  else if (!hindernisVorne && !hindernisLinks && !hindernisHinten && hindernisRechts)
    {return 1;}
  else if (hindernisVorne && hindernisLinks && !hindernisHinten && hindernisRechts)
    {return 2;}
  else if (hindernisVorne && hindernisLinks && !hindernisHinten && !hindernisRechts)
    {return 3;}  
  else if (hindernisVorne && !hindernisLinks && hindernisHinten && hindernisRechts)
    {return 4;}
  else if (hindernisVorne && !hindernisLinks && hindernisHinten && !hindernisRechts)
    {return 5;}
  else if (hindernisVorne && !hindernisLinks && !hindernisHinten && hindernisRechts)
    {return 6;}
  else if (hindernisVorne && !hindernisLinks && !hindernisHinten && !hindernisRechts)
    {return 7;}
  else if (!hindernisVorne && hindernisLinks && hindernisHinten && hindernisRechts)
    {return 8;}  
  else if (!hindernisVorne && hindernisLinks && hindernisHinten && !hindernisRechts)
    {return 9;}
  else if (!hindernisVorne && hindernisLinks && !hindernisHinten && hindernisRechts)
    {return 10;}
  else if (!hindernisVorne && hindernisLinks && !hindernisHinten && !hindernisRechts)
    {return 11;}
  else if (!hindernisVorne && !hindernisLinks && hindernisHinten && hindernisRechts)
    {return 12;}
  else if (!hindernisVorne && !hindernisLinks && hindernisHinten && !hindernisRechts)
    {return 13;}  
  else if (!hindernisVorne && !hindernisLinks && !hindernisHinten && hindernisRechts)
    {return 14;}
  else if (!hindernisVorne && !hindernisLinks && !hindernisHinten && !hindernisRechts)
    {return 15;}
  else {return -1;}//sonstiger Fehler
}

int SendeDaten(){
  bool error = false;
  
  //Hier Daten senden
  
  if (error) {
    return -1;
  }
  return 0;
}



void DreheSensoren(){
}
