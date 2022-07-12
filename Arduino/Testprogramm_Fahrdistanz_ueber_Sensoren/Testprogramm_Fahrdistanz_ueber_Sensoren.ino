/*  Testprogramm zur Ermittlung der gefahrenen Distanz über Sensordaten
    Dafür: Umbau der Sensoren auf Messrichtung vorne und hinten.
    
 */
#include "MeBaseBoard.h"
#include <Wire.h>
#include <SoftwareSerial.h>

// Motoren 
MeEncoderMotor motor1(0x09, SLOT1);   //  Motor rechts
MeEncoderMotor motor2(0x09, SLOT2);   //  Motor links

// Ultraschallsenoren 
MeUltrasonicSensor uss_v(6); //Ultraschallsensor vorne und hinten
MeUltrasonicSensor uss_h(7); //Ultraschallsensor links und rechts

const float minAbstandCm = 20;
const float minAbstandVCm = 20;
const float minAbstandHCm = 20;

float abstandHinten = 0;
float abstandVorne = 0;
float gefahreneDistanz = 0;

void setup() {
  // put your setup code here, to run once:
  motor1.begin();
  motor2.begin();
  Serial.begin(9600);
//  startAbstandHinten = uss_h.distanceCm();
  //startAbstandVorne = 
  //versuch ende, funktioniert nur in Raum mit max. größe kleiner als sensormessbereich.
}

void loop() {
  // put your main code here, to run repeatedly:
  if (uss_v.distanceCm() > minAbstandVCm) {
    motor1.runSpeed(50);
    motor2.runSpeed(-50);
  }
  else {
    motor1.runSpeed(0);
    motor2.runSpeed(0);
    abstandVorne = uss_v.distanceCm();
    gefahreneDistanz = abstandHinten
  }
}
