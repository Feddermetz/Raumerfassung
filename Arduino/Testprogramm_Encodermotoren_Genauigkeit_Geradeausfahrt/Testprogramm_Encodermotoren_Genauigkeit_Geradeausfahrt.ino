/*
 *  Testprogramm Motor 
 *  
 * SLOT1: Motor Fahrtrichtung rechts 
 * SLOT2: Motor Fahrtrichtung links
 * 
 */

#include "MeBaseBoard.h"
#include <Wire.h>
#include <SoftwareSerial.h>

MeEncoderMotor motor1(0x09, SLOT1);   //  Motor rechts
MeEncoderMotor motor2(0x09, SLOT2);   //  Motor links

float testSpeed = 100;
float speedStop = 0;
float mPosition1 = 0;
float mPosition2 = 0;
float delayTime = 1000;
float runTime = 60000;

bool testende = false;



void setup()
{
  motor1.begin();
  motor2.begin();
  Serial.begin(9600);
  /*
  Serial.print("Start Motorposition1;Start Motorposition2;Ende Motorposition1;Ende Motorposition2;Gesamtumdrehungen Motor1 (rpm);Gesamtumdrehungen Motor2 (rpm)");
  Serial.println("");
  */
  // Test Motorumdrehungen zu Radumdrehungen
  testende = false; 
  Serial.print("Start Motorposition1;Ende Motorposition1;Gesamtumdrehungen Motor1 (rpm);Gesamtumdrehungen Motor2 (rpm)");
  Serial.println("");


}

void loop()
{
  // Motordaten sammeln: fkt: runSpeed mit unterschiedlichen Geschwindigkeiten. Ziel: Genauigkeit ermitten
  /*
  DatenerfassungRunSpeed(6000,50,1000);
  DatenerfassungRunSpeed(6000,100,1000);
  DatenerfassungRunSpeed(6000,150,1000);
  DatenerfassungRunSpeed(6000,200,1000);
  */  

  /* Test Drehung des Roboters
  
  if (testende == false) {
      TurnRobot(50,500);
      StopMotor(delayTime);
      testende = true;
  }*/

   /* Test runTurns: getriebeübersetzung rausfinden
    *   Wir messen start und endeposition des Motors nach fkt runTurns(43, 50);
    */
   float mposition;

  /*
   motor2.moveTo(0, 200);
   delay(2000);
   mposition = motor2.getCurrentPosition();
   Serial.print(mposition);
   Serial.print(";");

  
   motor2.move(360, 200);
   delay(2000);
   mposition = motor2.getCurrentPosition();
   Serial.print(mposition);
   Serial.println(";");
   */
   
   
   /*
   if (testende == false) {
      for (int i = 0; i<100;i++){ 
        motor2.reset();
        motor2.begin();
        mposition = motor2.getCurrentPosition();
        Serial.print(mposition);
        Serial.print(";");
        motor2.runTurns(8,200);
        delay(3000);
        mposition = motor2.getCurrentPosition();
        Serial.println(mposition);
      }
      testende = true;
  }*/

  if (testende == false) {
      for (int i = 0; i<100;i++){ 
        motor2.moveTo(0,50);
        delay(5000);
        mposition = motor2.getCurrentPosition();
        Serial.print(mposition);
        Serial.print(";");
        //motor2.moveTo(180,50);
        motor2.runTurns(0.021344,50);
        delay(5000);
        mposition = motor2.getCurrentPosition();
        Serial.println(mposition);
      }
      testende = true;
  
  } 

}
  
  
  // testRun2(100,1000);
  //testMoveWinkelV(90,100); //unbrauchbar, ungenau, regelung dauert teilweise ewig
  
  /* TEST Motorpositionen genau ansteuern START
  float mposition;
  
  mposition = motor2.getCurrentPosition();
  Serial.print("Start Motorposition = ");
  Serial.println(mposition);
  delay(1000);
  
  testRunTurns(1,100);

  mposition = motor2.getCurrentPosition();
  Serial.print("Ende  Motorposition = ");
  Serial.println(mposition);
  delay(1000);
 
  //stopMotor();
  motor2.reset();
  motor2.begin();
 //  TEST Motorpositionen genau ansteuern ENDE */
  
  /*
  float mposition;
  motor1.move(20,200);
  mposition = motor1.getCurrentPosition();
  Serial.print("aktuelle Motorposition = ");
  Serial.println(mposition);
  delay(2000);
  motor1.move(-20,200);
  mposition = motor1.getCurrentPosition();
  Serial.print("aktuelle Motorposition = ");
  Serial.println(mposition);
  delay(2000);
  */

void DatenerfassungRunSpeed(float runTime, float runSpeed, float delayTime){
  Serial.print("Test ");
  Serial.print(runSpeed);
  Serial.println(" rpm");
  for (int i1=0; i1<20;i1++){
  motor1.reset();
  motor2.reset();
  motor1.begin();
  motor2.begin();
  mPosition1 = motor1.getCurrentPosition();
  mPosition2 = motor2.getCurrentPosition();
  
  Serial.print(mPosition1);
  Serial.print(";");
  Serial.print(mPosition2);
  Serial.print(";");
  delay(delayTime);
  
  RunSpeed(runSpeed,runTime);
  StopMotor(delayTime);

  mPosition1 = motor1.getCurrentPosition();
  mPosition2 = motor2.getCurrentPosition();
  Serial.print(mPosition1);
  Serial.print(";");
  Serial.print(mPosition2);
  Serial.print(";");
  float gesamtUmdrehungenProMin = 0;

  gesamtUmdrehungenProMin = mPosition1 / (360); 
  Serial.print(gesamtUmdrehungenProMin);
  Serial.print(";");
  gesamtUmdrehungenProMin = mPosition2 / (360); 
  Serial.print(gesamtUmdrehungenProMin);
  Serial.println("");

  delay(delayTime);
  }
}

void TurnRobot(float v, float runTime){
    motor1.runSpeed(v);
    motor2.runSpeed(v);
    delay(runTime);
}

void RunSpeed(float v, float runTime){
    motor1.runSpeed(v);
    motor2.runSpeed(-v);
    delay(runTime);
}

void StopMotor(float delayTime){
  motor1.runSpeed(0);
  motor2.runSpeed(0);
  delay(delayTime);
}

void testRun1(float v, float v_stop, float delayTime){
  // Testet beide Motoren auf gleiche Geschwindigkeit
  motor1.runSpeed(v); //v_max = +-200 , höhere Werte werden automatisch auf 200 gesetzt, siehe Bibliothek
  motor2.runSpeed(-v);
}

void testRunSpeedAndTime(float v_run, float runTime){
  // Testet die Motoren auf gleiche Geschwindigkeit und Dauer
  motor1.runSpeedAndTime(v_run, runTime);
  motor2.runSpeedAndTime(-v_run, runTime);

  delay(runTime+100); //WICHTIG!!!!! 
}

void testMoveWinkelV(float winkel, float v){
 // testet 
  motor1.moveTo(winkel,v); //Dreht motor auf absolute Position 
  motor2.moveTo(winkel,v);
  delay(2000);
}
void testRunTurns(float turns, float v){
  //motor1.runTurns(turns,v); //Dreht motor auf absolute Position 
  motor2.runTurns(turns,v);
  delay(2000);
}
void testRun5(){
  
}
