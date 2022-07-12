
#include "MeBaseBoard.h"
#include <Wire.h>
#include <SoftwareSerial.h>

// Servo zum Drehen der Ultraschallsensoren
#define SERV_SIGN_PIN 10
Servo servoMotor;

void setup() {
  Serial.begin(115200);
  servoMotor.attach(SERV_SIGN_PIN, 900, 1500);

  servoMotor.write(0);

}

void loop() {
//  servoMotor.write(120); entspricht echten 90°

  for(int i = 0;i<180;i = i+5){
    servoMotor.write(i);
    delay(2000);
    Serial.println(servoMotor.read());
  }
  delay(5000);
}


void DreheSensoren(int winkel) {
  servoMotor.write(winkel);
  delay(1000);
}

void InitSensoren() {
  
  delay(5000);
}
