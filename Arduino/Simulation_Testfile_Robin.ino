// @author: Robin Justinger

#include "MeBaseBoard.h"
#include <SoftwareSerial.h>

//RJ-Stecker Nummer 4 auf dem Board
MeBluetooth bluetooth(PORT_4);

int data_to_send[77];
char command;

void setup()
{
  Serial.begin(115200);
  bluetooth.begin(115200);    //The factory default baud rate is 115200

  //Initialisierung der Daten mit den Werten des Testraums von Jan
  
  data_to_send[0] = 1;
  data_to_send[1] = 1621;
  data_to_send[2] = 1334;
  data_to_send[3] = 555;
  data_to_send[4] = 557;
  data_to_send[5] = 568;
  data_to_send[6] = 577;
  data_to_send[7] = 584;
  data_to_send[8] = 647;
  data_to_send[9] = 651;
  data_to_send[10] = 665;
  data_to_send[11] = 1364;
  data_to_send[12] = 1358;
  data_to_send[13] = 1371;
  data_to_send[14] = 1348;
  data_to_send[15] = 1388;
  data_to_send[16] = 1621;
  data_to_send[17] = 1628;
  data_to_send[18] = 1622;
  data_to_send[19] = 1626;
  data_to_send[20] = 1622;
  data_to_send[21] = 1623;
  data_to_send[22] = 1623;
  data_to_send[23] = 1628;
  data_to_send[24] = 1627;
  data_to_send[25] = 1626;
  data_to_send[26] = 1632;
  data_to_send[27] = 1632;
  data_to_send[28] = 1644;
  data_to_send[29] = 1647;
  data_to_send[30] = 1668;
  data_to_send[31] = 1379;
  data_to_send[32] = 1739;
  data_to_send[33] = 1327;
  data_to_send[34] = 1208;
  data_to_send[35] = 359;
  data_to_send[36] = 346;
  data_to_send[37] = 122;
  data_to_send[38] = 176;
  data_to_send[39] = 331;
  data_to_send[40] = 363;
  data_to_send[41] = 343;
  data_to_send[42] = 377;
  data_to_send[43] = 217;
  data_to_send[44] = 401;
  data_to_send[45] = 192;
  data_to_send[46] = 191;
  data_to_send[47] = 176;
  data_to_send[48] = 171;
  data_to_send[49] = 171;
  data_to_send[50] = 167;
  data_to_send[51] = 164;
  data_to_send[52] = 156;
  data_to_send[53] = 163;
  data_to_send[54] = 163;
  data_to_send[55] = 163;
  data_to_send[56] = 163;
  data_to_send[57] = 168;
  data_to_send[58] = 164;
  data_to_send[59] = 173;
  data_to_send[60] = 178;
  data_to_send[61] = 176;
  data_to_send[62] = 188;
  data_to_send[63] = 207;
  data_to_send[64] = 192;
  data_to_send[65] = 284;
  data_to_send[66] = 639;
  data_to_send[67] = 236;
  data_to_send[68] = 615;
  data_to_send[69] = 605;
  data_to_send[70] = 609;
  data_to_send[71] = 551;
  data_to_send[72] = 554;
  data_to_send[73] = 0;
  data_to_send[74] = 0;
  data_to_send[75] = 0;
  data_to_send[76] = 0;
}

void loop()
{ 
  if (bluetooth.available())
  {
    Serial.print("Bluetooth Available: Es kommt was vom Programm an: ");
    command = (char)bluetooth.read();
    Serial.println(command);
    delay(100);
  }
  for (int i=0;i<77;i++)
  {
    bluetooth.println(data_to_send[i]);
    delay(10);
  }
  bluetooth.print('\n');
  delay(5000);
}
