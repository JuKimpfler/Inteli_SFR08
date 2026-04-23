#include "Arduino.h"
#include "Wire.h"


void scanI2C(long frequency){
  String normal = "standard mode (100 kHz):";
  String fast = "fast mode (400 kHz):";
  String fastPlus = "fast mode plus (1 MHz):";
  String highSpeed = "high speed mode (3.4 MHz):";
  String ultraSpeed = "ultra fast mode (5.0 MHz):";
  String defaultStr = " !!!!! Unzulässige Frequenz !!!!!";
  bool error = true;
  bool addressFound = false;
  Serial.print("Scanne im ");
  switch(frequency){
    case 100000:
      Serial.println(normal);
      break;
    case 400000:
      Serial.println(fast);
      break;
    case 1000000:
      Serial.println(fastPlus);
      break;
    case 3400000:
      Serial.println(highSpeed);
      break;
    case 5000000:
      Serial.println(ultraSpeed);
      break;
    default:
      Serial.println(defaultStr);
      break;
  }
  
  Wire1.setClock(frequency);
  for(int i=1; i<128; i++){
    Wire1.beginTransmission(i);
    error = Wire1.endTransmission();
    if(error == 0){
      addressFound = true;
      Serial.print("0x");
      Serial.println(i,HEX);
    }
  }
  if(!addressFound){
    Serial.println("Keine Adresse erkannt");
  }
  Serial.println();
}

void setup(){
  Wire1.begin();
  Serial.begin(115200);
  Serial.println("I2C Scanner ist bereit.");
  Serial.println();
}
void loop() {
  scanI2C(100000);
  scanI2C(400000);
  scanI2C(1000000); // nur aktivieren, wenn der Microcontroller diese Frequenz unterstützt
//  scanI2C(3400000); // nur aktivieren, wenn der Microcontroller diese Frequenz unterstützt
//  scanI2C(5000000); // nur aktivieren, wenn der Microcontroller diese Frequenz unterstützt
  
  Serial.println("****************************");
  Serial.println();
  delay(3000);
}