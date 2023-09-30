#include <ESP32Encoder.h>

// Encoder Pins
#define   M1_DT_PIN   32    // Motor-1 rotary encoder's DT pin
#define   M1_SW_PIN   33    // Motor-1 rotary encoder's SW pin
#define   M2_DT_PIN   25    // Motor-2 rotary encoder's DT pin
#define   M2_SW_PIN   26    // Motor-2 rotary encoder's SW pin


ESP32Encoder encoder1;     //8000 pulses per rotation
ESP32Encoder encoder2;     //8000 pulses per rotation

long M1_Pos, M2_Pos;

void setup () {
  encoder1.attachHalfQuad(M1_DT_PIN, M1_SW_PIN);
  encoder2.attachHalfQuad(M2_DT_PIN, M2_SW_PIN);
  
  encoder1.setCount(0);
  encoder2.setCount(0);
  Serial.begin(115200);
}

void loop () {
  long M1_Pos = encoder1.getCount();
  long M2_Pos = encoder2.getCount();
  
  Serial.println(M1_Pos);
  Serial.println(M2_Pos);
  Serial.println();
  delay(25);
}
