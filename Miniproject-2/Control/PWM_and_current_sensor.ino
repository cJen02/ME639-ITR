
int pin = 2;

int M2_dir = 27;
int M2_pwm = 14;
int M1_dir = 12;
int M1_pwm = 13;

const int M1_cs = 35;
const int M2_cs = 34;

const int freq = 50;
const int ch1 = 0;
const int ch2 = 1;
const int resolution = 8;

int mVperAmp = 185;           // this the 5A version of the ACS712 -use 100 for 20A Module and 66 for 30A Module
int Watt = 0;
double Voltage = 0;
double VRMS = 0;
double AmpsRMS = 0;

int M1_ar, M2_ar;

void setup() {
  Serial.begin(115200);
  
  pinMode(pin, OUTPUT);
  pinMode(M1_dir, OUTPUT);
  pinMode(M2_dir, OUTPUT);

  ledcSetup(ch1, freq, resolution);
  ledcSetup(ch2, freq, resolution);
  
  ledcAttachPin(M1_pwm, ch1);
  ledcAttachPin(M2_pwm, ch2);

  digitalWrite(M1_dir, LOW);
  digitalWrite(M2_dir, LOW);

  // Motor PWM
  ledcWrite(ch1, 0);   
  ledcWrite(ch2, 25); 

  delay(2000);
  Serial.println("Initialized...");
}

void loop() {
  digitalWrite(pin, LOW);
  delay(100);
  digitalWrite(pin, HIGH);
  delay(100);
  
  //ledcWrite(ch2, 25); 

  M1_ar = analogRead(M1_cs);
  //M2_ar = analogRead(M2_cs);
  
  Voltage = getVPP(M1_cs);
  VRMS = (Voltage/2.0) *0.707;   //root 2 is 0.707
  AmpsRMS = ((VRMS * 1000)/mVperAmp)-1.5; //0.3 is the error I got for my sensor

  
  Serial.println(M1_ar);
  Serial.println(AmpsRMS);
  Serial.println();

}


float getVPP(int sensorIn) {
  float result;
  int readValue;                // value read from the sensor
  int maxValue = 0;             // store max value here
  int minValue = 4096;          // store min value here ESP32 ADC resolution
  
   uint32_t start_time = millis();
   while((millis()-start_time) < 1000) //sample for 1 Sec
   {
       readValue = analogRead(sensorIn);
       // see if you have a new maxValue
       if (readValue > maxValue) 
       {
           /*record the maximum sensor value*/
           maxValue = readValue;
       }
       if (readValue < minValue) 
       {
           /*record the minimum sensor value*/
           minValue = readValue;
       }
   }
   
   // Subtract min from max
   result = ((maxValue - minValue) * 3.3)/4096.0; //ESP32 ADC resolution 4096
      
   return result;
}
