#include <ESP32Encoder.h>

#define   LED         2     // Onboard LED pin

// Encoder Pins
#define   M1_DT_PIN   32    // Motor-1 rotary encoder's DT pin
#define   M1_SW_PIN   33    // Motor-1 rotary encoder's SW pin
#define   M2_DT_PIN   25    // Motor-2 rotary encoder's DT pin
#define   M2_SW_PIN   26    // Motor-2 rotary encoder's SW pin


// Motor Driver Pins
#define   M1_dir      12    // Motor-1 Direction pin (connected through optocoupler)
#define   M1_pwm      13    // Motor-1 PWM pin (connected through optocoupler)
#define   M2_dir      27    // Motor-2 Direction pin (connected through optocoupler)
#define   M2_pwm      14    // Motor-2 PWM pin (connected through optocoupler)

// Motor & Encoder Parameters
#define   KVA         3.7
#define   ENC_PULSE   8000

// Manipulator Position Parameter
double t = 0, i = 0, j = 0 ;

// PWM channel Parameters
const int freq = 50;
const int ch1 = 0;
const int ch2 = 1;
const int resolution = 8;
int pwm = 40;

// PID Parameter
float e = 0;    // error between the desired output and the reading

double sensed_output, control_signal;
double setpoint;
double Kp; //proportional gain
double Ki; //integral gain
double Kd; //derivative gain
int T; //sample time in milliseconds (ms)
unsigned long last_time;
double total_error, last_error;
int max_control;
int min_control;

ESP32Encoder encoder1;     //8000 pulses per rotation
ESP32Encoder encoder2;     //8000 pulses per rotation

void setup() {
  Serial.begin(115200);
  
  pinMode(LED, OUTPUT);
  pinMode(M1_dir, OUTPUT);
  pinMode(M2_dir, OUTPUT);

  
  encoder1.attachHalfQuad(M1_DT_PIN, M1_SW_PIN);
  encoder2.attachHalfQuad(M2_DT_PIN, M2_SW_PIN);
  encoder1.setCount(0);
  encoder2.setCount(0);

  ledcSetup(ch1, freq, resolution);
  ledcSetup(ch2, freq, resolution);
  ledcAttachPin(M1_pwm, ch1);
  ledcAttachPin(M2_pwm, ch2);

  
  digitalWrite(M1_dir, LOW);
  digitalWrite(M2_dir, LOW);
  
  // Motor PWM
  ledcWrite(ch1, 0);   
  ledcWrite(ch2, 0); 

  delay(2000);
  Serial.println("Initialized...");

}

void loop() {

  FK(i, j);

  if (3000+t < millis()) {
    t = millis();
    
    i += 15;
    j += 45;
    
    //Serial.print(i); 
    //Serial.print(j); 
  }
  delay(25);
  Serial.println(); 
}

void FK(float q1, float q2) {
  Pos_control_v1(1, q1);
  Pos_control_v1(2, q2);
}

float M1_encoder() {
  long M1_Pos = encoder1.getCount();
  return -360*M1_Pos/8000;
}
float M2_encoder() {
  long M2_Pos = encoder2.getCount();
  return -360*M2_Pos/8000;
}

void Pos_control_v1(int link, float q) {
  float mot, dir, pos;
  if (link == 1) {
    mot = ch1;
    dir = M1_dir;
    pos = M1_encoder();
  } else if (link == 2) {
    mot = ch2;
    dir = M2_dir;
    pos = M2_encoder();
  } else return;
  
  Serial.print("Link-");
  Serial.print(link);
  Serial.print(" Position: ");
  Serial.println(pos);

  e = abs(q-pos);

  if (e > 40) {
    ledcWrite(mot, 70);  
  } else if (e > 5) {
    ledcWrite(mot, e+30);  
  } else {
    ledcWrite(mot, 0);  
  }
  if (q>pos) {
    digitalWrite(dir, LOW); 
    
    Serial.println("Increse...");
  } else if (q<pos) {
    digitalWrite(dir, HIGH);
    
    Serial.println("Decrese...");
  } else {
    ledcWrite(mot, 0);   
  }

  //pwm -= 0.01;
  Serial.println(e);
}

void Pos_PID_Control(float q) {
  unsigned long current_time = millis(); //returns the number of milliseconds passed since the Arduino started running the program
  
  int delta_time = current_time - last_time; //delta time interval 
  
  if (delta_time >= T){
  
    double error = setpoint - sensed_output;
    
    total_error += error; //accumalates the error - integral term
    if (total_error >= max_control) total_error = max_control;
    else if (total_error <= min_control) total_error = min_control;
    
    double delta_error = error - last_error; //difference of error for derivative term
    
    control_signal = Kp*error + (Ki*T)*total_error + (Kd/T)*delta_error; //PID control compute
    if (control_signal >= max_control) control_signal = max_control;
    else if (control_signal <= min_control) control_signal = min_control;
    
    last_error = error;
    last_time = current_time;
  } 
}
