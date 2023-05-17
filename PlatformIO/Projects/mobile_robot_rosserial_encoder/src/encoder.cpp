#include "encoder.h"

using namespace encoder;

void encoder::countLeft(){
  leftWheelTickCount.data++;
}

void encoder::countRight(){
  rightWheelTickCount.data++;
}

void encoder::setupPins(const uint8_t leftPin, const uint8_t rightPin){
  pinMode(leftPin, INPUT_PULLUP);
  pinMode(rightPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(leftPin), countLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(rightPin), countRight, RISING);
}




const uint8_t R_PWM = 14;
const uint8_t R_BACK = 33;
const uint8_t R_FORW = 25;
const uint8_t L_BACK = 26;
const uint8_t L_FORW = 27;
const uint8_t L_PWM = 18;

const uint8_t encoder_right = 12;
const uint8_t encoder_left = 13 ;
unsigned int rpm_right;
unsigned int rpm_left;
volatile byte pulses_right;
volatile byte pulses_left;
unsigned long TIME;
unsigned int pulse_per_turn = 20;

int PWM_MIN = 100;
int PWMRANGE = 255;





void cmd(const geometry_msgs::Twist &msg);
float mapPwm(float x, float out_min, float out_max);









void cmd(const geometry_msgs::Twist &msg){
  float l = (msg.angular.z - msg.linear.x) / 2;
  float r = (msg.angular.z + msg.linear.x) / 2;

  uint16_t lPwm = mapPwm(fabs(l), PWM_MIN, PWMRANGE);
  uint16_t rPwm = mapPwm(fabs(r), PWM_MIN, PWMRANGE);

   // Set direction pins and PWM
  digitalWrite(L_FORW, l > 0);
  digitalWrite(L_BACK, l < 0);
  digitalWrite(R_FORW, r > 0);
  digitalWrite(R_BACK, r < 0);
  analogWrite(L_PWM, lPwm);
  analogWrite(R_PWM, rPwm);
}

float mapPwm(float x, float out_min, float out_max){
  return x * (out_max - out_min) + out_min;
}




