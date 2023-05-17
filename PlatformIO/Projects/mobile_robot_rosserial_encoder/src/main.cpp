#include <Arduino.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16.h>

#define ROSSERIAL_ARDUINO_TCP


const uint8_t R_PWM = 14;
const uint8_t R_BACK = 33;
const uint8_t R_FORW = 25;
const uint8_t L_BACK = 26;
const uint8_t L_FORW = 27;
const uint8_t L_PWM = 18;

const int interval = 30;
long previousMillis = 0;
long currentMillis = 0;



IPAddress server(192,168,178,36);
uint16_t serverPort = 11411;

void setupWiFi(const char*  ssid, const char*  password);
void countLeft();
void countRight();
void setupPins(const uint8_t leftPin, const uint8_t rightPin);

ros::NodeHandle nh;

std_msgs::Int16 leftWheelTickCount;
std_msgs::Int16 rightWheelTickCount;
ros::Publisher rightPub("right_ticks", &rightWheelTickCount);
ros::Publisher leftPub("left_ticks", &leftWheelTickCount);

void setup() {
  Serial.begin(115200);
  setupWiFi( "FRITZ!Box 7530 XA", "87338834851078879323");
  setupPins(16, 17);
  nh.getHardware()->setConnection(server, serverPort);
  nh.advertise(rightPub);
  nh.advertise(leftPub); 
  
  pinMode(L_PWM, OUTPUT);
  pinMode(L_FORW, OUTPUT);
  pinMode(L_BACK, OUTPUT);
  pinMode(R_PWM, OUTPUT);
  pinMode(R_FORW, OUTPUT);
  pinMode(R_BACK, OUTPUT);
}

void loop(){
    nh.spinOnce();

    currentMillis = millis();
 
    if (currentMillis - previousMillis > interval) {
        
        previousMillis = currentMillis;
    
        // Publish tick counts to topics
        leftPub.publish(&leftWheelTickCount);
        rightPub.publish(&rightWheelTickCount);
  } 
}

void setupWiFi(const char*  ssid, const char*  password){
  
  
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) { delay(500);Serial.print("."); }
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());
  Serial.print("IP:   ");
  Serial.println(WiFi.localIP());
}

void countLeft(){
  leftWheelTickCount.data++;
}

void countRight(){
  rightWheelTickCount.data++;
}

void setupPins(const uint8_t leftPin, const uint8_t rightPin){
  pinMode(leftPin, INPUT_PULLUP);
  pinMode(rightPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(leftPin), countLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(rightPin), countRight, RISING);
}