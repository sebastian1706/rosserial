#include <Arduino.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16.h>
#include "WiFi.h"
#include <std_msgs/Float32MultiArray.h>
#include <ros.h>
#include <ros/time.h>
#include "sonar.h"

#define ROSSERIAL_ARDUINO_TCP

TaskHandle_t SonarTask;
TaskHandle_t EncoderTask;
static int SonarTaskCore = 0;
static int EncoderTaskCore = 1;

ros::NodeHandle nh;

std_msgs::Float32MultiArray range_msg;
ros::Publisher pub_range("/ultrasound", &range_msg);

std_msgs::Int16 leftWheelTickCount;
std_msgs::Int16 rightWheelTickCount;
ros::Publisher rightPub("right_ticks", &rightWheelTickCount);
ros::Publisher leftPub("left_ticks", &leftWheelTickCount);

//Sonar
long range_time;
float rangeVec[3];

//Encoder
const int interval = 30;
long previousMillis = 0;
long currentMillis = 0;

//WIFI
IPAddress server(192,168,178,36);
uint16_t serverPort = 11411;
const char*  ssid = "FRITZ!Box 7530 XA";
const char*  password = "87338834851078879323";

Sonar sonarRight;
Sonar sonarLeft;
Sonar sonarFront;

//declare Functions
void setupWiFi();
void Sonarloop(void * pvParameters);
void Encoderloop(void * pvParameters);
void countLeft();
void countRight();
void setupPins(const uint8_t leftPin, const uint8_t rightPin);

void setup() {
  xTaskCreatePinnedToCore(Sonarloop, 
                          "SonarTask", /* Name of the task */
                          10000,      /* Stack size in words */
                          NULL,       /* Task input parameter */
                          1,          /* Priority of the task */
                          NULL,       /* Task handle. */
                          SonarTaskCore); /* Core where the task should run */

  xTaskCreatePinnedToCore(Encoderloop, 
                          "EncoderTask", /* Name of the task */
                          10000,      /* Stack size in words */
                          NULL,       /* Task input parameter */
                          0,          /* Priority of the task */
                          NULL,       /* Task handle. */
                          EncoderTaskCore); /* Core where the task should run */

  Serial.begin(115200);
  setupWiFi();
  setupPins(16,17);
  nh.getHardware()->setConnection(server, serverPort);   
  nh.initNode();
  nh.advertise(pub_range);
  nh.advertise(rightPub);
  nh.advertise(leftPub);
  
  sonarRight.init(18, 19);
  sonarLeft.init(12,13);
  sonarFront.init(32, 33);
}

void loop() {
  
}

void Sonarloop(void * pvParameters){
  for(;;){
      if ( millis() >= range_time ){
  
        float rangeRight = sonarRight.getRange() / 100;
        float rangeLeft = sonarLeft.getRange() / 100;
        float rangeFront = sonarFront.getRange() / 100;

        rangeVec[0] = rangeRight;
        rangeVec[1] = rangeLeft;
        rangeVec[2] = rangeFront;
        range_msg.data = rangeVec;
        range_msg.data_length = 3;

        pub_range.publish(&range_msg);
        range_time =  millis() + 50;
      }    
      nh.spinOnce();
  }

}

void Encoderloop(void * pvParameters){
  for(;;){
    currentMillis = millis();
    if (currentMillis - previousMillis > interval){
        previousMillis = currentMillis;
        // Publish tick counts to topics
        leftPub.publish(&leftWheelTickCount);
        rightPub.publish(&rightWheelTickCount);
    } 
    nh.spinOnce();
  }
}

void setupWiFi()
{  
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