
#include "WiFi.h"
#include "sonar.h"
#include <std_msgs/Float32MultiArray.h>
#include <ros.h>
#include <ros/time.h>


#define ROSSERIAL_ARDUINO_TCP

ros::NodeHandle  nh;

std_msgs::Float32MultiArray range_msg;
ros::Publisher pub_range( "/ultrasound", &range_msg);

long range_time;
float rangeVec[3];
//WIFI
IPAddress server(192,168,178,36);
uint16_t serverPort = 11411;
const char*  ssid = "FRITZ!Box 7530 XA";
const char*  password = "87338834851078879323";


void setupWiFi();
Sonar sonarRight;
Sonar sonarLeft;
Sonar sonarFront;

void setup() {
  Serial.begin(115200);
  setupWiFi();
  nh.getHardware()->setConnection(server, serverPort);   
  nh.initNode();
  nh.advertise(pub_range);
  
  /*range_msg.layout.dim[0].size = rangeVec.size();
  range_msg.layout.dim[0].stride = 1;
  range_msg.layout.dim[0].label = "x"; // or whatever name you typically use to index*/
  
  sonarRight.init(18, 19);
  sonarLeft.init(12,13);
  sonarFront.init(32, 33);
  
}

void loop()
{
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


void setupWiFi()
{  
   WiFi.begin(ssid, password);
   while (WiFi.status() != WL_CONNECTED) { delay(500);Serial.print("."); }
   Serial.print("SSID: ");
   Serial.println(WiFi.SSID());
   Serial.print("IP:   ");
   Serial.println(WiFi.localIP());
}
