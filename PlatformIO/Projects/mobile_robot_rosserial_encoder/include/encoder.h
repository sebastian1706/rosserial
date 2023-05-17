#include <Arduino.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16.h>

#define ROSSERIAL_ARDUINO_TCP
#define MMperTICK  10.21
#define RADIUS 65

namespace encoder{
    ros::NodeHandle nh;
    
    std_msgs::Int16 leftWheelTickCount;
    std_msgs::Int16 rightWheelTickCount;

    ros::Publisher rightPub("right_ticks", &rightWheelTickCount);
    ros::Publisher leftPub("left_ticks", &leftWheelTickCount);

    void setupPins(const uint8_t leftPin, const uint8_t rightPin);
    void countRight();
    void countLeft();
};