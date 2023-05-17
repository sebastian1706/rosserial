#include "sonar.h"

bool Sonar::init(int tPin, int ePin){
    triggerPin = tPin;
    echoPin = ePin;
    return true;
}

float Sonar::getRange(){
    
    // establish variables for duration of the ping,
  // and the distance result in inches and centimeters:
  long duration, cm;
  
  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(triggerPin, OUTPUT);
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
  
  // The same pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(echoPin, INPUT);
  duration = pulseIn(echoPin, HIGH);
  
  // convert the time into a distance
  return microsecondsToCentimeters(duration);
}

long Sonar::microsecondsToCentimeters(long microseconds){
// The speed of sound is 340 m/s or 29 microseconds per centimeter.
// The ping travels out and back, so to find the distance of the
// object we take half of the distance travelled.
  return microseconds / 29.1 / 2;
}
