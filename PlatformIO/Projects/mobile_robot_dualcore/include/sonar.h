#include "Arduino.h"

class Sonar{
    public:
    bool init(int tPin, int ePin);
    float getRange();

    private:
        int triggerPin;
        int echoPin;
        long microsecondsToCentimeters(long microseconds);
};
