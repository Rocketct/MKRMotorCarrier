#include "Arduino.h"

class ServoMotor {
  public:
    ServoMotor(int pinA) : pin(pinA) {
      pinMode(pinA, OUTPUT);
    };
    void setDuty(int duty);
    void setFrequency(int frequency);
    void setPinMode(int status);
    void writeOutput(int value);
    int  readInput();
  private:
    int pin;
    int duty = 0;
    int frequency = 50;
};
