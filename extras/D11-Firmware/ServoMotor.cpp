#include "ServoMotor.h"

void ServoMotor::setDuty(int duty) {
  this->duty = duty;
  if (duty < 0) {
    pinMode(pin, INPUT);
  } else {
    analogWrite(pin, duty);
  }
}

void ServoMotor::setFrequency(int frequency) {
  // NB: not implemented at the moment!
  //this->frequency = frequency;
}

void ServoMotor::setPinMode(int status) {
  pinMode(pin, status);
}

int ServoMotor::readInput() {
  return digitalRead(pin);
}

void ServoMotor::writeOutput(int value) {
  digitalWrite(pin, value);
}