/*
  ServoMotor.cpp - Library for Arduino Motor Shields
  Copyright (c) 2018-2019 Arduino AG.  All right reserved.
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "ServoMotor.h"
#include "Common.h"

static int next_instance = 0;

mc::ServoMotor::ServoMotor() {
  instance = next_instance;
  next_instance++;
};

void mc::ServoMotor::setAngle(int angle) {
  setData(SET_PWM_DUTY_CYCLE_SERVO, instance, map(angle,0,180,7,28));
}

void mc::ServoMotor::detach() {
  setData(SET_PWM_DUTY_CYCLE_SERVO, instance, -1);
}

void mc::ServoMotor::setFrequency(int frequency) {
  setData(SET_PWM_FREQUENCY_SERVO, instance, frequency);
}

void mc::ServoMotor::setPinMode(int status) {
  setData(SET_SERVO_PIN_MODE, instance, status);
}

void mc::ServoMotor::void writeOutput(int value) {
  setData(SET_PIN_VALUE, instance, value);
}

int mc::ServoMotor::readInput() {
  int ret;
  int stat = getData(GET_PIN_VALUE, instance,(uint8_t*)&ret);
  return ret;
}
