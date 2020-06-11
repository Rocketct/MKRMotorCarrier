#include <Wire.h>
#include "EncoderWrapper.h"
#include "Battery.h"
#include "DCMotor.h"
#include "ServoMotor.h"
#include "Common.h"
#include "Events.h"
#include "FreeRAM.h"

#define Fix16 mn::MFixedPoint::FpF32<8>

// grab core from https://github.com/arduino/ArduinoCore-samd/tree/mkrmotorcarrier
// compile me with target arduino:samd:mkrmotorshield:bootloader=0kb,pinmap=complete,lto=disabled during development
// compile me with target arduino:samd:mkrmotorshield:bootloader=4kb,pinmap=complete,lto=enabled for release


const char* FW_VERSION = "0.30";


DCMotor* dcmotors[2];
ServoMotor* servos[4];
EncoderWrapper* encoders[2];

Battery* battery;

void led_on() {
  digitalWrite(LED_BUILTIN, HIGH);
}

typedef struct {
  Fix16 P;
  Fix16 I;
  Fix16 D;
} PIDGains;

union PIDData {
  Fix16 txFloat;
  uint8_t txArray[4];
} PIDGain;

void setup() {

  WDT->CTRL.reg &= ~WDT_CTRL_ENABLE;
  while (WDT->STATUS.reg & WDT_STATUS_SYNCBUSY);

  //temp_init();
  battery = new Battery(ADC_BATTERY);


  dcmotors[1] = new DCMotor(MOTOR_2_COUNTER, MOTOR_2_PIN_A, MOTOR_2_PIN_B),

  servos[0] = new ServoMotor(PWM_PIN_SERVO_1);
  servos[1] = new ServoMotor(PWM_PIN_SERVO_2);
  servos[2] = new ServoMotor(PWM_PIN_SERVO_3);
  servos[3] = new ServoMotor(PWM_PIN_SERVO_4);

  encoders[0] = new EncoderWrapper(ENCODER_2_PIN_A, ENCODER_2_PIN_B, 1);
  encoders[1] = new EncoderWrapper(ENCODER_1_PIN_A, ENCODER_1_PIN_B, 0);

  Wire.begin(I2C_ADDRESS);
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(IRQ_PIN, OUTPUT);
  analogWriteResolution(8);
}

volatile uint8_t command = 0;
volatile uint8_t target = 0;
volatile uint8_t irq_status = 0;
volatile unsigned long nextTimedEvent = 0;
volatile unsigned long lastMessageReceived = 0;

bool irq_enabled = true;

void loop() {
  if (command == RESET || ((lastMessageReceived != 0) && (millis() - lastMessageReceived > 10000))) {
    reboot();
  }
  if (command == PING) {
    lastMessageReceived = millis();
  }
  executeTimedEvents();
}

void receiveEvent(int howMany) {
  noInterrupts();
  command = Wire.read();

  if (command < GET_VERSION) {
    // empty buffer
    while (Wire.available()) {
      Wire.read();
    }
    interrupts();
    return;
  }

  if (Wire.available()) {
    target = Wire.read();
  } else {
    interrupts();
    return;
  }
  int value = 0;


  if (!Wire.available()) {
    interrupts();
    return;
  }

  uint8_t buf[12];
  int i = 0;
  while (Wire.available() && i < sizeof(buf)) {
    buf[i++] = (uint8_t)Wire.read();
  }

  // copies the bytes to int
  memcpy(&value, buf, sizeof(value));

  switch (command) {
    case SET_PWM_DUTY_CYCLE_SERVO:
      servos[target]->setDuty(value);
      break;
    case SET_PWM_FREQUENCY_SERVO:
      servos[target]->setFrequency(value);
      break;
    case SET_SERVO_PIN_MODE:
      servos[target]->setPinMode(value);
      break;
    case SET_PIN_VALUE:
      servos[target]->writeOutput(value);
      break;
    case SET_PWM_FREQUENCY_DC_MOTOR:
      dcmotors[target]->setFrequency(value);
      break;
    case RESET_COUNT_ENCODER:
      encoders[target]->resetCounter(value);
      break;
    case SET_INTERRUPT_ON_COUNT_ENCODER:
      encoders[target]->setIrqOnCount(value);
      break;
    case SET_INTERRUPT_ON_VELOCITY_ENCODER:
      encoders[target]->setIrqOnVelocity(value);
      break;
  }
  interrupts();
}

void requestEvent() {
  noInterrupts();
  //deassert IRQ
  if (irq_enabled) {
    digitalWrite(IRQ_PIN, HIGH);
  }

  // Always reply with irq status
  Wire.write(irq_status);

  switch (command) {
    case GET_VERSION:
      getFWVersion();
      break;
    case GET_RAW_COUNT_ENCODER:
      encoders[target]->getRawCount();
      break;
    case GET_OVERFLOW_UNDERFLOW_STATUS_ENCODER:
      encoders[target]->getOverflowUnderflow();
      break;
    case GET_COUNT_PER_SECOND_ENCODER:
      encoders[target]->getCountPerSecond();
      break;
    case GET_RAW_ADC_BATTERY:
      battery->getRaw();
      break;
    case GET_CONVERTED_ADC_BATTERY:
      battery->getConverted();
      break;
    case GET_FILTERED_ADC_BATTERY:
      battery->getFiltered();
      break;
    case GET_INTERNAL_TEMP:
      getInternalTemperature();
      break;
    case CLEAR_IRQ:
      Wire.write((int)irq_status);
      irq_status = 0;
      break;
    case GET_FREE_RAM:
      Wire.write((int)FreeRam());
      break;
    case GET_PIN_VALUE:
      Wire.write((servos[target]->readInput()));
      break;
  }
  interrupts();
}

void requestAttention(int cause) {
  irq_status |= (1 << cause);
  if (irq_enabled) {
    digitalWrite(IRQ_PIN, LOW);
  }
}

void getFWVersion() {
  Wire.write(FW_VERSION);
}

void getInternalTemperature() {
  //Wire.write(temp_raw_to_mdeg(temp_read_raw()));
}

void reboot() {
  NVIC_SystemReset();
}