/*
  This example shows how to change the configuration mode 
  of the servo pin on MKR Motor Carrier
  to use it:
  - short the servo1 and servo2 data pins
  - power the motorcarrier trhough the gnd and V connector on the shield
  - open the setial montior and see the satus change each 500 ms
 */


#include <Arduino_PMIC.h>
#include <MKRMotorCarrier.h>
//#include <MKRMotorCarrier_REV2.h>
#define INTERRUPT_PIN 6


void setup()
{
  //Serial port initialization
  Serial.begin(115200);
  while (!Serial);

  //Establishing the communication with the motor shield
  if (controller.begin())
  {
    Serial.print("MKR Motor Shield connected, firmware version ");
    Serial.println(controller.getFWVersion());
  }
  else
  {
    Serial.println("Couldn't connect! Is the red led blinking? You may need to update the firmware with FWUpdater sketch");
    while (1);
  }

  // Reboot the motor controller; brings every value back to default
  Serial.println("reboot");
  controller.reboot();
  delay(500);

  PMIC.begin();
  if (!PMIC.enableBoostMode()) {
    Serial.println("Error enabling Boost Mode");
  }

     // to use as input change OUTPUT with INPUT
      servo1.setPinMode(INPUT);
    servo3.setPinMode(OUTPUT);
}

int status = HIGH;
void loop() {

    //Servo sweep from 180 position to 0
  

    //Choose what of all the servo connectors do you want to use: servo1(default), servo2, servo3 or servo4
    servo3.writeOutput(HIGH);

    Serial.println(servo1.readInput());
    delay(1000);
    servo3.writeOutput(LOW);

    Serial.println(servo1.readInput());


  controller.ping();
  //wait
  delay(1000);
}