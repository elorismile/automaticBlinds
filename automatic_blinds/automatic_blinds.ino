//www.elegoo.com
//2018.10.25

/*
  Stepper Motor Control - one revolution

  This program drives a unipolar or bipolar stepper motor.
  The motor is attached to digital pins 8 - 11 of the Arduino.

  The motor should revolve one revolution in one direction, then
  one revolution in the other direction.

*/

#include <Stepper.h>

#define LIGHT_PIN A0
#define TEMPERATURE_PIN A1

#define LIGHT_THRESHOLD 300
#define TEMPERATURE_THRESHOLD 60

#define LIGHT_DEADZONE 50
#define TEMPERATURE_DEADZONE 2

const int stepsPerRevolution = 2048;  // change this to fit the number of steps per revolution
const int rolePerMinute = 12;         // Adjustable range of 28BYJ-48 stepper is 0~17 rpm

// initialize the stepper library on pins 8 through 11:
Stepper myStepper(stepsPerRevolution, 8, 10, 9, 11);

void disengageStepper() {
  digitalWrite(8, LOW);
  digitalWrite(10, LOW);
  digitalWrite(9, LOW);
  digitalWrite(11, LOW);
}

void setPosition(int target_position) {
  static int current_position = 0;
  int steps = target_position - current_position;
  //Serial.println("Stepping %d steps", steps);
  myStepper.step(steps);
  current_position = target_position;
  disengageStepper();
}

void setup() {
  myStepper.setSpeed(rolePerMinute);
  // initialize the serial port:
  Serial.begin(9600);
}

void loop() {  
  int light_value = analogRead(LIGHT_PIN);
  int temperature_value = analogRead(TEMPERATURE_PIN);
  Serial.println(temperature_value);
  if (light_value > LIGHT_THRESHOLD + LIGHT_DEADZONE) {
    if (temperature_value > TEMPERATURE_THRESHOLD + TEMPERATURE_DEADZONE) {
      setPosition(500); //close up
    }
    else if (temperature_value < TEMPERATURE_THRESHOLD - TEMPERATURE_DEADZONE) {
      setPosition(0); //set to open
    }
    
  }
  else if (light_value < LIGHT_THRESHOLD - LIGHT_DEADZONE) {
    if (temperature_value > TEMPERATURE_THRESHOLD + TEMPERATURE_DEADZONE) {
      setPosition(500); //close down
    }
    else if (temperature_value < TEMPERATURE_THRESHOLD - TEMPERATURE_DEADZONE) {
      setPosition(500); //close up
    }
  }
  delay(1000);
}