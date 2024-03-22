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
#define BUTTON_PIN 2

#define LIGHT_THRESHOLD 400
#define TEMPERATURE_THRESHOLD 360

#define LIGHT_DEADZONE 100
#define TEMPERATURE_DEADZONE 20

//int stepper_position = 0; //store current stepper position

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
  static int stepper_position = 0;
  int steps = target_position - stepper_position;
  //Serial.println("Stepping %d steps", steps);
  myStepper.step(steps);
  stepper_position = target_position;
  disengageStepper();
}

void setup() {
  pinMode(LIGHT_PIN, INPUT);
  pinMode(TEMPERATURE_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT);
  
  myStepper.setSpeed(rolePerMinute);
  // initialize the serial port:
  Serial.begin(9600);
}

void loop() {  
  int light_value = analogRead(LIGHT_PIN);
  int temperature_value = analogRead(TEMPERATURE_PIN);

  static int state = 9;
  int setpoint = -3;

  Serial.println(light_value);
  Serial.println(temperature_value);
  if (light_value > LIGHT_THRESHOLD + LIGHT_DEADZONE) {
    if (temperature_value > TEMPERATURE_THRESHOLD + TEMPERATURE_DEADZONE) {
      setpoint = 500; //close up
      state = 0;
    }
    else if (temperature_value < TEMPERATURE_THRESHOLD - TEMPERATURE_DEADZONE) {
      setpoint = 0; //set to open
      state = 1;
    }
    
  }
  else if (light_value < LIGHT_THRESHOLD - LIGHT_DEADZONE) {
    if (temperature_value > TEMPERATURE_THRESHOLD + TEMPERATURE_DEADZONE) {
      setpoint = -500; //close down
      state = 2;
    }
    else if (temperature_value < TEMPERATURE_THRESHOLD - TEMPERATURE_DEADZONE) {
      setpoint = 500; //close up
      state = 3;
    }
  }


  setPosition(setpoint);
  Serial.println(state);

  delay(500);
}