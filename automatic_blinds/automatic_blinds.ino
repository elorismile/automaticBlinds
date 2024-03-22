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
#include <stdarg.h>

#define LIGHT_PIN A0
#define TEMPERATURE_PIN A1
#define BUTTON_PIN 2

#define LIGHT_THRESHOLD 400
#define TEMPERATURE_THRESHOLD 360

#define LIGHT_DEADZONE 100
#define TEMPERATURE_DEADZONE 20

volatile bool buttonToggle = false;

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

void button_ISR() {
  buttonToggle = !buttonToggle;
}

void setup() {
  pinMode(LIGHT_PIN, INPUT);
  pinMode(TEMPERATURE_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), button_ISR, RISING);
  
  myStepper.setSpeed(rolePerMinute);
  // initialize the serial port:
  Serial.begin(9600);
}

void loop() {  
  int light_value = analogRead(LIGHT_PIN);
  int temperature_value = analogRead(TEMPERATURE_PIN);

  static bool toggle = 0;
  if (digitalRead(BUTTON_PIN)) {
    toggle = !toggle;
  }

  Serial.println(light_value);
  Serial.println(temperature_value);
  Serial.println(buttonToggle);
  Serial.println("---");
  
  static bool temp_state = 0;
  if (temperature_value > TEMPERATURE_THRESHOLD + TEMPERATURE_DEADZONE) {
    temp_state = 1;
  }
  else if (temperature_value < TEMPERATURE_THRESHOLD - TEMPERATURE_DEADZONE) {
    temp_state = 0;
  }

  static bool light_state = 0;
  if (light_value > LIGHT_THRESHOLD + LIGHT_DEADZONE)  {
    light_state = 1;
  }
  else if (light_value < LIGHT_THRESHOLD - LIGHT_DEADZONE) {
    light_state = 0;
  }

  if (light_state && temp_state) {
    setPosition(50); //close blinds upwards to block light
  }
  else if (light_state && !temp_state) {
    setPosition(0); //open blinds to increase temperature
  }
  else if (!light_state && temp_state) {
    setPosition(-50); //close blinds downwards to let cold air in
  }
  else {
    setPosition(50); //close blinds upwards to block cold drafts
  }

  delay(1000);
}