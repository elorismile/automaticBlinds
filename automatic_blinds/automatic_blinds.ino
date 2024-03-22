#include <Stepper.h>
#include <stdarg.h>

#define LIGHT_PIN A0
#define TEMPERATURE_PIN A1
#define BUTTON_PIN 2

#define LIGHT_THRESHOLD 400
#define TEMPERATURE_THRESHOLD 335

#define LIGHT_DEADZONE 100
#define TEMPERATURE_DEADZONE 10

#define MOTION_RANGE 250  //defines how many steps the motor should move in each direction

volatile bool buttonToggle = false;

const int stepsPerRevolution = 2048;  // change this to fit the number of steps per revolution
const int rolePerMinute = 12;         // Adjustable range of 28BYJ-48 stepper is 0~17 rpm

// initialize the stepper library on pins 8 through 11:
Stepper myStepper(stepsPerRevolution, 8, 10, 9, 11);

void disengageStepper() {  //disengages the stepper when not in use to save power, since position isn't critical
  digitalWrite(8, LOW);
  digitalWrite(10, LOW);
  digitalWrite(9, LOW);
  digitalWrite(11, LOW);
}

void setPosition(int target_position) {  //go to a desired position, then disengage
  static int stepper_position = 0;
  int steps = target_position - stepper_position;
  //Serial.println("Stepping %d steps", steps);
  myStepper.step(steps);
  stepper_position = target_position;
  disengageStepper();
}

void button_ISR() {  //handles the interrupt for the toggle button
  buttonToggle = !buttonToggle;
}

void setup() {
  pinMode(LIGHT_PIN, INPUT);
  pinMode(TEMPERATURE_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), button_ISR, RISING); //attatch button interrupt
  
  myStepper.setSpeed(rolePerMinute);  //set servo speed, slower provides more torque
  // initialize the serial port:
  Serial.begin(9600);
}

void loop() {  
  int light_value = analogRead(LIGHT_PIN); //read light sensor
  int temperature_value = analogRead(TEMPERATURE_PIN); //read temp sensor

  Serial.println(light_value);
  Serial.println(temperature_value);
  Serial.println(buttonToggle);
  Serial.println("---");
  
  static bool temp_state = 0;
  static bool prev_temp_state = 0;
  if (temperature_value > TEMPERATURE_THRESHOLD + TEMPERATURE_DEADZONE) {  //check if temp has changed outside deadzone
    temp_state = 1;
  }
  else if (temperature_value < TEMPERATURE_THRESHOLD - TEMPERATURE_DEADZONE) {
    temp_state = 0;
  }

  static bool light_state = 0;
  static bool prev_light_state = 0;
  if (light_value > LIGHT_THRESHOLD + LIGHT_DEADZONE)  {  //check if light level has changed outside deadzone
    light_state = 1;
  }
  else if (light_value < LIGHT_THRESHOLD - LIGHT_DEADZONE) {
    light_state = 0;
  }

  if (temp_state!=prev_temp_state || light_state!=prev_light_state || !buttonToggle) {
    if (light_state && temp_state) {
      setPosition(MOTION_RANGE); //close blinds upwards to block light
    }
    else if (light_state && !temp_state) {
      setPosition(0); //open blinds to increase temperature
    }
    else if (!light_state && temp_state) {
      setPosition(-MOTION_RANGE); //close blinds downwards to let cold air in
    }
    else {
      setPosition(MOTION_RANGE); //close blinds upwards to block cold drafts
    }
    buttonToggle = false;
  }


  if (buttonToggle) {
    if (light_state && !temp_state) {
      setPosition(-MOTION_RANGE); //toggle blinds closed downwards
    }
    else {
      setPosition(0); //toggle blinds open
    }
  }

  prev_temp_state = temp_state;
  prev_light_state = light_state;

  delay(1000);
}