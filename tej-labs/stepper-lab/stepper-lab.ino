#include <Stepper.h>

const int stepsPerRevolution = 200;

Stepper step(stepsPerRevolution, 8, 9, 10, 11);

void setup() {
}

void loop() {
  step.setSpeed(20);
  step.step(stepsPerRevolution);
  delay(1000);

  step.setSpeed(20);
  step.step(-stepsPerRevolution);
  delay(1000);
}
