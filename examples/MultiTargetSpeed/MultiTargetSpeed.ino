#include <AccelStepper.h>
// Define a stepper and the pins it will use
#define NUM_STEPS_PER_REV (1600)
#define MOTOR_INTERFACE_TYPE (1)
#define ENABLE_PIN (5)
#define STEP_PIN (6)
#define DIR_PIN (7)
#define ENABLE_VAL (LOW)

int state = 0;
uint32_t startT = 0;
AccelStepper stepper = AccelStepper(MOTOR_INTERFACE_TYPE, STEP_PIN, DIR_PIN); // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5

void setup() {
  // put your setup code here, to run once:
  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  digitalWrite(ENABLE_PIN, ENABLE_VAL);
  //accelerate to 4000 steps/sec with 300 acceleration
  stepper.setMaxSpeed(8000);
  stepper.setAcceleration(300);
  stepper.setTargetSpeed(4000);

  startT = millis();
}

void loop() {
  // put your main code here, to run repeatedly:
  stepper.run();
  uint32_t deltaTime = millis() - startT;
  if(deltaTime > 10000 && state == 0) {//deccelerate to zero with acceleration 500 from 10-th sec
    stepper.setAcceleration(500);
    stepper.setTargetSpeed(0);
    state++;
  }
  else if(deltaTime > 12000 && state == 1) {//accelerate to 2500 with acceleration 500 at 12-th sec
    stepper.setTargetSpeed(2500);
    state++;
  }
  else if(deltaTime > 20000 && state == 2) {//accelerate to 5000 with acceleration 800 at 20-th sec
    stepper.setAcceleration(800);
    stepper.setTargetSpeed(5000);
    state++;
  }
}
