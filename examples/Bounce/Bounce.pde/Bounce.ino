// Bounce.pde
// -*- mode: C++ -*-
//
// Make a single stepper bounce from one limit to another
//
// Copyright (C) 2012 Mike McCauley
// $Id: Random.pde,v 1.1 2011/01/05 01:51:01 mikem Exp mikem $

#include <AccelStepper.h>
// Define a stepper and the pins it will use
#define NUM_STEPS_PER_REV (1600)
#define MOTOR_INTERFACE_TYPE (1)
#define ENABLE_PIN (5)//D5
#define STEP_PIN (6)//D6
#define DIR_PIN (7)//D7
#define ENABLE_VAL (LOW)

// Define a stepper and the pins it will use
AccelStepper stepper = AccelStepper(MOTOR_INTERFACE_TYPE, STEP_PIN, DIR_PIN); // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5

void setup()
{  
  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  digitalWrite(ENABLE_PIN, ENABLE_VAL);
  
  // Change these to suit your stepper if you want
  stepper.setMaxSpeed(2000);
  stepper.setAcceleration(300);
  stepper.moveTo(8000);
}

void loop()
{
    // If at the end of travel go to the other end
    if (stepper.distanceToGo() == 0)
      stepper.moveTo(-stepper.currentPosition());

    stepper.run();
}
