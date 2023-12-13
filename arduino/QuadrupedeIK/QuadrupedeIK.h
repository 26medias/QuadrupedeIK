// Ensure this library description is only included once
#ifndef QuadrupedeIK_h
#define QuadrupedeIK_h

// Include types & constants of Wiring core API
#include <Arduino.h>
#include <Servo.h>
#include "LegIK.h"

// Library interface description
class QuadrupedeIK {
  public:
    // Constructor
    QuadrupedeIK(int _pinShoulder1, int _pinElbow1, int _pinShoulder2, int _pinElbow2, int _pinShoulder3, int _pinElbow3, int _pinShoulder4, int _pinElbow4);

    int pinShoulder1;
    int pinElbow1;
    int pinShoulder2;
    int pinElbow2;
    int pinShoulder3;
    int pinElbow3;
    int pinShoulder4;
    int pinElbow4;

    LegIK leg1;
    LegIK leg2;
    LegIK leg3;
    LegIK leg4;

    int gaitStep = 0;
    int gaitCycleLength = 60;
    int gaitStepRadius = 5.;
    int gaitStepHeight = 5.;
    int groundDistance = 5.;
  
    // Methods
    void begin();
    void setGaitCycleLength(int gaitCycleLength);
    void setGaitStepRadius(int gaitStepRadius);
    void setGaitStepHeight(int gaitStepHeight);
    void setGroundDistance(int groundDistance);
    int step();

};

#endif
