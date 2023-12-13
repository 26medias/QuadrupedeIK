#include "QuadrupedeIK.h"
#include "LegIK.h"

// Constructor: empty as there's nothing to initialize in this simple example
QuadrupedeIK::QuadrupedeIK(int _pinShoulder1, int _pinElbow1, int _pinShoulder2, int _pinElbow2, int _pinShoulder3, int _pinElbow3, int _pinShoulder4, int _pinElbow4): leg1(_pinShoulder1, _pinElbow1), leg2(_pinShoulder2, _pinElbow2), leg3(_pinShoulder3, _pinElbow3), leg4(_pinShoulder4, _pinElbow4) {
    this->leg1 = LegIK(_pinShoulder1, _pinElbow1);
    this->leg2 = LegIK(_pinShoulder2, _pinElbow2);
    this->leg3 = LegIK(_pinShoulder3, _pinElbow3);
    this->leg4 = LegIK(_pinShoulder4, _pinElbow4);
    
    this->pinShoulder1 = _pinShoulder1;
    this->pinElbow1 = _pinElbow1;
    this->pinShoulder2 = _pinShoulder2;
    this->pinElbow2 = _pinElbow2;
    this->pinShoulder3 = _pinShoulder3;
    this->pinElbow3 = _pinElbow3;
    this->pinShoulder4 = _pinShoulder4;
    this->pinElbow4 = _pinElbow4;
}

void QuadrupedeIK::begin() {
    Serial.begin(9600);
    this->leg1.gaitLegCount = 4;
    this->leg2.gaitLegCount = 4;
    this->leg3.gaitLegCount = 4;
    this->leg4.gaitLegCount = 4;
    
    this->leg1.cycle_offset = 0;
    this->leg2.cycle_offset = 1;
    this->leg3.cycle_offset = 2;
    this->leg4.cycle_offset = 3;
    
    this->leg1.clockwise = false;
    this->leg2.clockwise = true;
    this->leg3.clockwise = false;
    this->leg4.clockwise = true;
    
    this->leg1.begin();
    this->leg2.begin();
    this->leg3.begin();
    this->leg4.begin();
}

void QuadrupedeIK::setGaitCycleLength(int gaitCycleLength) {
    this->gaitCycleLength = gaitCycleLength;
    this->leg1.gaitCycleLength = this->gaitCycleLength;
    this->leg2.gaitCycleLength = this->gaitCycleLength;
    this->leg3.gaitCycleLength = this->gaitCycleLength;
    this->leg4.gaitCycleLength = this->gaitCycleLength;
}
void QuadrupedeIK::setGaitStepRadius(int gaitStepRadius) {
    this->gaitStepRadius = gaitStepRadius;
    this->leg1.step_radius = this->gaitStepRadius;
    this->leg2.step_radius = this->gaitStepRadius;
    this->leg3.step_radius = this->gaitStepRadius;
    this->leg4.step_radius = this->gaitStepRadius;
}
void QuadrupedeIK::setGaitStepHeight(int gaitStepHeight) {
    this->gaitStepHeight = gaitStepHeight;
    this->leg1.step_height = this->gaitStepHeight;
    this->leg2.step_height = this->gaitStepHeight;
    this->leg3.step_height = this->gaitStepHeight;
    this->leg4.step_height = this->gaitStepHeight;
}
void QuadrupedeIK::setGroundDistance(int groundDistance) {
    this->groundDistance = groundDistance;
    this->leg1.ground_distance = this->groundDistance;
    this->leg2.ground_distance = this->groundDistance;
    this->leg3.ground_distance = this->groundDistance;
    this->leg4.ground_distance = this->groundDistance;
}

int QuadrupedeIK::step() {
    this->gaitStep += 1;
    this->leg1.cyclicalIndex(this->gaitStep, this->leg1.gaitCycleLength);
    this->leg2.cyclicalIndex(this->gaitStep, this->leg2.gaitCycleLength);
    this->leg3.cyclicalIndex(this->gaitStep, this->leg3.gaitCycleLength);
    this->leg4.cyclicalIndex(this->gaitStep, this->leg4.gaitCycleLength);
    return this->gaitStep;
}