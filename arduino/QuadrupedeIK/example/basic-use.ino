#include <QuadrupedeIK.h>

// Create an instance of the library
QuadrupedeIK leg1(3, 5);
QuadrupedeIK leg2(6, 9);

int index = 0;

void setup() {
  // Start the serial communication
  Serial.begin(9600);

  leg1.debug = true;
  leg2.debug = true;

  // Setup
  leg1.clockwise = false;
  leg1.gaitCycleLength = 80;
  leg1.step_radius = 5.;
  leg1.step_height = 4.;
  leg1.ground_distance = 6;
  leg1.cycle_offset = 0;
  leg1.x_offset = -5;
  leg1.isLeft = true;

  leg2.clockwise = true;
  leg2.gaitCycleLength = 80;
  leg2.step_radius = 5.;
  leg2.step_height = 4.;
  leg2.ground_distance = 6;
  leg2.cycle_offset = 0;
  leg2.x_offset = 0;
  leg2.isLeft = true;

  leg1.begin();
  leg2.begin();
}

void loop() {
  /*index = index + 1;
  index = leg1.cyclicalIndex(index, leg1.gaitCycleLength);
  leg1.runGait(index);*/
  leg1.moveToCoordinates(-8, -2);
  leg2.moveToCoordinates(0, -10);
  
  // Wait for a second²
  delay(1000000);
}
