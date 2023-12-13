// Ensure this library description is only included once
#ifndef LegIK_h
#define LegIK_h

// Include types & constants of Wiring core API
#include <Arduino.h>
#include <Servo.h>

// Library interface description
class LegIK {
  public:
    // Constructor
    LegIK(int pinShoulder, int pinElbow);

    bool debug = false;

    int shoulderPin = 3;
    int elbowPin = 5;
    
    // gait property
    int gaitLegCount = 4; // How many legs to spread the gait
    int gaitCycleIdx = -1; // Current cycle step (all legs must be in sync)
    int gaitCycleLength = 80; // How many frames in the cycle (all legs must be in sync)
    int cycle_offset = 0; // Assign number 0 to `gaitLegCount` to each leg

    // Leg Properties
    float clockwise = false; // Servo direction
    float reverseX = false; // Servo direction, to rename
    float reverseY = false; // Servo direction, to rename
    bool foldOut = true; // Which direction to fold the arm
    float sideA = 6.;   // Upper arm
    float sideB = 6.7;  // Forearm
    float minRadius = abs(sideA-sideB)*1.05;
    float maxRadius = (sideA+sideB)*0.9999;
    float step_radius = 5.;
    float step_height = 4.;
    float ground_distance = 6;
    int x_offset = -5;

    // Speed
    int itv = 1;
    
    // Angles & position memory
    int angle_leg_servo1, angle_leg_servo2 = 0;
    int leg_x, leg_y = 0;

    // Methods
    void begin();
    void printQuadrupedeIK();
    int fix_shoulder(int x);
    int fix_elbow(int x);
    void moveServos(int a1, int a2);
    void moveServosRaw(int a1, int a2);
    void moveToCoordinates(float x, float y);
    float normalize_angle_180(float angle);
    void restrictMovements(float &x, float &y);
    void getDistanceTarget(float x, float y, float limit, float &adjusted_length, float &_x, float &_y);
    void getPointB(float ax, float ay, float cx, float cy, float sideC, int side, float &bx, float &by);
    void IK(float x, float y, float &pointA_x, float &pointA_y, float &pointB_x, float &pointB_y, float &pointC_x, float &pointC_y);
    float getShoulderAngle(float x, float y);
    void trianglePointsToAngles(float pA_x, float pA_y, float pB_x, float pB_y, float pC_x, float pC_y, float &angle_AC, float &angle_AB, float &angle_BC);
    void buildGaitFrame(int frames, int frame_number, float &longitudinalMovement, float &verticalMovement);
    int cyclicalIndex(int idx, int limit);
    void runGait(int step);
  
  private:
    Servo leg_servo1;
    Servo leg_servo2;
};

#endif
