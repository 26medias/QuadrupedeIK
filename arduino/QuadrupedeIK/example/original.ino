#include <Wire.h>
#include <Servo.h>

// Constants for servos and I2C
// Leg #1
const int leg1_pin1 = 3;    // Hip
const int leg1_pin2 = 5;    // Elbow
// Leg #2
const int leg2_pin1 = 6;    // Hip
const int leg2_pin2 = 9;    // Elbow
const int i2cAddress = 0x08; // I2C address for this leg

// Servo objects
Servo leg1_servo1; // Leg #1
Servo leg1_servo2; // Leg #1
Servo leg2_servo1; // Leg #2
Servo leg2_servo2; // Leg #2


//----------------------------------
// PHYSICAL SETTINGS
//----------------------------------

// Physical Attributes
float sideA = 6.;     // Upper arm
float sideB = 6.7;    // Forearm

// Physical Limits
float minRadius = abs(sideA-sideB)*1.05;
float maxRadius = (sideA+sideB)*0.9999;

// What side the legs are located
bool isLeftSide = false;

//----------------------------------

// Gait variables
int gaitCycleIdx = -1;
int gaitCycleLength = 80;
float step_radius = 5.;
float step_height = 4.;
float ground_distance = 6;
int leg1_cycle_offset = 0; // 3 on the other side
int leg2_cycle_offset = 1; // 4 on the other side
int leg1_x_offset = -5;
int leg2_x_offset = 5;

//----------------------------------

// Angles & position memory
int angle_leg1_servo1, angle_leg1_servo2, angle_leg2_servo1, angle_leg2_servo2 = 0;
int leg1_x, leg1_y, leg2_x, leg2_y = 0;


bool debug = false;
bool isTest = false;

int itv = 1;

// Setup
void setup() {
    Serial.begin(9600);
    // Initialize servos
    leg1_servo1.attach(leg1_pin1);
    leg1_servo2.attach(leg1_pin2);
    leg2_servo1.attach(leg2_pin1);
    leg2_servo2.attach(leg2_pin2);

    // Initialize I2C communications as a slave
    Wire.begin(i2cAddress);
    Wire.onReceive(receiveEvent);
    Wire.onRequest(requestEvent);
    Serial.println("I2C Quadrupede started");
    delay(itv);
}


// Loop
void loop() {
    if (debug) {Serial.println("--------------");}

    if (isTest) {
        moveServos(90, 90, 0);
        moveServos(90, 90, 1);
        //testLinearX(-7, 7, -7, 0.2, 0);
        //elay(1000);
        //testLinearX(7, -7, -7, 0.2, 0);
        //moveToCoordinates(12, 0, 1);
    } else {
        stepGait(-1);
    }
    //
    if (debug) {
        Serial.print("Leg 1: ");
        Serial.print(leg1_x);
        Serial.print(", ");
        Serial.println(leg1_y);
        Serial.print("Leg 2: ");
        Serial.print(leg2_x);
        Serial.print(", ");
        Serial.println(leg2_y);
    }
    //moveToCoordinates(0, -12);
    //testLinearX(-7, 7, -7, 0.2, 0);
    //delay(1000);
    /*testLinearX(7, -7, -7, 0.2);
    delay(1000);
    testLinearY(-9, 9, -7, 0.2);
    delay(1000);
    testLinearY(9, -9, -7, 0.2);
    delay(1000);
    testCircle(-9, -3, 5, 360);
    delay(1000);
    testCircle(0, -6, 3, 360);*/
    delay(100);
}




//----------------------------------
// Angle upgrade
//----------------------------------

// Angle Normalization based on the robot's physical setup
int fix_shoulder(int x, int legNumber) {
    if (legNumber==0) {
        return normalize_angle_180(x);
    }
    return normalize_angle_180(180-x);
}
int fix_elbow(int x, int legNumber) {
    if (legNumber==0) {
        return normalize_angle_180(x);
    }
    return normalize_angle_180(180-x);
}




//----------------------------------
// Servo Control
//----------------------------------

// Universal angles -> Servo Angles
void moveServos(int a1, int a2, int legNumber) {
    float shoulder = fix_shoulder(a1, legNumber);
    float elbow = fix_elbow(a2, legNumber);
    moveServosRaw(shoulder, elbow, legNumber);
}

// Raw servo angles
void moveServosRaw(int a1, int a2, int legNumber) {
    if (legNumber==0) {
        leg1_servo1.write(a1);
        leg1_servo2.write(a2);
        angle_leg1_servo1 = a1;
        angle_leg1_servo2 = a2;
    } else {
        leg2_servo1.write(a1);
        leg2_servo2.write(a2);
        angle_leg2_servo1 = a1;
        angle_leg2_servo2 = a2;
    }
}

// Move the tip of a leg to coordinates (x, y)
void moveToCoordinates(float x, float y, int legNumber) {
    if (!isLeftSide) {
        legNumber = !legNumber;
    }
    if ((legNumber==1 && isLeftSide) || (legNumber==0 && !isLeftSide)) {
        x *= -1; // Reverse the X axis for uniform control
    }
    float pointA_x, pointA_y, pointB_x, pointB_y, pointC_x, pointC_y;
    IK(x, y, pointA_x, pointA_y, pointB_x, pointB_y, pointC_x, pointC_y);
    
    float ElbowAngle, angle_AB, angle_BC;
    trianglePointsToAngles(pointA_x, pointA_y, pointB_x, pointB_y, pointC_x, pointC_y, ElbowAngle, angle_AB, angle_BC);
    float shoulderAngle = abs(getShoulderAngle(pointB_x, pointB_y));

    moveServos(shoulderAngle, ElbowAngle, legNumber);
    if (legNumber==0) {
        leg1_x = x;
        leg1_y = y;
    } else {
        leg2_x = x;
        leg2_y = y;
    }
}




//----------------------------------
// Inverse Kinematic
//----------------------------------

// Normalize the angle between -180 and 180
float normalize_angle_180(float angle) {
        float normalized_angle = fmod(angle, 360);
        if (normalized_angle > 180) {
                normalized_angle -= 360;
        }
        return normalized_angle;
}

// Apply enveloppe restrictMovementsions on input coordinates
void restrictMovements(float &x, float &y) {
        float current_distance = sqrt(x*x + y*y);
        float scale_factor;
        if (current_distance < minRadius) {
                scale_factor = minRadius / current_distance;
        } else if (current_distance >= maxRadius) {
                scale_factor = maxRadius / current_distance;
        } else {
                return;
        }
        x *= scale_factor;
        y *= scale_factor;
}

// Calculate the distance from the origin to the point (x, y)
void getDistanceTarget(float x, float y, float limit, float &adjusted_length, float &_x, float &_y) {
        float length = sqrt(x*x + y*y);
        float scale_factor;
        if (length > limit) {
                scale_factor = limit / length;
                _x = x * scale_factor;
                _y = y * scale_factor;
                adjusted_length = limit;
        } else {
                _x = x;
                _y = y;
                adjusted_length = length;
        }
}

// Returns the missing point's coordinates (elbow) given the location of 2 points in the triangle & the length of 2 sides (upper arm & forearm)
void getPointB(float ax, float ay, float cx, float cy, float sideC, int side, float &bx, float &by) {
        float cos_angle_A = max(min((sideA*sideA + sideC*sideC - sideB*sideB) / (2 * sideA * sideC), 1.0f), -1.0f);
        float angle_A_rad = acos(cos_angle_A);

        float angle_AC_direction_rad = atan2(cy - ay, cx - ax);

        float angle_B_direction_rad;
        if (side == 1) {
                angle_B_direction_rad = angle_AC_direction_rad - angle_A_rad;
        } else {
                angle_B_direction_rad = angle_AC_direction_rad + angle_A_rad;
        }

        bx = ax + sideA * cos(angle_B_direction_rad);
        by = ay + sideA * sin(angle_B_direction_rad);
}


// Convert 2D coordinates to servo angles
void IK(float x, float y, float &pointA_x, float &pointA_y, float &pointB_x, float &pointB_y, float &pointC_x, float &pointC_y) {
        pointA_x = 0;
        pointA_y = 0;
        restrictMovements(x, y);

        float adjusted_length, _x, _y;
        getDistanceTarget(x, y, maxRadius, adjusted_length, _x, _y);

        pointC_x = _x;
        pointC_y = _y;
        getPointB(pointA_x, pointA_y, pointC_x, pointC_y, adjusted_length, 1, pointB_x, pointB_y);
}

float getShoulderAngle(float x, float y) {
        float shoulderAngle = atan2(x, y); // Note: Arduino's atan2 takes y first, then x
        return degrees(shoulderAngle); // Convert from radians to degrees
}


// Convert triangle points to angles
void trianglePointsToAngles(float pA_x, float pA_y, float pB_x, float pB_y, float pC_x, float pC_y, float &angle_AC, float &angle_AB, float &angle_BC) {
        auto distance = [](float x1, float y1, float x2, float y2) {
                return sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2));
        };
        
        float side_AB = distance(pA_x, pA_y, pB_x, pB_y);
        float side_BC = distance(pB_x, pB_y, pC_x, pC_y);
        float side_AC = distance(pA_x, pA_y, pC_x, pC_y);

        angle_AC = acos((side_AB*side_AB + side_BC*side_BC - side_AC*side_AC) / (2 * side_AB * side_BC)) * 180.0 / PI;
        angle_AB = acos((side_AC*side_AC + side_BC*side_BC - side_AB*side_AB) / (2 * side_AC * side_BC)) * 180.0 / PI;
        angle_BC = acos((side_AB*side_AB + side_AC*side_AC - side_BC*side_BC) / (2 * side_AB * side_AC)) * 180.0 / PI;
}






//----------------------------------
// Gait Calculation
//----------------------------------

// Function to calculate a single frame of the gait cycle
void buildGaitFrame(int frames, int frame_number, float &longitudinalMovement, float &verticalMovement) {
    // Ensure frame_number is within the valid range
    frame_number = frame_number % frames;
    float swingEnd = M_PI / 2;

    // Calculate the angular position for the given frame
    float angle = (2 * M_PI / frames) * frame_number;

    // Longitudinal Movement
    if (angle <= swingEnd) {
        // Swing phase
        longitudinalMovement = -cos(2 * angle);
    } else {
        // Stance phase
        longitudinalMovement = cos((2.0 / 3.0) * (angle - swingEnd));
    }

    // Vertical Movement
    if (angle <= swingEnd) {
        // Swing phase
        verticalMovement = sin(2 * angle);
    } else {
        // Stance phase
        verticalMovement = 0;
    }
}

int cyclicalIndex(int idx, int limit) {
        idx = idx % limit;
        if (idx < 0) {
                idx += limit;
        }
        return idx;
}


void stepGait(int step) {
    float leg1_x, leg1_y, leg2_x, leg2_y;
    gaitCycleIdx += step;
    gaitCycleIdx = cyclicalIndex(gaitCycleIdx, gaitCycleLength);

    int leg1_k_offset = (gaitCycleLength/4)*leg1_cycle_offset;
    int leg2_k_offset = (gaitCycleLength/4)*leg2_cycle_offset;

    int leg1_idx = cyclicalIndex(round((gaitCycleIdx+leg1_k_offset)), gaitCycleLength);
    int leg2_idx = cyclicalIndex(round((gaitCycleIdx+leg2_k_offset)), gaitCycleLength);
    if (debug) {
        Serial.print("[STEP] #");
        Serial.print(gaitCycleIdx);
        Serial.print(" / ");
        Serial.println(gaitCycleLength);

        Serial.print("[Offset] ");
        Serial.print(leg1_k_offset);
        Serial.print(", ");
        Serial.println(leg2_k_offset);

        Serial.print("[IDX] ");
        Serial.print(leg1_idx);
        Serial.print(", ");
        Serial.println(leg2_idx);

        Serial.print("[OUT #1]");
        Serial.print(leg1_x);
        Serial.print(", ");
        Serial.println(leg1_y);

        Serial.print("[OUT #2]");
        Serial.print(leg2_x);
        Serial.print(", ");
        Serial.println(leg2_y);
    }


    buildGaitFrame(gaitCycleLength, leg1_idx, leg1_x, leg1_y);
    buildGaitFrame(gaitCycleLength, leg2_idx, leg2_x, leg2_y);

    float leg1_final_x = leg1_x*step_radius + leg1_x_offset;
    float leg2_final_x = leg2_x*step_radius + leg2_x_offset;

    moveToCoordinates(leg1_final_x, (ground_distance*-1)+(leg1_y*step_height), 0);
    moveToCoordinates(leg2_final_x, (ground_distance*-1)+(leg2_y*step_height), 1);
}


//----------------------------------
// Robot Setting methods
//----------------------------------

void setBotSettings(float A, float B, int side) {
    sideA = A;     // Upper arm
    sideB = B;     // Forearm
    if (side) {
        isLeftSide = true;
    } else {
        isLeftSide = false;
    }
    // Physical Limits
    minRadius = abs(sideA-sideB)*1.05;
    maxRadius = (sideA+sideB)*0.9999;
}



//----------------------------------
// I2C Events
//----------------------------------

// Send
void requestEvent() {
    // When data is requested, send current position and servo angles
    Wire.write(0);
    Wire.write(angle_leg1_servo1);
    Wire.write(angle_leg1_servo2);
    Wire.write(leg1_x);
    Wire.write(leg1_y);
    Wire.write(1);
    Wire.write(angle_leg2_servo1);
    Wire.write(angle_leg2_servo2);
    Wire.write(leg2_x);
    Wire.write(leg2_y);
}

//Receive
/*
    Ops:
    0: moveToCoordinates 
    1: moveServos
    2: moveServosRaw
    3: setBotSettings
*/
void receiveEvent(int howMany) {
    // Read incoming data (3D coordinates)
    if (Wire.available() >= 4) {
        int op = Wire.read();
        if (op==0) {
            int n = Wire.read();
            int x = Wire.read();
            int y = Wire.read();
            moveToCoordinates(x, y, n);
        } else if (op==1) {
            int n = Wire.read();
            int a1 = Wire.read();
            int a2 = Wire.read();
            moveServos(a1, a2, n);
        } else if (op==2) {
            int n = Wire.read();
            int a1 = Wire.read();
            int a2 = Wire.read();
            moveServosRaw(a1, a2, n);
        } else if (op==3) {
            int side = Wire.read();
            int lengthA = Wire.read();
            int lengthB = Wire.read();
            setBotSettings(lengthA, lengthB, side);
        }
        
    }
}



//----------------------------------
// Test Methods
//----------------------------------

void getCirclePoints(float center_x, float center_y, float radius, float angle, float &x, float &y) {
        float angle_radians = radians(angle); // Convert angle from degrees to radians
        x = center_x + radius * cos(angle_radians);
        y = center_y + radius * sin(angle_radians);
}


void testCircle(float center_x, float center_y, float radius, float points, int legNumber) {
    float x, y;
    for (float i=0.;i<=360.;i+=360./points) {
        getCirclePoints(center_x, center_y, radius, i, x, y);
        moveToCoordinates(x, y, legNumber);
        delay(itv);
    }
}

void testLinearX(float from, float to, float y, float incr, int legNumber) {
    if (from < to) {
        for (float i=from;i<=to;i+=incr) {
            if (legNumber>=0) {
                moveToCoordinates(i, y, legNumber);
            } else {
                //moveToCoordinates(i, y, 0);
                //moveToCoordinates(i, y, 1);
            }
            delay(itv);
        }
    } else {
        for (float i=from;i>=to;i-=incr) {
            if (legNumber>=0) {
                moveToCoordinates(i, y, legNumber);
            } else {
                //moveToCoordinates(i, y, 0);
                //moveToCoordinates(i, y, 1);
            }
            delay(itv);
        }
    }
}
void testLinearY(float from, float to, float x, float incr, int legNumber) {
    if (from < to) {
        for (float i=from;i<=to;i+=incr) {
            if (legNumber>=0) {
                moveToCoordinates(x, i, legNumber);
            } else {
                moveToCoordinates(x, i, 0);
                moveToCoordinates(x, i, 1);
            }
            delay(itv);
        }
    } else {
        for (float i=from;i>=to;i-=incr) {
            if (legNumber>=0) {
                moveToCoordinates(x, i, legNumber);
            } else {
                moveToCoordinates(x, i, 0);
                moveToCoordinates(x, i, 1);
            }
            delay(itv);
        }
    }
}