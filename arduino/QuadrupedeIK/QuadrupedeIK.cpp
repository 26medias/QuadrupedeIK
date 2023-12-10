#include "QuadrupedeIK.h"

// Constructor: empty as there's nothing to initialize in this simple example
QuadrupedeIK::QuadrupedeIK(int pinShoulder, int pinElbow) {
    this->shoulderPin = pinShoulder;
    this->elbowPin = pinElbow;
}

void QuadrupedeIK::begin() {
    Serial.begin(9600);
    this->leg_servo1.attach(this->shoulderPin);
    this->leg_servo2.attach(this->elbowPin);
}

// Method implementation
void QuadrupedeIK::printQuadrupedeIK() {
    Serial.println("Hello, World!");
}




//----------------------------------
// Angle upgrade
//----------------------------------

// Angle Normalization based on the robot's physical setup
int QuadrupedeIK::fix_shoulder(int x) {
    if (!this->clockwise) {
        return normalize_angle_180(x);
    }
    return normalize_angle_180(180-x);
}
int QuadrupedeIK::fix_elbow(int x) {
    if (!this->clockwise) {
        return normalize_angle_180(x);
    }
    return normalize_angle_180(180-x);
}




//----------------------------------
// Servo Control
//----------------------------------

// Universal angles -> Servo Angles
void QuadrupedeIK::moveServos(int a1, int a2) {
    float shoulder = fix_shoulder(a1);
    float elbow = fix_elbow(a2);
    moveServosRaw(shoulder, elbow);
}

// Raw servo angles
void QuadrupedeIK::moveServosRaw(int a1, int a2) {
    this->leg_servo1.write(a1);
    this->leg_servo2.write(a2);
    angle_leg_servo1 = a1;
    angle_leg_servo2 = a2;
}

// Move the tip of a leg to coordinates (x, y)
void QuadrupedeIK::moveToCoordinates(float x, float y) {
    if (!this->isLeft) {
        x *= -1; // Reverse the X axis for uniform control
    }
    float pointA_x, pointA_y, pointB_x, pointB_y, pointC_x, pointC_y;
    IK(x, y, pointA_x, pointA_y, pointB_x, pointB_y, pointC_x, pointC_y);
    
    float ElbowAngle, angle_AB, angle_BC;
    trianglePointsToAngles(pointA_x, pointA_y, pointB_x, pointB_y, pointC_x, pointC_y, ElbowAngle, angle_AB, angle_BC);
    float shoulderAngle = abs(getShoulderAngle(pointB_x, pointB_y));
    
    if (this->debug) {

        Serial.print("[REQUEST] (");
        Serial.print(x);
        Serial.print(", ");
        Serial.print(y);
        Serial.println(")");

        Serial.print("[ElbowAngle] ");
        Serial.println(ElbowAngle);

        Serial.print("[shoulderAngle] ");
        Serial.println(shoulderAngle);

        Serial.print("[TIP] (");
        Serial.print(pointB_x);
        Serial.print(", ");
        Serial.print(pointB_y);
        Serial.println(")");
    }

    moveServos(shoulderAngle, ElbowAngle);
    leg_x = x;
    leg_y = y;
}


//----------------------------------
// Inverse Kinematic
//----------------------------------

// Normalize the angle between -180 and 180
float QuadrupedeIK::normalize_angle_180(float angle) {
        float normalized_angle = fmod(angle, 360);
        if (normalized_angle > 180) {
                normalized_angle -= 360;
        }
        return normalized_angle;
}

// Apply enveloppe restrictMovementsions on input coordinates
void QuadrupedeIK::restrictMovements(float &x, float &y) {
        float current_distance = sqrt(x*x + y*y);
        float scale_factor;
        if (current_distance < this->minRadius) {
                scale_factor = this->minRadius / current_distance;
        } else if (current_distance >= this->maxRadius) {
                scale_factor = this->maxRadius / current_distance;
        } else {
                return;
        }
        x *= scale_factor;
        y *= scale_factor;
}

// Calculate the distance from the origin to the point (x, y)
void QuadrupedeIK::getDistanceTarget(float x, float y, float limit, float &adjusted_length, float &_x, float &_y) {
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
void QuadrupedeIK::getPointB(float ax, float ay, float cx, float cy, float sideC, int side, float &bx, float &by) {
        float cos_angle_A = max(min((this->sideA*this->sideA + sideC*sideC - this->sideB*this->sideB) / (2 * this->sideA * sideC), 1.0f), -1.0f);
        float angle_A_rad = acos(cos_angle_A);

        float angle_AC_direction_rad = atan2(cy - ay, cx - ax);

        float angle_B_direction_rad;
        if (side == 1) {
                angle_B_direction_rad = angle_AC_direction_rad - angle_A_rad;
        } else {
                angle_B_direction_rad = angle_AC_direction_rad + angle_A_rad;
        }

        bx = ax + this->sideA * cos(angle_B_direction_rad);
        by = ay + this->sideA * sin(angle_B_direction_rad);
}


// Convert 2D coordinates to servo angles
void QuadrupedeIK::IK(float x, float y, float &pointA_x, float &pointA_y, float &pointB_x, float &pointB_y, float &pointC_x, float &pointC_y) {
        pointA_x = 0;
        pointA_y = 0;
        restrictMovements(x, y);

        float adjusted_length, _x, _y;
        getDistanceTarget(x, y, this->maxRadius, adjusted_length, _x, _y);

        pointC_x = _x;
        pointC_y = _y;
        getPointB(pointA_x, pointA_y, pointC_x, pointC_y, adjusted_length, 1, pointB_x, pointB_y);
}

float QuadrupedeIK::getShoulderAngle(float x, float y) {
        float shoulderAngle = atan2(x, y); // Note: Arduino's atan2 takes y first, then x
        return degrees(shoulderAngle); // Convert from radians to degrees
}


// Convert triangle points to angles
void QuadrupedeIK::trianglePointsToAngles(float pA_x, float pA_y, float pB_x, float pB_y, float pC_x, float pC_y, float &angle_AC, float &angle_AB, float &angle_BC) {
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
void QuadrupedeIK::buildGaitFrame(int frames, int frame_number, float &longitudinalMovement, float &verticalMovement) {
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

int QuadrupedeIK::cyclicalIndex(int idx, int limit) {
        idx = idx % limit;
        if (idx < 0) {
                idx += limit;
        }
        return idx;
}


void QuadrupedeIK::runGait(int step) {
    this->gaitCycleIdx = step;
    float leg_x, leg_y;
    step = cyclicalIndex(step, this->gaitCycleLength);

    int leg_k_offset = (this->gaitCycleLength/4)*this->cycle_offset;

    int leg_idx = cyclicalIndex(round((this->gaitCycleIdx+leg_k_offset)), this->gaitCycleLength);
    if (this->debug) {
        Serial.print("[STEP] #");
        Serial.print(this->gaitCycleIdx);
        Serial.print(" / ");
        Serial.println(this->gaitCycleLength);

        Serial.print("[Offset] ");
        Serial.println(leg_k_offset);

        Serial.print("[IDX] ");
        Serial.println(leg_idx);

        Serial.print("[OUT]");
        Serial.print(leg_x);
        Serial.print(", ");
        Serial.println(leg_y);
    }


    buildGaitFrame(this->gaitCycleLength, leg_idx, leg_x, leg_y);

    float leg_final_x = leg_x*this->step_radius + this->x_offset;

    moveToCoordinates(leg_final_x, (this->ground_distance*-1)+(leg_y*this->step_height));
}