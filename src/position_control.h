#pragma once
#include "hal.h"

// Maximum number of radians a motor is allowed to turn in one second
#define MAX_ANGULAR_SPEED (float)((1.0f / 180.0f) * PI)
#define STEPS_PER_SECOND 100
#define MAX_ANGULAR_STEP (float)(MAX_ANGULAR_SPEED / (float)(STEPS_PER_SECOND))

enum robot_type {
    // Robot Type V1 = Rotary base, vertical rotation, vertical rotation, vertical
    // rotation, no end effector
    ROBOT_TYPE_V1 = 0,
    // Robot Type V2 = Rotary base, vertical rotation, vertical rotation, vertical
    // rotation, wrist end effector?
    ROBOT_TYPE_V2 = 1,
};

struct Point {
    float x;
    float y;
    float z;
};

// Joint angles for the joints of the arm
struct JointAngles {
    float theta1; // Joint 1 (Rotary base) angle
    float theta2; // Joint 2 (Shoulder) angle
    float theta3; // Joint 3 (Elbow 1) angle
    float theta4; // Joint 4 (Elbow 2) angle
};

extern struct JointAngles current_angles;

extern enum robot_type robot_type;

int position_control_inverse_kinematics(struct Point p, struct JointAngles *angles);
float position_control_get_angle_from_duty_cycle(enum motor_num motor);

void position_control_forward_kinematics(struct JointAngles angles, struct Point *position);
int position_control_set_target_position(struct Point target);
int position_control_check_angles(void);
int position_control_step_towards_position(void);
void position_control_move_to_start(void);
