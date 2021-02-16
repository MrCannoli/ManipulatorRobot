#pragma once
#include "hal.h"

// Maximum number of radians a motor is allowed to turn in one second
#define MAX_ANGULAR_SPEED (float)((5.0f / 180.0f) * PI)
#define STEPS_PER_SECOND 100
#define MAX_ANGULAR_STEP (float)(MAX_ANGULAR_SPEED / (float)(STEPS_PER_SECOND))
#define NUM_JOINTS 4 // Number of motorized joints
#define NUM_JOINTS_AND_END (NUM_JOINTS+1) // Num joints + the end effector

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
extern float joint_angles[NUM_JOINTS];

extern enum robot_type robot_type;

extern struct Point joint_positions[NUM_JOINTS_AND_END];

extern float current_angles[NUM_JOINTS];

int position_control_fabrik(struct Point target, float angles_out[]) ;
int position_control_inverse_kinematics(struct Point p, float angles[]);
float position_control_get_angle_from_duty_cycle(enum motor_num motor);

void position_control_forward_kinematics(float angles[], struct Point *position);
void position_control_joint_forward_kinematics(float angles[], struct Point fw_joint_positions[]);
int position_control_set_target_position(struct Point target);
int position_control_check_angles(void);
int position_control_step_towards_position(void);
void position_control_move_to_start(void);
