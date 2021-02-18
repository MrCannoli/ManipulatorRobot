#pragma once
#include "hal.h"

/// \def Maximum number of radians a motor is allowed to turn in one second
#define MAX_ANGULAR_SPEED (float)((5.0f / 180.0f) * PI)

/// \def Number of stepper movements made in a second
#define STEPS_PER_SECOND 100

/// \def Max angle difference between two motor steps
#define MAX_ANGULAR_STEP (float)(MAX_ANGULAR_SPEED / (float)(STEPS_PER_SECOND))

/// \def Number of motorized joints
#define NUM_JOINTS 4 // Number of motorized joints

/// \def Total number of joints including the end effector
#define NUM_JOINTS_AND_END (NUM_JOINTS + 1)

/// \def Maximum length of the arm
#define MAX_ARM_LENGTH (float)(d12 + d23 + d34 + d45)

/// \enum Values denoting the types of robots supported
enum robot_type {
    // Robot Type V1 = Rotary base, vertical rotation, vertical rotation, vertical
    // rotation, no end effector
    ROBOT_TYPE_V1 = 0,
    // Robot Type V2 = Rotary base, vertical rotation, vertical rotation, vertical
    // rotation, wrist end effector?
    ROBOT_TYPE_V2 = 1,
};

/// \struct Cartesian point in space
struct Point {
    float x;
    float y;
    float z;
};

/// \enum Enumerated joint numbers for indexing arrays
enum joint_indexes {
    J1 = 0,
    J2 = 1,
    J3 = 2,
    J4 = 3,
    JEND = 4,
};

/// \var Version of the current robot
extern enum robot_type robot_type;

// Inverse Kinematics
int position_control_fabrik(struct Point target, float angles_out[]);

// Forward Kinematics
void position_control_forward_kinematics(float angles[], struct Point *position);
void position_control_joint_forward_kinematics(float angles[], struct Point fw_joint_positions[]);

// Joint angle set and check
float position_control_get_angle_from_duty_cycle(enum motor_num motor);
int position_control_check_angles(void);

// Movement control
int position_control_set_target_position(struct Point target);
int position_control_step_towards_position(void);
void position_control_move_to_start(void);
