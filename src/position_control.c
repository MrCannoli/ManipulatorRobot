#include <math.h>
#include <stdlib.h>
#include <string.h> // For memcmp()

#include "hal.h"
#include "position_control.h"

// For now this is defined in the C code, but eventually should be moved to a
// "settings" section of ROM
enum robot_type robot_type = ROBOT_TYPE_V1;

/// \def Distance constants for the different joints. All values are in
/// centimeters
#define d1 30.0f // Distance from Joint 2 to Joint 3
#define d2 30.0f // Distance from Joint 3 to Joint 4
#define d3 30.0f // Distance from Joint 4 to Joint 5 / initial endpoint

#define PI 3.14159265f

// Maximum length of the arm
const float MAX_ARM_LENGTH = sqrtf(powf(d1, 2) + powf(d2, 2) + powf(d3, 2));

// Struct containing the current joint angles
struct JointAngles current_angles;
// Struct containing the target joint angles
struct JointAngles target_angles;

// Struct containing the current position
struct Point current_position;
// Struct containing the target position
struct Point target_position;

// Frame of reference: {0,0,0} is the center point of Joint 2, directly above
// the rotational Joint 1. All angles being 0 will result in the arm IK being
// completely horizontal

/*!
 * \brief Generates a set of joint angles for the robot arm to reach the
 * designated point. Assumes "elbow up" wherever possible.
 *
 * \param p The {x,y,z} point we want IK for
 * \param angles Pointer to the struct we want to store the angles in. This will
 * not be modified if the function fails.
 *
 * \returns 1 if succeeded, 0 if the point is not possible to access
 */
int position_control_inverse_kinematics(struct Point p, struct JointAngles *angles) {
    // Check if the point is within our workspace
    if (sqrt(powf(p.x, 2) + powf(p.y, 2) + powf(p.z, 2)) > MAX_ARM_LENGTH) {
        // Point is outside of our workspace
        return 0;
    } else if (p.z < 0) {
        // Limit the arm from going below the horizontal plane
        return 0;
    }

    THIS FAILS. Need to rewrite the IK so that it can reach arbitrary points, not just those that meet the trapezoidal requirements
    Likely will need a numerical solution, not an analytical one?;

    struct JointAngles ik_angles;
    ik_angles.theta1 = atan2f(p.y, p.x); // This will be the same no matter what

    // Calculate intermediate distances & angles so we can generate the joint
    // angles. These calculations are based on the assumption our joints form an
    // imaginary trapezoid
    float r1 = sqrtf(powf(p.x, 2) + powf(p.y, 2)); // XY plane distance from origin
    float r2 = p.z;                                // Z axis increase from the joint angles
    float r3 = sqrtf(powf(r1, 2) + powf(r2, 2));   // Distance from Joint 2 to Joint 5 / target
    float phi1 = atan2f(r2, r1);                   // Angle of the triangle formed by Joint 2, Joint
                                                   // 5, and the point below Joint 5
    float phi2 = acosf((powf(d1, 2) + powf((r3 - d2), 2) - powf(d3, 2)) /
                       (2 * d1 * (r3 - d2)));                        // Angle between Joint 5, Joint 2, and Joint 3
    float phi3 = (PI / 2.0f) - phi2;                                 // Angle between Joint 2, Joint 3, and Joint 4
    float r4 = powf(d1, 2) + powf(d2, 2) - 2 * d1 * d2 * cosf(phi3); // Distance between Joint 2 and Joint 4
    float phi4 =
        acosf((powf(r3, 2) + powf(d3, 2) - powf(r4, 2)) / (2 * r3 * d3)); // Angle between Joint 2, Joint 5, and Joint 4

    // Generate the Joint angles from our calculated values
    ik_angles.theta1 = atan2f(p.y, p.x);
    ik_angles.theta2 = phi1 - phi2;
    ik_angles.theta3 = phi2;
    ik_angles.theta4 = phi4;

    if (ik_angles.theta1 > PI / 3.0f || ik_angles.theta2 > PI / 3.0f || ik_angles.theta3 > PI / 3.0f ||
        ik_angles.theta4 > PI / 3.0f) {
        // The motors are limited to 120deg of motion. Going beyond that means that
        // we cannot reach the point.
        return 0;
    } else if (sinf(ik_angles.theta2) < 0) {
        // Joint 3 will be shifted below the z plane
        return 0;
    } else if ((d1 * sinf(ik_angles.theta2) + d2 * sinf(ik_angles.theta2 + ik_angles.theta3)) < 0) {
        // Joint 4 will be below the z plane
        return 0;
    } else if ((d1 * sinf(ik_angles.theta2) + d2 * sinf(ik_angles.theta2 + ik_angles.theta3) +
                d3 * sinf(ik_angles.theta2 + ik_angles.theta3 + ik_angles.theta4)) < 0) {
        // Joint 5 will be below the z plane
        // Not a necessary step when Joint 5 is the endpoint, but this will be
        // necessary for the full wrist
        return 0;
    }

    *angles = ik_angles;
    return 1;
}

float position_control_get_angle_from_duty_cycle(enum motor_num motor) {
    // Need to confirm accuracy of this!!
    return ((PI / 3.0f) * motor_duty_cycle[motor - 1] / 100.0f);
}

void position_control_set_motor_angle(enum motor_num motor, float angle) {
    if (angle > PI / 3.0f) {
        // Cannot set it above 120 degrees
        logic_fault();
    }

    switch(motor){
        case MOTOR1:
            current_angles.theta1 = angle;
            break;
        case MOTOR2:
            current_angles.theta2 = angle;
            break;
        case MOTOR3:
            current_angles.theta3 = angle;
            break;
        case MOTOR4:
            current_angles.theta4 = angle;
            break;
        case MOTOR5:
        case MOTOR6:
        case MOTOR7:
        case MOTOR8:
        default:
            // No other motors are currently supported
            logic_fault();
            break;
    }

    float duty_cycle = (angle / (PI / 3.0f)) * 100;
    hal_set_motor_duty_cycle(motor, duty_cycle);
}

/*!
 * \brief Calculate the current end effector position via forward kinematics
 *
 * \param angles The joint configuration angles
 * \param position Point of the end effector location
 */
void position_control_forward_kinematics(struct JointAngles angles, struct Point *position) {
    switch (robot_type) {
        case ROBOT_TYPE_V1: {
            // Shorthand angle sums
            float angles23 = angles.theta2 + angles.theta3;
            float angles234 = angles.theta2 + angles.theta3 + angles.theta4;

            // Magnitude of distance in the xy plane
            float xy_plane_distance = d1 * cosf(angles.theta2) + d2 * cosf(angles23) + d3 * cosf(angles234);

            // Calculate the position
            position->x = cosf(angles.theta1) * xy_plane_distance;
            position->y = sinf(angles.theta1) * xy_plane_distance;
            position->z = d1 * sinf(angles.theta2) + d2 * sinf(angles23) + d3 * sinf(angles234);
            break;
        }
        case ROBOT_TYPE_V2:
            // This robot type includes the wrist! Update the forward kinematics when we
            // get here.

            // First rotation is the base (motor 1)
            // Second rotation is motor 2 at offset ROBOT_ARM_A_OFFSET
            // Third rotation is motor 3 at offset ROBOT_ARM_A_LEN -
            // ROBOT_ARM_A_B_TRANSITION_OFFSET from the second rotation Fourth rotation
            // is motor 4 at offset ROBOT_ARM_B_LEN - ROBOT_ARM_B_C_TRANSITION_OFFSET
            // from the third rotation Fifth (wrist) rotation is motor 5 at offset
            // ROBOT_ARM_C_LEN - ROBOT_ARM_C_WRIST_TRANSITION from the fourth rotation

            break;
        default:
            logic_fault();
            break;
    }
}

/*!
 * \brief Calculate if we can go to the given position, and begin movement if it is possible
 *
 * \param target The target point we want to move towards
 *
 * \returns 1 if movement started, 0 if cannot reach the target point
 */
int position_control_set_target_position(struct Point target) {
    // Calculate the ik angles of the target position
    if (!position_control_inverse_kinematics(target, &target_angles)) {
        // Cannot reach the target position
        return 0;
    }
    else{
        // We can get to the target. Set the target point and begin the movement timer.
        target_position = target;
        hal_movement_timer_enable();
        return 1;
    }
}

/*!
 * \brief Check our commanded and target angles, reporting if they are the same or different
 *
 * \return 1 if the same, 0 if different
 */
int position_control_check_angles() { return !memcmp(&current_angles, &target_angles, sizeof(current_angles)); }

/*!
 * \brief Step all motors towards the specified position at a maximum speed. Does not move if
 * the position is unreachable or IK fails.
 *
 * \param p The {x,y,z} point we want our robot to reach
 *
 * \returns 1 if stepped successfully, 0 if failed
 */
int position_control_step_towards_position() {
    // Determine if we need the joint to move in the positive or negative
    // directions
    float theta1_diff = target_angles.theta1 - current_angles.theta1;
    float theta2_diff = target_angles.theta2 - current_angles.theta2;
    float theta3_diff = target_angles.theta3 - current_angles.theta3;
    float theta4_diff = target_angles.theta4 - current_angles.theta4;

    if (theta1_diff == 0) {
        // We are at the target angle, skip ahead
    } else if (fabsf(theta1_diff) < MAX_ANGULAR_STEP) {
        // Move to the final position
        position_control_set_motor_angle(MOTOR1, target_angles.theta1);
    } else if (theta1_diff > 0) {
        position_control_set_motor_angle(MOTOR1, current_angles.theta1 - MAX_ANGULAR_STEP);
    } else { // (theta1_diff < 0)
        position_control_set_motor_angle(MOTOR1, current_angles.theta1 + MAX_ANGULAR_STEP);
    }

    if (theta2_diff == 0) {
        // We are at the target angle, skip ahead
    } else if (fabsf(theta2_diff) < MAX_ANGULAR_STEP) {
        // Move to the final position
        position_control_set_motor_angle(MOTOR2, target_angles.theta2);
    } else if (theta2_diff > 0) {
        position_control_set_motor_angle(MOTOR2, current_angles.theta2 - MAX_ANGULAR_STEP);
    } else { // (theta2_diff < 0)
        position_control_set_motor_angle(MOTOR2, current_angles.theta2 + MAX_ANGULAR_STEP);
    }

    if (theta3_diff == 0) {
        // We are at the target angle, skip ahead
    } else if (fabsf(theta3_diff) < MAX_ANGULAR_STEP) {
        // Move to the final position
        position_control_set_motor_angle(MOTOR3, target_angles.theta3);
    } else if (theta3_diff > 0) {
        position_control_set_motor_angle(MOTOR3, current_angles.theta3 - MAX_ANGULAR_STEP);
    } else { // (theta3_diff < 0)
        position_control_set_motor_angle(MOTOR3, current_angles.theta3 + MAX_ANGULAR_STEP);
    }

    if (theta4_diff == 0) {
        // We are at the target angle, skip ahead
    } else if (fabsf(theta4_diff) < MAX_ANGULAR_STEP) {
        // Move to the final position
        position_control_set_motor_angle(MOTOR4, target_angles.theta4);
    } else if (theta4_diff > 0) {
        position_control_set_motor_angle(MOTOR4, current_angles.theta4 - MAX_ANGULAR_STEP);
    } else { // (theta4_diff < 0)
        position_control_set_motor_angle(MOTOR4, current_angles.theta4 + MAX_ANGULAR_STEP);
    }

    // Calculate the current position from the current angles
    position_control_forward_kinematics(current_angles, &current_position);

    // Successfully stepped
    return 1;
}

void position_control_move_to_start() {}
