#include <math.h>
#include <stdlib.h>
#include <string.h> // For memcmp()
#include <stdint.h>

#include "hal.h"
#include "position_control.h"

// For now this is defined in the C code, but eventually should be moved to a
// "settings" section of ROM
enum robot_type robot_type = ROBOT_TYPE_V1;

/// \def Distance constants for the different joints. All values are in
/// centimeters. D12 is 0 because we consider Joint 1 and Joint 2 to exist
/// at the same point (the origin) to simplify the math.
#define d12 0.0f  // Distance from Joint 1 to Joint 2
#define d23 30.0f // Distance from Joint 2 to Joint 3
#define d34 30.0f // Distance from Joint 3 to Joint 4
#define d45 30.0f // Distance from Joint 4 to Joint 5 / initial endpoint

// Array containing the distances between the joints and end effector
float joint_distances[NUM_JOINTS] = {d12, d23, d34, d45};

/// \def Shorthand macro for squaring a number
#define SQUARE(X) X*X

/// \def pi to as many places as I care about
#define PI 3.14159265f

#define MAX_J_ANGLE (float)(PI * 2.0f / 3.0f)

/// \def Timeout for the IK calculation in milliseconds
#define IK_TIMEOUT 1000
/// \def Maximum iterations allowed for the IK calculation
#define IK_MAX_ITERATIONS 10000

// Maximum position error tolerance in centimeters
#define POSITION_ERROR_TOLERANCE 0.1f

// Maximum length of the arm
const float MAX_ARM_LENGTH = d12 + d23 + d34 + d45;

// Starting points for the joints
const struct Point origin = {0, 0, 0};
const struct Point j1_start = origin;
const struct Point j2_start = {d12, 0, 0};
const struct Point j3_start = {d12 + d23, 0, 0};
const struct Point j4_start = {d12 + d23 + d34, 0, 0};
const struct Point end_start = {d12 + d23 + d34 + d45, 0, 0};

// Positions of the joints. Joint[0] is always at the origin.
struct Point joint_positions[NUM_JOINTS_AND_END] = {
    j1_start, j2_start, j3_start, j4_start, end_start
};

// Enum for indexing arrays involving the joints
enum joint_indexes{
    J1 = 0,
    J2 = 1,
    J3 = 2,
    J4 = 3,
    JEND = 4,
};

// Struct containing the current joint angles
float current_angles[NUM_JOINTS];
// Struct containing the target joint angles
float target_angles [NUM_JOINTS];

// Struct containing the current end effector position
struct Point current_position;
// Struct containing the target end effector position
struct Point target_position;

// Frame of reference: {0,0,0} is the center point of Joint 2, directly above
// the rotational Joint 1. All angles being 0 will result in the arm IK being
// completely horizontal

/*!
 * \brief Get the geometric distance between two points
 *
 * \param p1 Point 1
 * \param p2 Point 2
 *
 * \returns Distance between the two points
 */
float position_control_point_distance(struct Point p1, struct Point p2) {
    return (float)sqrtf(SQUARE((p1.x - p2.x)) + SQUARE((p1.y - p2.y)) + SQUARE((p1.z - p2.z)));
}

/*!
 * \brief Get the geometric distance between two points
 *
 * \param p1 Point 1, the vertex
 * \param p2 Point 2, an endpoint
 * \param p3 Point 3, an endpoint
 *
 * \returns The angle (in radians) between the three points
 */
float position_control_angle_between_points(struct Point p1, struct Point p2, struct Point p3) {
    // Create vectors from p1->p2 and p1->p3
    // These are vectors even though they use the Point struct type. Reusing so I don't have to make an
    // identical struct.
    struct Point v12 = {p1.x - p2.x, p1.y - p2.y, p1.z - p2.z};
    struct Point v13 = {p1.x - p3.x, p1.y - p3.y, p1.z - p3.z};

    float dot_product = v12.x * v13.x + v12.y * v13.y + v12.z * v13.z;
    float v12_mag = sqrtf(SQUARE(v12.x) + SQUARE(v12.y) + SQUARE(v12.z));
    float v13_mag = sqrtf(SQUARE(v13.x) + SQUARE(v13.y) + SQUARE(v13.z));

    return acosf(dot_product / (v12_mag * v13_mag));
}

/*!
 * \brief Perform Forwards And Backwards Reaching Inverse Kinematics (FABRIK) to
 * achieve the joint angles necessary for the manipulator to reach a specified point.
 *
 * \param target The target point we wish to reach
 * \param angles_out Array that the generated angles will be placed. If the target is unreachable or
 * a failure occurs, the array will not be modified.
 *
 * \note Generates the new joint positions?
 * \note di = distance between joint i and i+1
 *
 * \returns 1 if successful, 0 if failed
 */
struct Point target_joint_positions[NUM_JOINTS_AND_END]; // +1 for end effector
float ik_angles[NUM_JOINTS];

int position_control_fabrik(struct Point target, float angles_out[]) {
    // Check whether the target is within reachable distance of the root
    float distance = position_control_point_distance(target, j1_start);
    if (distance > MAX_ARM_LENGTH || target.z < 0) {
        // The target is unreachable
        return 0;
    }

    // Create a temporary array to calculate the joint positions
    //struct Point target_joint_positions[NUM_JOINTS_AND_END]; // +1 for end effector
    for (size_t i = J1; i <= JEND; i++) {
        target_joint_positions[i] = joint_positions[i];
    }


    // Loop through the IK Heuristic until we find a way to reach the point or time out
    uint32_t iteration_count = 0;
    uint32_t ik_timeout = hal_get_systick_counter() + IK_TIMEOUT;
    while(1){
        iteration_count++;
        // Set b = p1, initial position of joint 1
        struct Point b = target_joint_positions[J1];

        // Check whether the distance between the end effector pn and target is greater than a tolerance
        float position_error = position_control_point_distance(target, target_joint_positions[JEND]);

        while (position_error > POSITION_ERROR_TOLERANCE) {
            if (hal_get_systick_counter() > ik_timeout) {
                // We timed out
                counters.ik_timeout_count++;
                return 0;
            }
            else if(iteration_count > IK_MAX_ITERATIONS){
                // We hit the iteration limit
                counters.ik_iteration_overrun_count++;
                return 0;
            }

            // Stage 1, Forward Reaching
            // Set the end effector position as target
            target_joint_positions[JEND] = target;
            for (int i = JEND-1; i >= J1; i--) {
                // Find the distance r between the new joint position Pi+1 and the joint pi
                float r = position_control_point_distance(target_joint_positions[i], target_joint_positions[i + 1]);
                float lambda = joint_distances[i] / r;

                // Find the new joint position i
                target_joint_positions[i].x =
                    (1.0f - lambda) * target_joint_positions[i + 1].x + lambda * target_joint_positions[i].x;
                target_joint_positions[i].y =
                    (1.0f - lambda) * target_joint_positions[i + 1].y + lambda * target_joint_positions[i].y;
                target_joint_positions[i].z =
                    (1.0f - lambda) * target_joint_positions[i + 1].z + lambda * target_joint_positions[i].z;
            }

            // Stage 2, Backwards reaching!
            // Set the root p1 as the initial position (p1 = b)
            target_joint_positions[0] = b;
            for (int i = J1; i < JEND; i++) {
                // Find the distance ri between the new joint position pi and the joint pi+1
                float r = position_control_point_distance(target_joint_positions[i], target_joint_positions[i + 1]);
                float lambda = joint_distances[i] / r;

                // Find the new joint position i + 1
                target_joint_positions[i + 1].x =
                    (1.0f - lambda) * target_joint_positions[i].x + lambda * target_joint_positions[i + 1].x;
                target_joint_positions[i + 1].y =
                    (1.0f - lambda) * target_joint_positions[i].y + lambda * target_joint_positions[i + 1].y;
                target_joint_positions[i + 1].z =
                    (1.0f - lambda) * target_joint_positions[i].z + lambda * target_joint_positions[i + 1].z;
            }

            // Calculate the position error from the target
            position_error = position_control_point_distance(target, target_joint_positions[JEND]);
        }

        for(size_t i = 0; i<sizeof(target_joint_positions)/sizeof(target_joint_positions[0]); i++){
            if(isnan(target_joint_positions[i].x) || isnan(target_joint_positions[i].y) || isnan(target_joint_positions[i].z)){
                counters.ik_nan_position++;
                return 0;
            }
        }

        // If we reached this point, we found a set of joint positions that can reach the target
        // Extrapolate the joint angles from the positions
        //float ik_angles[NUM_JOINTS];

        // Joint 1 (Rotary base) and Joint 2 (First Hinge) are always at the origin.
        // Angle of J1 can be calculated simply using an atan calculation of the target point
        ik_angles[J1] = atan2f(target.y, target.x); // This will be the same no matter what

        // Angle of J2 can be extrapolated from right triangle math using the points of J2, J3, and J3 with 0 on the z
        // axis
        struct Point j3_no_z = target_joint_positions[J3];
        j3_no_z.z = 0;
        ik_angles[J2] = position_control_angle_between_points(target_joint_positions[J2],
                                                                 target_joint_positions[J3], j3_no_z);

        // Angle of J3 onwards can be extrapolated from the point of the previous joint, the current joint, and the next
        ik_angles[J3] = PI - position_control_angle_between_points(target_joint_positions[J3],
                                                                      target_joint_positions[J4],
                                                                      target_joint_positions[J2]);
        ik_angles[J4] = PI - position_control_angle_between_points(target_joint_positions[J4],
                                                                      target_joint_positions[J3],
                                                                      target_joint_positions[JEND]);

        // This variable will toggle high if any of our angles are bad.
        bool bad_angle = 0;

        // Check if any of the angles are beyond the driving range
        for(size_t i = 0; i < NUM_JOINTS-1; i++){
            if(ik_angles[i] < 0){
                // We have a negative angle, which isn't allowed for any joint
                // Set the offending joint angle to 0 and recalculate the joint positions
                ik_angles[i] = 0;

                // Flag that we had a bad angle
                bad_angle = 1;

                counters.ik_negative_angle_count++;
            }
            else if(ik_angles[i] > MAX_J_ANGLE){
                // We have a motor angle that goes beyond the driveable range
                // Set the offending joint angle to half its current angle and recalculate the joint positions
                ik_angles[i] = ik_angles[i] / 2.0f;

                // Flag that we had a bad angle
                bad_angle = 1;

                counters.ik_too_large_angle_count++;
            }
        }

        if(bad_angle){
            // Recalculate the potential arm joint positions from the fake angles and attempt to iterate again
            position_control_joint_forward_kinematics(ik_angles, target_joint_positions);
        }
        else{
            // We completed the IK!
            // Copy out the joint angles
            for(size_t i = 0; i < NUM_JOINTS; i++){
                angles_out[i] = ik_angles[i];
            }
            counters.ik_success_count++;

            // Return 1 for success
            return 1;
        }
    }
}

float position_control_get_angle_from_duty_cycle(enum motor_num motor) {
    // Need to confirm accuracy of this!!
    return ((MAX_J_ANGLE) * motor_duty_cycle[motor - 1] / 100.0f);
}

void position_control_set_motor_angle(enum motor_num motor, float angle) {
    if (angle > MAX_J_ANGLE) {
        // Cannot set it above 120 degrees
        logic_fault();
    }

    switch (motor) {
        case MOTOR1:
        case MOTOR2:
        case MOTOR3:
        case MOTOR4:
            current_angles[motor] = angle;
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

    float duty_cycle = (angle / MAX_J_ANGLE) * 100;
    hal_set_motor_duty_cycle(motor, duty_cycle);
}

/*!
 * \brief Calculate the current end effector position via forward kinematics
 *
 * \param angles The joint configuration angles
 * \param position Point of the end effector location
 */
void position_control_forward_kinematics(float angles[], struct Point *position) {
    switch (robot_type) {
        case ROBOT_TYPE_V1: {
            // Shorthand angle sums
            float angles23 = angles[J2] + angles[J3];
            float angles234 = angles[J2] + angles[J3] + angles[J4];

            // Magnitude of distance in the xy plane
            float xy_plane_distance = d23 * cosf(angles[J2]) + d34 * cosf(angles23) + d45 * cosf(angles234);

            // Calculate the position
            position->x = cosf(angles[J1]) * xy_plane_distance;
            position->y = sinf(angles[J1]) * xy_plane_distance;
            position->z = d23 * sinf(angles[J2]) + d34 * sinf(angles23) + d45 * sinf(angles234);
            break;
        }
        case ROBOT_TYPE_V2:
            // This robot type includes the wrist! Update the forward kinematics when we
            // get here.

            // This can likely just be two separate calculations:
            // First is up to the point of the wrist found in type V1
            // Second is from the start of the wrist to the end effector

            break;
        default:
            logic_fault();
            break;
    }
}

/*!
 * \brief Calculate the current end effector position via forward kinematics
 *
 * \param angles The joint configuration angles
 * \param fw_joint_positions The calculated positions of every joint and the end effector
 */
void position_control_joint_forward_kinematics(float angles[], struct Point fw_joint_positions[]){
    switch (robot_type) {
        case ROBOT_TYPE_V1: {
            // Shorthand angle sums
            float angles23 = angles[J2] + angles[J3];
            float angles234 = angles[J2] + angles[J3] + angles[J4];

            // Magnitude of distance in the xy plane for each joint
            float xy_plane_distance_j3 = d23 * cosf(angles[J2]);
            float xy_plane_distance_j4 = d23 * cosf(angles[J2]) + d34 * cosf(angles23);
            float xy_plane_distance_jend = d23 * cosf(angles[J2]) + d34 * cosf(angles23) + d45 * cosf(angles234);

            // Calculate the positions
            fw_joint_positions[J1] = origin; // J1 and J2 are defined as the origin
            fw_joint_positions[J2] = origin;

            fw_joint_positions[J3].x = cosf(angles[J1]) * xy_plane_distance_j3;
            fw_joint_positions[J3].y = sinf(angles[J1]) * xy_plane_distance_j3;
            fw_joint_positions[J3].z = d23 * sinf(angles[J2]);

            fw_joint_positions[J4].x = cosf(angles[J1]) * xy_plane_distance_j4;
            fw_joint_positions[J4].y = sinf(angles[J1]) * xy_plane_distance_j4;
            fw_joint_positions[J4].z = d23 * sinf(angles[J2]) + d34 * sinf(angles23);

            fw_joint_positions[JEND].x = cosf(angles[J1]) * xy_plane_distance_jend;
            fw_joint_positions[JEND].y = sinf(angles[J1]) * xy_plane_distance_jend;
            fw_joint_positions[JEND].z = d23 * sinf(angles[J2]) + d34 * sinf(angles23) + d45 * sinf(angles234);
            break;
        }
        case ROBOT_TYPE_V2:
            // This robot type includes the wrist! Update the forward kinematics when we
            // get here.

            // This can likely just be two separate calculations:
            // First is up to the point of the wrist found in type V1
            // Second is from the start of the wrist to the end effector

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
 * \returns 1 if successful and movement towards target started, 0 if cannot reach the target point
 */
int position_control_set_target_position(struct Point target) {
    // Calculate the ik angles of the target position
    if (!position_control_fabrik(target, target_angles)) {
        // Cannot reach the target position
        return 0;
    } else {
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
int position_control_check_angles() {
    hal_compiler_barrier();
    for(size_t i = 0; i < sizeof(current_angles)/sizeof(current_angles[0]); i++){
        if(current_angles[i] != target_angles[i]){
            return 0;
        }
    }
    return 1;
    //return !memcmp(current_angles, target_angles, sizeof(current_angles));
}

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

    for(size_t i = 0; i < sizeof(current_angles)/sizeof(current_angles[0]); i++){
        // Get the difference in target and current joint angles
        float angle_diff = target_angles[i] - current_angles[i];
        if(angle_diff == 0){
            // We are at the target angle for this joint
            continue;
        } else if (fabsf(angle_diff) < MAX_ANGULAR_STEP) {
            // We are within one step; set the angle to the final position
            position_control_set_motor_angle(i, target_angles[i]);
        } else if (angle_diff > 0) {
            position_control_set_motor_angle(i, current_angles[i] + MAX_ANGULAR_STEP);
        } else { // (angle_diff < 0)
            position_control_set_motor_angle(i, current_angles[i] - MAX_ANGULAR_STEP);
        }
    }

    // Calculate the current position from the current angles
    position_control_forward_kinematics(current_angles, &current_position);

    // Successfully stepped
    return 1;
}

void position_control_move_to_start() {}
